import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from hexapod_msgs.msg import LegPositions, LegCommands, BoolArray
import pytest
import time


class _TestStaticBalancingNode(Node):
    def __init__(self):
        super().__init__('test_static_balancing')

        # Publishers
        self.pub_com = self.create_publisher(Point, '/center_of_mass', 10)
        self.pub_margin = self.create_publisher(Float64, '/stability_margin', 10)
        self.pub_centroid = self.create_publisher(Point, '/polygon_centroid', 10)
        self.pub_foot_positions = self.create_publisher(LegPositions, '/foot_positions', 10)
        self.pub_foot_status = self.create_publisher(BoolArray, '/foot_status', 10)

        # Subscriber to validate outputs
        self.sub_leg_commands = self.create_subscription(
            LegCommands, '/leg_commands', self.leg_commands_callback, 10
        )

        # Test State
        self.received_leg_commands = None
        self.test_complete = False

        # Timer to publish test data
        self.timer = self.create_timer(1.0, self.publish_test_data)

    def publish_test_data(self):
        try:
            # Mock data for publishing
            com = Point(x=0.1, y=-0.05, z=0.0)
            centroid = Point(x=0.0, y=0.0, z=0.0)
            margin = Float64(data=-0.02)  # Unstable condition
            foot_positions = LegPositions()
            foot_positions.positions = [
                Point(x=0.2, y=0.1, z=0.0),
                Point(x=-0.2, y=0.1, z=0.0),
                Point(x=0.0, y=-0.2, z=0.0),
            ]
            foot_status = BoolArray()
            foot_status.data = [True, True, True]  # All feet grounded

            # Publish data with logs
            self.pub_com.publish(com)
            self.get_logger().info(f"Published CoM: {com}")

            self.pub_margin.publish(margin)
            self.get_logger().info(f"Published Stability Margin: {margin}")

            self.pub_centroid.publish(centroid)
            self.get_logger().info(f"Published Centroid: {centroid}")

            self.pub_foot_positions.publish(foot_positions)
            self.get_logger().info(f"Published Foot Positions: {foot_positions}")

            self.pub_foot_status.publish(foot_status)
            self.get_logger().info(f"Published Foot Status: {foot_status}")
        except Exception as e:
            self.get_logger().error(f"Error in publish_test_data: {e}")

    def leg_commands_callback(self, msg):
        try:
            # Store received leg commands for validation
            self.received_leg_commands = msg
            self.test_complete = True
            self.get_logger().info(f"Received leg commands: {msg}")
        except Exception as e:
            self.get_logger().error(f"Error in leg_commands_callback: {e}")


def run_node_for_test():
    rclpy.init()
    node = _TestStaticBalancingNode()

    try:
        # Spin until test is complete or timeout
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        start_time = time.time()
        while not node.test_complete and time.time() - start_time < 30.0:  # Extended timeout to 30 seconds
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    return node.received_leg_commands


@pytest.mark.rostest
def test_static_balancing():
    received_commands = run_node_for_test()

    # Expected results
    expected_adjustments = [
        (0.30000000000000004, 0.2, 0.0),
        (-0.1, 0.2, 0.0),
        (0.1, -0.1, 0.0),
    ]

    # Validate received leg commands
    assert received_commands is not None, "No leg commands received!"
    for i, position in enumerate(received_commands.positions):
        adjusted_position = (position.x, position.y, position.z)
        print(f"Leg {i}: Adjusted Position: {adjusted_position}")
        assert (
            abs(adjusted_position[0] - expected_adjustments[i][0]) < 1e-2 and
            abs(adjusted_position[1] - expected_adjustments[i][1]) < 1e-2
        ), f"Position {i} is incorrect: {adjusted_position}"
