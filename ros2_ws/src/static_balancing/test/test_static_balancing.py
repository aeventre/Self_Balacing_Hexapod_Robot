import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
from hexapod_msgs.msg import LegPositions, LegCommands

class TestStaticBalancing(Node):
    def __init__(self):
        super().__init__('test_static_balancing')
        
        # Publishers
        self.pub_com = self.create_publisher(Point, '/center_of_mass', 10)
        self.pub_margin = self.create_publisher(Float64, '/stability_margin', 10)
        self.pub_centroid = self.create_publisher(Point, '/polygon_centroid', 10)
        self.pub_foot_positions = self.create_publisher(LegPositions, '/foot_positions', 10)
        self.pub_foot_status = self.create_publisher(Bool, '/foot_status', 10)

        # Subscriber to validate outputs
        self.sub_leg_commands = self.create_subscription(LegCommands, '/leg_commands', self.leg_commands_callback, 10)

        # Test State
        self.received_leg_commands = None

        # Timer to publish test data
        self.timer = self.create_timer(1.0, self.publish_test_data)

    def publish_test_data(self):
        # Mock data
        com = Point(x=0.1, y=-0.05, z=0.0)
        centroid = Point(x=0.0, y=0.0, z=0.0)
        margin = Float64(data=-0.02)  # Unstable
        foot_positions = LegPositions()
        foot_positions.positions = [
            LegPositions.Position(x=0.2, y=0.1, z=0.0),
            LegPositions.Position(x=-0.2, y=0.1, z=0.0),
            LegPositions.Position(x=0.0, y=-0.2, z=0.0),
        ]
        foot_status = Bool(data=[True, True, True])  # All feet grounded

        # Publish data
        self.pub_com.publish(com)
        self.pub_margin.publish(margin)
        self.pub_centroid.publish(centroid)
        self.pub_foot_positions.publish(foot_positions)
        self.pub_foot_status.publish(foot_status)

        self.get_logger().info("Published test data.")

    def leg_commands_callback(self, msg):
        # Validate leg commands
        self.received_leg_commands = msg
        self.get_logger().info(f"Received leg commands: {msg}")

        # Example validation
        expected_adjustments = [
            (0.15, 0.125, 0.0),
            (-0.25, 0.125, 0.0),
            (-0.05, -0.175, 0.0),
        ]
        for i, position in enumerate(msg.positions):
            adjusted_position = (position.x, position.y, position.z)
            assert (
                abs(adjusted_position[0] - expected_adjustments[i][0]) < 1e-2 and
                abs(adjusted_position[1] - expected_adjustments[i][1]) < 1e-2
            ), f"Position {i} is incorrect: {adjusted_position}"

        self.get_logger().info("Leg commands are correct!")

def main(args=None):
    rclpy.init(args=args)
    node = TestStaticBalancing()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down test node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
