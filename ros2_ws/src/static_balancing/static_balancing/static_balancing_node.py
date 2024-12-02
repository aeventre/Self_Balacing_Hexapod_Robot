import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool
from hexapod_msgs.msg import LegPositions, LegCommands

class StaticBalancingNode(Node):
    def __init__(self):
        super().__init__('static_balancing_node')

        # Parameters
        self.k_scaling = self.declare_parameter('k_scaling', 0.5).value

        # Subscribers
        self.sub_com = self.create_subscription(Point, '/center_of_mass', self.com_callback, 10)
        self.sub_stability_margin = self.create_subscription(Float64, '/stability_margin', self.margin_callback, 10)
        self.sub_polygon_centroid = self.create_subscription(Point, '/polygon_centroid', self.centroid_callback, 10)
        self.sub_foot_positions = self.create_subscription(LegPositions, '/foot_positions', self.foot_positions_callback, 10)
        self.sub_foot_status = self.create_subscription(Bool, '/foot_status', self.foot_status_callback, 10)

        # Publisher
        self.pub_leg_commands = self.create_publisher(LegCommands, '/leg_commands', 10)

        # State Variables
        self.com = None
        self.centroid = None
        self.margin = None
        self.foot_positions = None
        self.foot_status = None

    def com_callback(self, msg):
        self.com = (msg.x, msg.y, msg.z)

    def margin_callback(self, msg):
        self.margin = msg.data

    def centroid_callback(self, msg):
        self.centroid = (msg.x, msg.y, msg.z)

    def foot_positions_callback(self, msg):
        self.foot_positions = [(foot.x, foot.y, foot.z) for foot in msg.positions]

    def foot_status_callback(self, msg):
        self.foot_status = msg.data

    def adjust_foot_positions(self):
        # Ensure all necessary data is available
        if None in (self.com, self.centroid, self.margin, self.foot_positions, self.foot_status):
            self.get_logger().warn("Waiting for necessary data...")
            return

        # Check stability margin
        if self.margin > 0.01:  # Replace 0.01 with your stability threshold
            self.get_logger().info("Stable, no adjustment needed.")
            return

        # Compute CoM adjustment
        delta_x = self.centroid[0] - self.com[0]
        delta_y = self.centroid[1] - self.com[1]

        # Adjust foot positions only for grounded feet
        adjusted_positions = []
        for i, (foot_x, foot_y, foot_z) in enumerate(self.foot_positions):
            if not self.foot_status[i]:
                adjusted_positions.append((foot_x, foot_y, foot_z))  # Skip non-grounded feet
                continue

            # Apply scaling to shift foot positions for CoM adjustment
            new_x = foot_x + self.k_scaling * delta_x
            new_y = foot_y + self.k_scaling * delta_y
            adjusted_positions.append((new_x, new_y, foot_z))

        # Publish adjusted positions
        self.publish_leg_commands(adjusted_positions)

    def publish_leg_commands(self, positions):
        # Create and populate LegCommands message
        msg = LegCommands()
        for pos in positions:
            command = LegPositions.Position()
            command.x, command.y, command.z = pos
            msg.positions.append(command)
        self.pub_leg_commands.publish(msg)

    def timer_callback(self):
        self.adjust_foot_positions()

def main(args=None):
    rclpy.init(args=args)
    node = StaticBalancingNode()

    # Create a timer to periodically adjust foot positions
    timer_period = 0.1  # 10 Hz
    node.create_timer(timer_period, node.timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down static balancing node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
