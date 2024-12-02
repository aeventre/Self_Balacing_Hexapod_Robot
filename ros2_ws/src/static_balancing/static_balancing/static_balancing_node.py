import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from hexapod_msgs.msg import LegPositions, LegCommands, BoolArray


class StaticBalancingNode(Node):
    def __init__(self):
        super().__init__('static_balancing_node')

        # Initialize internal state variables
        self.com = None
        self.margin = None
        self.centroid = None
        self.foot_positions = None
        self.foot_status = None

        # Publishers
        self.pub_leg_commands = self.create_publisher(LegCommands, '/leg_commands', 10)

        # Subscribers
        self.create_subscription(Point, '/center_of_mass', self.com_callback, 10)
        self.create_subscription(Float64, '/stability_margin', self.margin_callback, 10)
        self.create_subscription(Point, '/polygon_centroid', self.centroid_callback, 10)
        self.create_subscription(LegPositions, '/foot_positions', self.foot_positions_callback, 10)
        self.create_subscription(BoolArray, '/foot_status', self.foot_status_callback, 10)

        # Timer to periodically adjust foot positions
        self.timer = self.create_timer(1.0, self.timer_callback)

    def com_callback(self, msg):
        self.com = (msg.x, msg.y, msg.z)
        self.get_logger().info(f"Received CoM: {self.com}")

    def margin_callback(self, msg):
        self.margin = msg.data
        self.get_logger().info(f"Received Stability Margin: {self.margin}")

    def centroid_callback(self, msg):
        self.centroid = (msg.x, msg.y, msg.z)
        self.get_logger().info(f"Received Centroid: {self.centroid}")

    def foot_positions_callback(self, msg):
        self.foot_positions = [(p.x, p.y, p.z) for p in msg.positions]
        self.get_logger().info(f"Received Foot Positions: {self.foot_positions}")

    def foot_status_callback(self, msg):
        self.foot_status = msg.data
        self.get_logger().info(f"Received Foot Status: {self.foot_status}")

    def timer_callback(self):
        try:
            self.adjust_foot_positions()
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    def adjust_foot_positions(self):
        # Log missing data before returning
        if None in (self.com, self.centroid, self.margin, self.foot_positions, self.foot_status):
            self.get_logger().warn(
                f"Waiting for necessary data...\n"
                f"  CoM: {self.com}\n"
                f"  Centroid: {self.centroid}\n"
                f"  Margin: {self.margin}\n"
                f"  Foot Positions: {self.foot_positions}\n"
                f"  Foot Status: {self.foot_status}"
            )
            return

        # Log valid data
        self.get_logger().info(
            f"Adjusting foot positions with data:\n"
            f"  CoM: {self.com}\n"
            f"  Centroid: {self.centroid}\n"
            f"  Margin: {self.margin}\n"
            f"  Foot Positions: {self.foot_positions}\n"
            f"  Foot Status: {self.foot_status}"
        )

        # Skip adjustments if stable
        if self.margin > 0.01:
            self.get_logger().info("Robot is stable, no adjustment needed.")
            return

        # Adjust foot positions
        adjusted_positions = []
        for i, (foot_x, foot_y, foot_z) in enumerate(self.foot_positions):
            if not self.foot_status[i]:
                adjusted_positions.append((foot_x, foot_y, foot_z))  # Skip feet not in contact
                continue

            # Example adjustment logic
            new_x = foot_x + 0.1
            new_y = foot_y + 0.1
            new_z = foot_z
            adjusted_positions.append((new_x, new_y, new_z))

        # Publish leg commands
        leg_commands = LegCommands()
        for pos in adjusted_positions:
            leg_commands.positions.append(Point(x=pos[0], y=pos[1], z=pos[2]))

        self.pub_leg_commands.publish(leg_commands)
        self.get_logger().info(f"Published leg commands: {leg_commands}")


def main(args=None):
    rclpy.init(args=args)
    node = StaticBalancingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
