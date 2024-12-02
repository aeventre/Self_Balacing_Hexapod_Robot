import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class FootSensorNode(Node):
    def __init__(self):
        super().__init__('foot_sensor_node')
        self.gpio_pins = [29, 27, 31, 33, 35, 37]
        self.foot_status_pub = self.create_publisher(Bool, 'foot_status', 10)

        GPIO.setmode(GPIO.BCM)
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.IN)

        self.create_timer(0.5, self.publisj_foot_status)

    def publish_foot_status(self):

        foot_status = []
        for pin in self.gpio_pins:
            foot_status.append(GPIO.input(pin) == GPIO.HIGH)

        msg = Bool()
        msg.data = foot_status
        self.foot_status_pub.publish(msg)
        self.get_logger().info(f'Published foot status: {foot_status}')

def main(args=None):
    rclpy.init(args=args)
    foot_sensor_node = FootSensorNode()
    rclpy.spin(foot_sensor_node)
    foot_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
