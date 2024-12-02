import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class GpioSensorNode(Node):
    def __init__(self):
        super().__init__('gpio_sensor_node')
        self.gpio_pins = [29, 27, 31, 33, 35, 37]
        self.foot_status_pubs = [
            self.create_publisher(Bool, f'foot_status_{pin}', 10) for pin in self.gpio_pins
        ]

        GPIO.setmode(GPIO.BCM)
        for pin in self.gpio_pins:
            GPIO.setup(pin, GPIO.IN)

        self.create_timer(0.5, self.publish_foot_status)

    def publish_foot_status(self):
        for i, pin in enumerate(self.gpio_pins):
            pin_state = GPIO.input(pin) == GPIO.HIGH
            msg = Bool()
            msg.data = pin_state
            self.foot_status_pubs[i].publish(msg)
            self.get_logger().info(f'Published foot status for pin {pin}: {pin_state}')
        
def main(args=None):
    rclpy.init(args=args)
    gpio_sensor_node = GpioSensorNode()

    try:
        rclpy.spin(gpio_sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        gpio_sensor_node.destroy_node()
        GPIO.cleanup()  # Clean up GPIO on shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()

