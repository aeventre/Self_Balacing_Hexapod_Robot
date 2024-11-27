#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class StaticBalancer : public rclcpp::Node
{
public:
    StaticBalancer()
        : Node("static_balancer")
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_data", std::bind(&StaticBalancer::imuCallback, this, std:placeholders::_1));

        servo_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("servo_commands");

        RCLCPP_INFO(this->get_logger(), "Static Balancer Node Initialized");

    }
private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto orientation = msg->orientation;
        std_msgs::msg::Float32MultiArray joint_angles;
        joint_angles.data = computeJointAngles(orientation);

        servo_publisher_->publish(joint_angles);

        RCLCPP_INFO(this->get_logger(), "Published joint angles to servo_commands");

    }

    std::vector<float> computeJointAngles(const geometry_msgs::msg::Quaternion &orientation)
    {
        float roll = atan2(2.0 * (orientation.w * orientation.x + orientation.y * orientation.z), 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y));
        float pitch = asin(2.0 * (orientation.w * orientation.y - orientation.z * orientation.x));
        float yaw = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

        std::vector<float> angles(18, 90.0);
        for (int i = 0; i < angles.size(); ++i)
        {
            angles[i] += roll * 10;
            angles[i] -= pitch * 10;
        }

        return angles;
    
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>:SharedPtr servo_publisher_;

};

int main(int argc, char **argv)
(
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticBalancer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
)