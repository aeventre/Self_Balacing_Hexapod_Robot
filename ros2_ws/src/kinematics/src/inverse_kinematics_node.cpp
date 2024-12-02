#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "Hexapod.hpp"

class InverseKinematicsNode : public rclcpp::Node {
public:
    InverseKinematicsNode() : Node("inverse_kinematics_node") {
        leg_commands_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "leg_commands", 10, std::bind(&InverseKinematicsNode::legCommandsCallback, this, std::placeholders::_1));
        joint_angles_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_angles", 10);

        RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Node initialized.");
    }

private:
    void legCommandsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received leg_commands with %zu elements.", msg->data.size());
        if (msg->data.size() != 18) {
            RCLCPP_ERROR(this->get_logger(), "Invalid leg_commands size. Expected 18 elements.");
            return;
        }

        std::vector<std::vector<float>> targets;
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            targets.emplace_back(std::vector<float>{msg->data[i], msg->data[i + 1], msg->data[i + 2]});
        }

        RCLCPP_INFO(this->get_logger(), "Parsed targets:");
        for (size_t i = 0; i < targets.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Leg[%zu]: [%f, %f, %f]", i, targets[i][0], targets[i][1], targets[i][2]);
        }

        try {
            auto joint_angles = hexapod_.computeInverseKinematics(targets);

            std_msgs::msg::Float32MultiArray joint_angles_msg;
            for (const auto& angles : joint_angles) {
                joint_angles_msg.data.insert(joint_angles_msg.data.end(), angles.begin(), angles.end());
            }

            joint_angles_pub_->publish(joint_angles_msg);
            RCLCPP_INFO(this->get_logger(), "Published joint angles.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error computing inverse kinematics: %s", e.what());
        }
    }

    Hexapod hexapod_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr leg_commands_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_angles_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
