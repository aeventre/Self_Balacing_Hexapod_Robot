#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "Hexapod.hpp"

class ForwardKinematicsNode : public rclcpp::Node {
public:
    ForwardKinematicsNode() : Node("forward_kinematics_node") {
        // Subscription to joint angles topic
        joint_angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "joint_angles", 10, std::bind(&ForwardKinematicsNode::computeFootPositions, this, std::placeholders::_1));
        
        // Publisher for foot positions
        foot_positions_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("foot_positions", 10);

        RCLCPP_INFO(this->get_logger(), "Forward Kinematics Node initialized.");
    }

private:
    Hexapod hexapod_; // Hexapod model instance

    // Subscription and Publisher
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_angles_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr foot_positions_pub_;

    void computeFootPositions(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Validate the input size (6 legs, 3 angles per leg)
        if (msg->data.size() != 18) {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint_angles size. Expected 18 elements.");
            return;
        }

        // Parse the joint angles from the message
        std::vector<std::vector<float>> joint_angles(6);
        for (size_t i = 0; i < 6; ++i) {
            joint_angles[i] = {msg->data[i * 3], msg->data[i * 3 + 1], msg->data[i * 3 + 2]};
        }

        // Compute the foot positions using forward kinematics
        auto foot_positions = hexapod_.computeForwardKinematics(joint_angles);

        // Prepare the foot positions message
        std_msgs::msg::Float32MultiArray foot_positions_msg;
        for (const auto& pos : foot_positions) {
            foot_positions_msg.data.insert(foot_positions_msg.data.end(), pos.begin(), pos.end());
        }

        // Publish the computed foot positions
        foot_positions_pub_->publish(foot_positions_msg);

        // Log the foot positions for debugging
        for (size_t i = 0; i < foot_positions.size(); ++i) {
            RCLCPP_INFO(this->get_logger(),
                        "Leg[%lu]: Foot Position: [%.2f, %.2f, %.2f]",
                        i,
                        foot_positions[i][0],
                        foot_positions[i][1],
                        foot_positions[i][2]);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
