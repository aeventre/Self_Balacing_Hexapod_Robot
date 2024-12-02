#include "SpiderLeg.hpp"
#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

// Constructor
SpiderLeg::SpiderLeg(const std::string& name, float coxa, float femur, float tibia, 
                     float origin_x, float origin_y, float origin_z, float rotation_z, bool is_mirrored)
    : name_(name), COXA(coxa), FEMUR(femur), TIBIA(tibia), 
      origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z), 
      rotation_z_(rotation_z), is_mirrored_(is_mirrored) {}

// Forward kinematics: compute foot position from joint angles
std::vector<float> SpiderLeg::forwardKinematics(const std::vector<float>& angles) const {
    if (angles.size() != 3) {
        throw std::invalid_argument("Expected 3 joint angles for forward kinematics.");
    }

    float theta1 = deg2rad(angles[0]);
    float theta2 = deg2rad(angles[1]);
    float theta3 = deg2rad(angles[2]);

    if (is_mirrored_) {
        theta1 = -theta1;
        theta2 = -theta2;
        theta3 = -theta3;
    }

    // Compute position relative to the leg base
    float x_local = (COXA + FEMUR * cos(theta2) + TIBIA * cos(theta2 + theta3)) * cos(theta1);
    float y_local = (COXA + FEMUR * cos(theta2) + TIBIA * cos(theta2 + theta3)) * sin(theta1);
    float z_local = FEMUR * sin(theta2) + TIBIA * sin(theta2 + theta3);

    // Transformations into the global frame
    float x_global = x_local * cos(rotation_z_) - y_local * sin(rotation_z_) + origin_x_;
    float y_global = x_local * sin(rotation_z_) + y_local * cos(rotation_z_) + origin_y_;
    float z_global = z_local + origin_z_;

    return {x_global, y_global, z_global};
}

// Inverse kinematics: compute joint angles from foot position
std::vector<float> SpiderLeg::inverseKinematics(const std::vector<float>& target) const {
    if (target.size() != 3) {
        throw std::invalid_argument("Expected 3 target coordinates for inverse kinematics.");
    }

    // Transform target to local leg frame
    float x_global = target[0] - origin_x_;
    float y_global = target[1] - origin_y_;
    float z_global = target[2] - origin_z_;

    float x_local = x_global * cos(-rotation_z_) - y_global * sin(-rotation_z_);
    float y_local = x_global * sin(-rotation_z_) + y_global * cos(-rotation_z_);
    float z_local = z_global;

    // Compute Coxa angle
    float theta1 = atan2(y_local, x_local);

    // Compute distances
    float horizontal_distance = sqrt(x_local * x_local + y_local * y_local) - COXA;
    float H = sqrt(horizontal_distance * horizontal_distance + z_local * z_local);

    // Check for reachability
    if (H > (FEMUR + TIBIA)) {
        RCLCPP_WARN(rclcpp::get_logger("SpiderLeg"),
                    "[%s] Target out of reach. Clamping to maximum reachable distance.",
                    name_.c_str());
        H = FEMUR + TIBIA - 1e-3; // Slightly reduce to avoid numerical issues
    }

    // Compute angles
    float phi1 = atan2(z_local, horizontal_distance);
    float phi2 = acos((FEMUR * FEMUR + H * H - TIBIA * TIBIA) / (2 * FEMUR * H));
    float phi3 = acos((FEMUR * FEMUR + TIBIA * TIBIA - H * H) / (2 * FEMUR * TIBIA));

    float theta2 = phi1 + phi2;
    float theta3 = M_PI - phi3;

    // Mirror adjustments
    if (is_mirrored_) {
        theta1 = -theta1;
        theta2 = -theta2;
        theta3 = -theta3;
    }

    return {rad2deg(theta1), rad2deg(theta2), rad2deg(theta3)};
}

// Utility: Convert radians to degrees
float SpiderLeg::rad2deg(float radians) const {
    return radians * 180.0 / M_PI;
}

// Utility: Convert degrees to radians
float SpiderLeg::deg2rad(float degrees) const {
    return degrees * M_PI / 180.0;
}

// Accessor for leg name
std::string SpiderLeg::getName() const {
    return name_;
}
