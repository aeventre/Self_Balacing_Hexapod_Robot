#include "Hexapod.hpp"
#include <stdexcept>
#include <cmath>
#include <vector>
#include <iostream>

// Constructor
Hexapod::Hexapod() {
    // Initialize the six legs with their specific parameters
    legs_.emplace_back("Leg0", 44.0, 80.0, 80.0, -90.0, -65.0, -35.0, -M_PI * 3 / 4, false); // Leg 0
    legs_.emplace_back("Leg1", 44.0, 80.0, 80.0, 0.0, -90.0, -35.0, -M_PI / 4, false);       // Leg 1
    legs_.emplace_back("Leg2", 44.0, 80.0, 80.0, 90.0, -65.0, -35.0, M_PI * 3 / 4, true);    // Leg 2
    legs_.emplace_back("Leg3", 44.0, 80.0, 80.0, 90.0, 65.0, -35.0, M_PI / 4, true);         // Leg 3
    legs_.emplace_back("Leg4", 44.0, 80.0, 80.0, 0.0, 90.0, -35.0, -M_PI / 4, true);         // Leg 4
    legs_.emplace_back("Leg5", 44.0, 80.0, 80.0, -90.0, 65.0, -35.0, -M_PI * 3 / 4, true);   // Leg 5
}

// Compute forward kinematics for all legs
std::vector<std::vector<float>> Hexapod::computeForwardKinematics(const std::vector<std::vector<float>>& joint_angles) const {
    if (joint_angles.size() != 6) {
        throw std::invalid_argument("Expected joint angles for six legs.");
    }

    std::vector<std::vector<float>> positions(6);
    for (size_t i = 0; i < legs_.size(); ++i) {
        positions[i] = legs_[i].forwardKinematics(joint_angles[i]);
    }
    return positions;
}

// Compute inverse kinematics for all legs
std::vector<std::vector<float>> Hexapod::computeInverseKinematics(const std::vector<std::vector<float>>& target_positions) const {
    if (target_positions.size() != 6) {
        throw std::invalid_argument("Expected target positions for six legs.");
    }

    std::vector<std::vector<float>> joint_angles(6);
    for (size_t i = 0; i < legs_.size(); ++i) {
        joint_angles[i] = legs_[i].inverseKinematics(target_positions[i]);
    }
    return joint_angles;
}

// Accessor for legs
const std::vector<SpiderLeg>& Hexapod::getLegs() const {
    return legs_;
}
