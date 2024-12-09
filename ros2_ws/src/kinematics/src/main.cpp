#include <iostream>
#include <vector>
#include "Hexapod.hpp"

void testForwardKinematics() {
    Hexapod hexapod;

    // Example joint angles for all legs [coxa, femur, tibia]
    std::vector<std::vector<float>> joint_angles = {
        {90.0f, 90.0f, 90.0f}, // Leg 0
        {90.0f, 90.0f, 90.0f}, // Leg 1
        {90.0f, 90.0f, 90.0f}, // Leg 2
        {90.0f, 90.0f, 90.0f}, // Leg 3
        {90.0f, 90.0f, 90.0f}, // Leg 4
        {90.0f, 90.0f, 90.0f}  // Leg 5
    };

    try {
        auto foot_positions = hexapod.computeForwardKinematics(joint_angles);
        std::cout << "Forward Kinematics Results:\n";
        for (size_t i = 0; i < foot_positions.size(); ++i) {
            std::cout << "Leg " << i << ": Position ["
                      << foot_positions[i][0] << ", "
                      << foot_positions[i][1] << ", "
                      << foot_positions[i][2] << "]\n";
        }
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error in forward kinematics: " << e.what() << '\n';
    }
}

void testInverseKinematics() {
    Hexapod hexapod;

    // Example target positions for all legs [x, y, z]
    std::vector<std::vector<float>> target_positions = {
        {-90.0f, -65.0f, -120.0f}, // Leg 0
        {0.0f, -90.0f, -120.0f},   // Leg 1
        {90.0f, -65.0f, -120.0f},  // Leg 2
        {-90.0f, 65.0f, -120.0f},  // Leg 3
        {0.0f, 90.0f, -120.0f},    // Leg 4
        {90.0f, 65.0f, -120.0f}    // Leg 5
    };

    try {
        auto joint_angles = hexapod.computeInverseKinematics(target_positions);
        std::cout << "Inverse Kinematics Results:\n";
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            std::cout << "Leg " << i << ": Angles ["
                      << joint_angles[i][0] << ", "
                      << joint_angles[i][1] << ", "
                      << joint_angles[i][2] << "]\n";
        }
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error in inverse kinematics: " << e.what() << '\n';
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error in inverse kinematics: " << e.what() << '\n';
    }
}

int main() {
    std::cout << "Testing Forward Kinematics:\n";
    testForwardKinematics();

    std::cout << "\nTesting Inverse Kinematics:\n";
    testInverseKinematics();

    return 0;
}
