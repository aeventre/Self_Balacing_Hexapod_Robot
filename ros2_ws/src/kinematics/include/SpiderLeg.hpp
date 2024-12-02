#ifndef SPIDERLEG_HPP
#define SPIDERLEG_HPP

#include <string>
#include <vector>

class SpiderLeg {
public:
    /**
     * @brief Construct a new SpiderLeg object
     * 
     * @param name Name of the leg
     * @param coxa Length of the coxa segment
     * @param femur Length of the femur segment
     * @param tibia Length of the tibia segment
     * @param origin_x X-coordinate of the leg's origin relative to the robot base
     * @param origin_y Y-coordinate of the leg's origin relative to the robot base
     * @param origin_z Z-coordinate of the leg's origin relative to the robot base
     * @param rotation_z Rotation about the Z-axis of the leg's local frame relative to the global frame (in radians)
     * @param is_mirrored Whether the leg is mirrored (e.g., on the opposite side of the robot)
     */
    SpiderLeg(const std::string& name, float coxa, float femur, float tibia,
              float origin_x, float origin_y, float origin_z, float rotation_z, bool is_mirrored);

    /**
     * @brief Compute the foot position from joint angles using forward kinematics
     * 
     * @param angles Vector containing the three joint angles (in degrees): [theta1, theta2, theta3]
     * @return Vector containing the 3D foot position [x, y, z] in the global frame
     */
    std::vector<float> forwardKinematics(const std::vector<float>& angles) const;

    /**
     * @brief Compute the joint angles required to reach a target foot position using inverse kinematics
     * 
     * @param target Vector containing the target 3D foot position [x, y, z] in the global frame
     * @return Vector containing the three joint angles (in degrees): [theta1, theta2, theta3]
     * @throws std::invalid_argument if the target vector size is not 3
     */
    std::vector<float> inverseKinematics(const std::vector<float>& target) const;

    /**
     * @brief Get the name of the leg
     * 
     * @return Name of the leg
     */
    std::string getName() const;

private:
    // Utility functions
    float rad2deg(float radians) const;
    float deg2rad(float degrees) const;

    // Leg properties
    const std::string name_;       ///< Name of the leg
    const float COXA;              ///< Length of the coxa segment
    const float FEMUR;             ///< Length of the femur segment
    const float TIBIA;             ///< Length of the tibia segment

    // Leg's position and orientation in the global frame
    const float origin_x_;         ///< X-coordinate of the leg's origin
    const float origin_y_;         ///< Y-coordinate of the leg's origin
    const float origin_z_;         ///< Z-coordinate of the leg's origin
    const float rotation_z_;       ///< Rotation about the Z-axis in radians
    const bool is_mirrored_;       ///< Whether the leg is mirrored
};

#endif // SPIDERLEG_HPP
