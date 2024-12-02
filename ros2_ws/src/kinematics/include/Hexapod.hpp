#ifndef HEXAPOD_HPP
#define HEXAPOD_HPP

#include "SpiderLeg.hpp"
#include <vector>
#include <string>

class Hexapod {
public:
    /**
     * @brief Construct a new Hexapod object
     * Initializes all six legs with their respective configurations.
     */
    Hexapod();

    /**
     * @brief Compute the forward kinematics for all legs
     * 
     * @param joint_angles A vector of 6 vectors, each containing the three joint angles (in degrees) for one leg
     * @return A vector of 6 vectors, each containing the foot position [x, y, z] in the global frame
     * @throws std::invalid_argument if joint_angles does not contain exactly 6 vectors
     */
    std::vector<std::vector<float>> computeForwardKinematics(const std::vector<std::vector<float>>& joint_angles) const;

    /**
     * @brief Compute the inverse kinematics for all legs
     * 
     * @param target_positions A vector of 6 vectors, each containing the target foot position [x, y, z] in the global frame
     * @return A vector of 6 vectors, each containing the three joint angles (in degrees) for one leg
     * @throws std::invalid_argument if target_positions does not contain exactly 6 vectors
     */
    std::vector<std::vector<float>> computeInverseKinematics(const std::vector<std::vector<float>>& target_positions) const;

    /**
     * @brief Get a reference to the legs for debugging or advanced control
     * 
     * @return A const reference to the vector of SpiderLeg objects
     */
    const std::vector<SpiderLeg>& getLegs() const;

private:
    std::vector<SpiderLeg> legs_; ///< Vector of six SpiderLeg objects representing the robot's legs
};

#endif // HEXAPOD_HPP
