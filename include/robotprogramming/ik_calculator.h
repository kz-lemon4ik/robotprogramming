#ifndef ROBOTPROGRAMMING_IK_CALCULATOR_H
#define ROBOTPROGRAMMING_IK_CALCULATOR_H

#include <cmath>
#include <utility>

namespace robotprogramming {

class IKCalculator {
public:
    static constexpr double NUMERICAL_TOLERANCE = 1e-10;
    
    IKCalculator(double link1_length, double link2_length);
    
    ~IKCalculator() = default;
    
    // Calculate joint angles for target position
    bool calculate_joint_angles(double target_x, double target_y, 
                               double& theta1, double& theta2) const;
    
    // Check if target position is reachable
    bool is_reachable(double target_x, double target_y) const;
    
    // Get workspace boundaries
    double get_max_reach() const;
    double get_min_reach() const;
    
    // Get link lengths
    double get_link1_length() const { return L1_; }
    double get_link2_length() const { return L2_; }

private:
    double L1_;  // First link length
    double L2_;  // Second link length
    
    // Calculate distance to target
    double calculate_distance(double x, double y) const;
    
    // Validate input parameters
    bool validate_target(double x, double y) const;
    
    // Handle special cases and singularities
    bool handle_origin_singularity(double target_x, double target_y,
                                  double& theta1, double& theta2) const;
    bool handle_boundary_cases(double target_x, double target_y,
                              double& theta1, double& theta2) const;
};

}  // namespace robotprogramming

#endif  // ROBOTPROGRAMMING_IK_CALCULATOR_H