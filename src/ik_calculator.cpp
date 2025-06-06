#include "robotprogramming/ik_calculator.h"

namespace robotprogramming {

IKCalculator::IKCalculator(double link1_length, double link2_length)
    : L1_(link1_length), L2_(link2_length) {
    // Constructor implementation will be added in next phase
}

bool IKCalculator::calculate_joint_angles(double target_x, double target_y, 
                                         double& theta1, double& theta2) const {
    // Implementation placeholder - will be completed in geometric solution phase
    return false;
}

bool IKCalculator::is_reachable(double target_x, double target_y) const {
    double distance = calculate_distance(target_x, target_y);
    return (distance >= get_min_reach() && distance <= get_max_reach());
}

double IKCalculator::get_max_reach() const {
    return L1_ + L2_;
}

double IKCalculator::get_min_reach() const {
    return std::abs(L1_ - L2_);
}

double IKCalculator::calculate_distance(double x, double y) const {
    return std::sqrt(x * x + y * y);
}

bool IKCalculator::validate_target(double x, double y) const {
    return std::isfinite(x) && std::isfinite(y);
}

}  // namespace robotprogramming