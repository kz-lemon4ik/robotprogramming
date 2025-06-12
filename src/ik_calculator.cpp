#include "robotprogramming/ik_calculator.h"
#include <stdexcept>

namespace robotprogramming {

IKCalculator::IKCalculator(double link1_length, double link2_length)
    : L1_(link1_length), L2_(link2_length) {
    if (L1_ <= NUMERICAL_TOLERANCE || L2_ <= NUMERICAL_TOLERANCE) {
        throw std::invalid_argument("Link lengths must be positive");
    }
}

bool IKCalculator::calculate_joint_angles(double target_x, double target_y, 
                                         double& theta1, double& theta2) const {
    if (!validate_target(target_x, target_y)) {
        return false;
    }
    
    if (!is_reachable(target_x, target_y)) {
        return false;
    }
    
    const double d_squared = target_x * target_x + target_y * target_y;
    
    // Law of cosines: cos(theta2) = (d² - L1² - L2²) / (2*L1*L2)
    double cos_theta2 = (d_squared - L1_ * L1_ - L2_ * L2_) / (2.0 * L1_ * L2_);
    
    // Clamp to [-1, 1] to handle numerical precision errors
    cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));
    
    // Elbow up configuration (positive theta2)
    theta2 = std::acos(cos_theta2);
    
    // theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
    const double alpha = std::atan2(target_y, target_x);
    const double beta = std::atan2(L2_ * std::sin(theta2), L1_ + L2_ * std::cos(theta2));
    theta1 = alpha - beta;
    
    return true;
}

bool IKCalculator::is_reachable(double target_x, double target_y) const {
    const double distance = calculate_distance(target_x, target_y);
    return (distance >= get_min_reach() && distance <= get_max_reach());
}

double IKCalculator::get_max_reach() const {
    // Maximum reach when both links are fully extended
    return L1_ + L2_;
}

double IKCalculator::get_min_reach() const {
    // Minimum reach when links are aligned in opposite directions
    return std::abs(L1_ - L2_);
}

double IKCalculator::calculate_distance(double x, double y) const {
    return std::sqrt(x * x + y * y);
}

bool IKCalculator::validate_target(double x, double y) const {
    return std::isfinite(x) && std::isfinite(y);
}

}  // namespace robotprogramming