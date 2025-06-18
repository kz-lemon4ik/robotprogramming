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
    
    // Handle origin singularity
    if (handle_origin_singularity(target_x, target_y, theta1, theta2)) {
        return true;
    }
    
    if (!is_reachable(target_x, target_y)) {
        return false;
    }
    
    // Handle boundary cases (fully extended/retracted)
    if (handle_boundary_cases(target_x, target_y, theta1, theta2)) {
        return true;
    }
    
    const double d_squared = target_x * target_x + target_y * target_y;
    const double d = std::sqrt(d_squared);
    
    // Law of cosines: cos(theta2) = (d² - L1² - L2²) / (2*L1*L2)
    double cos_theta2 = (d_squared - L1_ * L1_ - L2_ * L2_) / (2.0 * L1_ * L2_);
    
    // Clamp to [-1, 1] to handle numerical precision errors
    cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));
    
    // Elbow up configuration (positive theta2)
    theta2 = std::acos(cos_theta2);
    
    // Avoid division by zero in atan2 calculation
    const double denominator = L1_ + L2_ * std::cos(theta2);
    if (std::abs(denominator) < NUMERICAL_TOLERANCE) {
        // Special handling for near-zero denominator
        theta1 = std::atan2(target_y, target_x);
        if (target_x < 0) {
            theta1 += M_PI;
        }
    } else {
        // theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
        const double alpha = std::atan2(target_y, target_x);
        const double beta = std::atan2(L2_ * std::sin(theta2), denominator);
        theta1 = alpha - beta;
    }
    
    return true;
}

bool IKCalculator::is_reachable(double target_x, double target_y) const {
    const double distance = calculate_distance(target_x, target_y);
    const double min_reach = get_min_reach();
    const double max_reach = get_max_reach();
    
    // Add numerical tolerance to boundaries
    return (distance >= (min_reach - NUMERICAL_TOLERANCE) && 
            distance <= (max_reach + NUMERICAL_TOLERANCE));
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

bool IKCalculator::handle_origin_singularity(double target_x, double target_y,
                                            double& theta1, double& theta2) const {
    const double distance = calculate_distance(target_x, target_y);
    
    // Check if target is at or very close to origin
    if (distance < NUMERICAL_TOLERANCE) {
        // For origin, any theta1 is valid if L1 == L2
        if (std::abs(L1_ - L2_) < NUMERICAL_TOLERANCE) {
            theta1 = 0.0;  // Default orientation
            theta2 = M_PI; // Links folded back
            return true;
        }
        return false; // Origin not reachable if L1 != L2
    }
    
    return false; // Not an origin case
}

bool IKCalculator::handle_boundary_cases(double target_x, double target_y,
                                        double& theta1, double& theta2) const {
    const double distance = calculate_distance(target_x, target_y);
    const double max_reach = get_max_reach();
    const double min_reach = get_min_reach();
    
    // Fully extended configuration
    if (std::abs(distance - max_reach) < NUMERICAL_TOLERANCE) {
        theta1 = std::atan2(target_y, target_x);
        theta2 = 0.0; // Links aligned
        return true;
    }
    
    // Fully retracted configuration (only possible if L1 != L2)
    if (std::abs(distance - min_reach) < NUMERICAL_TOLERANCE && 
        std::abs(L1_ - L2_) > NUMERICAL_TOLERANCE) {
        theta1 = std::atan2(target_y, target_x);
        theta2 = M_PI; // Links in opposite directions
        
        // Adjust theta1 for the case where L2 > L1
        if (L2_ > L1_) {
            theta1 += M_PI;
        }
        
        return true;
    }
    
    return false; // Not a boundary case
}

}  // namespace robotprogramming