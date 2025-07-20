#include <iomanip>
#include <iostream>
#include <vector>
#include "robotprogramming/ik_calculator.h"

// Test case structure for comprehensive IK validation
struct TestCase {
  double x;
  double y;
  std::string description;
};

// Execute single test case and display results with validation
void print_test_result(const robotprogramming::IKCalculator& calculator, const TestCase& test) {
  std::cout << "\nTest: " << test.description << std::endl;
  std::cout << "Target: (" << test.x << ", " << test.y << ")" << std::endl;

  if (!calculator.is_reachable(test.x, test.y)) {
    std::cout << "Result: UNREACHABLE" << std::endl;
    return;
  }

  double theta1, theta2;
  bool success = calculator.calculate_joint_angles(test.x, test.y, theta1, theta2);

  if (success) {
    std::cout << "Result: SUCCESS" << std::endl;
    std::cout << "theta1: " << std::fixed << std::setprecision(4) << theta1 << " rad ("
              << theta1 * 180.0 / M_PI << " deg)" << std::endl;
    std::cout << "theta2: " << std::fixed << std::setprecision(4) << theta2 << " rad ("
              << theta2 * 180.0 / M_PI << " deg)" << std::endl;

    // Forward kinematics verification
    double L1 = calculator.get_link1_length();
    double L2 = calculator.get_link2_length();
    double calc_x = L1 * std::cos(theta1) + L2 * std::cos(theta1 + theta2);
    double calc_y = L1 * std::sin(theta1) + L2 * std::sin(theta1 + theta2);

    std::cout << "Forward check: (" << std::fixed << std::setprecision(4) << calc_x << ", "
              << calc_y << ")" << std::endl;

    double error =
        std::sqrt((calc_x - test.x) * (calc_x - test.x) + (calc_y - test.y) * (calc_y - test.y));
    std::cout << "Position error: " << std::scientific << std::setprecision(2) << error
              << std::endl;
  } else {
    std::cout << "Result: FAILED" << std::endl;
  }
}

int main() {
  const double L1 = 1.0;  // First link length
  const double L2 = 1.0;  // Second link length

  robotprogramming::IKCalculator calculator(L1, L2);

  std::cout << "=== IKCalculator Test Program ===" << std::endl;
  std::cout << "Link lengths: L1 = " << L1 << ", L2 = " << L2 << std::endl;
  std::cout << "Workspace: min_reach = " << calculator.get_min_reach()
            << ", max_reach = " << calculator.get_max_reach() << std::endl;

  std::vector<TestCase> test_cases = {{1.5, 0.0, "Basic horizontal position"},
                                      {0.0, 1.5, "Basic vertical position"},
                                      {1.0, 1.0, "45 degree diagonal"},
                                      {2.0, 0.0, "Maximum reach horizontal"},
                                      {0.0, 2.0, "Maximum reach vertical"},
                                      {1.414, 1.414, "Maximum reach diagonal"},
                                      {0.5, 0.5, "Close position"},
                                      {-1.0, 0.5, "Negative x position"},
                                      {0.5, -1.0, "Negative y position"},
                                      {-1.0, -1.0, "Third quadrant"},
                                      {3.0, 0.0, "Beyond maximum reach"},
                                      {0.0, 0.0, "Origin position"},
                                      {0.1, 0.0, "Very close position"}};

  for (const auto& test : test_cases) {
    print_test_result(calculator, test);
  }

  // Interactive mode
  std::cout << "\n=== Interactive Mode ===" << std::endl;
  std::cout << "Enter target coordinates (x y) or 'q' to quit:" << std::endl;

  double x, y;
  while (std::cout << "> " && std::cin >> x >> y) {
    TestCase interactive_test = {x, y, "Interactive input"};
    print_test_result(calculator, interactive_test);
  }

  return 0;
}