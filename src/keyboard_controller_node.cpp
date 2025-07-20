#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>

// Interactive keyboard controller for 2-link arm target input
class KeyboardController {
 private:
  ros::NodeHandle nh_;
  ros::Publisher target_pub_;

 public:
  KeyboardController() {
    target_pub_ = nh_.advertise<geometry_msgs::Point>("target_goal", 10);
    ROS_INFO("Keyboard Controller Node initialized");
    ROS_INFO("Publishing to: %s", target_pub_.getTopic().c_str());
  }

  // Main control loop for user input processing
  void run() {
    std::string input;
    double x, y;

    std::cout << "\n=== 2-Link Arm Keyboard Controller ===" << std::endl;
    std::cout << "Enter target coordinates (x y) or 'quit' to exit" << std::endl;
    std::cout << "Valid range: x[-2,2], y[-2,2]" << std::endl;
    std::cout << "Examples: '1.5 0.5', '0 2', '-1 1'" << std::endl;

    while (ros::ok()) {
      std::cout << "\nTarget (x y): ";

      if (!std::getline(std::cin, input)) {
        break;
      }

      if (input == "quit" || input == "exit" || input == "q") {
        break;
      }

      // Parse input coordinates
      std::istringstream iss(input);
      if (iss >> x >> y) {
        geometry_msgs::Point target;
        target.x = x;
        target.y = y;
        target.z = 0.0;  // 2D planar manipulator

        target_pub_.publish(target);

        std::cout << "Sent target: (" << x << ", " << y << ")" << std::endl;
      } else {
        std::cout << "Invalid input. Use format: x y (e.g., 1.5 0.5)" << std::endl;
      }

      ros::spinOnce();
    }

    std::cout << "Keyboard controller shutting down..." << std::endl;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "keyboard_controller_node");

  try {
    KeyboardController controller;
    controller.run();
  } catch (const std::exception& e) {
    ROS_ERROR("Keyboard Controller failed: %s", e.what());
    return 1;
  }

  return 0;
}