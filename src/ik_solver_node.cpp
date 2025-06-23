#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include "robotprogramming/ik_calculator.h"

class IKSolverNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher joint_pub_;
    
    robotprogramming::IKCalculator ik_calculator_;
    
    // Robot parameters
    double link1_length_;
    double link2_length_;
    
    // Joint names for publishing
    std::vector<std::string> joint_names_;

public:
    IKSolverNode() : ik_calculator_(1.0, 1.0) {
        // Get parameters from parameter server
        nh_.param("link1_length", link1_length_, 1.0);
        nh_.param("link2_length", link2_length_, 1.0);
        
        // Reinitialize calculator with parameters
        ik_calculator_ = robotprogramming::IKCalculator(link1_length_, link2_length_);
        
        // Set up joint names
        joint_names_.push_back("joint1");
        joint_names_.push_back("joint2");
        
        // Initialize subscribers and publishers
        target_sub_ = nh_.subscribe("target_goal", 10, 
                                   &IKSolverNode::targetCallback, this);
        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
        
        ROS_INFO("IK Solver Node initialized");
        ROS_INFO("Link lengths: L1=%.2f, L2=%.2f", link1_length_, link2_length_);
        ROS_INFO("Workspace: min_reach=%.2f, max_reach=%.2f", 
                 ik_calculator_.get_min_reach(), ik_calculator_.get_max_reach());
    }
    
    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        double theta1, theta2;
        
        ROS_INFO("Received target: (%.3f, %.3f)", msg->x, msg->y);
        
        // Calculate inverse kinematics
        bool success = ik_calculator_.calculate_joint_angles(msg->x, msg->y, theta1, theta2);
        
        if (success) {
            ROS_INFO("IK solution: theta1=%.3f, theta2=%.3f", theta1, theta2);
            publishJointStates(theta1, theta2);
        } else {
            if (!ik_calculator_.is_reachable(msg->x, msg->y)) {
                ROS_WARN("Target (%.3f, %.3f) is not reachable", msg->x, msg->y);
            } else {
                ROS_ERROR("IK calculation failed for target (%.3f, %.3f)", msg->x, msg->y);
            }
        }
    }
    
    void publishJointStates(double theta1, double theta2) {
        sensor_msgs::JointState joint_state;
        
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = joint_names_;
        joint_state.position.push_back(theta1);
        joint_state.position.push_back(theta2);
        
        joint_pub_.publish(joint_state);
    }
    
    void spin() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ik_solver_node");
    
    try {
        IKSolverNode node;
        node.spin();
    } catch (const std::exception& e) {
        ROS_ERROR("IK Solver Node failed: %s", e.what());
        return 1;
    }
    
    return 0;
}