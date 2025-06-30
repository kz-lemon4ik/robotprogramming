#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "robotprogramming/ik_calculator.h"

class IKSolverNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher joint_pub_;
    ros::Publisher reachable_pub_;
    ros::Publisher status_pub_;
    
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
        reachable_pub_ = nh_.advertise<std_msgs::Bool>("target_reachable", 10);
        status_pub_ = nh_.advertise<std_msgs::String>("ik_status", 10);
        
        ROS_INFO("IK Solver Node initialized");
        ROS_INFO("Link lengths: L1=%.2f, L2=%.2f", link1_length_, link2_length_);
        ROS_INFO("Workspace: min_reach=%.2f, max_reach=%.2f", 
                 ik_calculator_.get_min_reach(), ik_calculator_.get_max_reach());
        ROS_INFO("Subscribed to: %s", target_sub_.getTopic().c_str());
        ROS_INFO("Publishing to: %s", joint_pub_.getTopic().c_str());
    }
    
    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        double theta1, theta2;
        
        ROS_INFO("Received target: (%.3f, %.3f)", msg->x, msg->y);
        
        // Check reachability first and publish result
        bool reachable = ik_calculator_.is_reachable(msg->x, msg->y);
        publishReachability(reachable);
        
        if (!reachable) {
            ROS_WARN("Target (%.3f, %.3f) is not reachable", msg->x, msg->y);
            publishStatus("UNREACHABLE");
            return;
        }
        
        // Calculate inverse kinematics
        bool success = ik_calculator_.calculate_joint_angles(msg->x, msg->y, theta1, theta2);
        
        if (success) {
            ROS_INFO("IK solution: theta1=%.3f rad (%.1f deg), theta2=%.3f rad (%.1f deg)", 
                     theta1, theta1 * 180.0 / M_PI, theta2, theta2 * 180.0 / M_PI);
            publishJointStates(theta1, theta2);
            publishStatus("SUCCESS");
        } else {
            ROS_ERROR("IK calculation failed for target (%.3f, %.3f)", msg->x, msg->y);
            publishStatus("CALCULATION_FAILED");
        }
    }
    
    void publishJointStates(double theta1, double theta2) {
        sensor_msgs::JointState joint_state;
        
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = joint_names_;
        joint_state.position.push_back(theta1);
        joint_state.position.push_back(theta2);
        
        // Add zero velocities and efforts for complete joint state
        joint_state.velocity.push_back(0.0);
        joint_state.velocity.push_back(0.0);
        joint_state.effort.push_back(0.0);
        joint_state.effort.push_back(0.0);
        
        joint_pub_.publish(joint_state);
    }
    
    void publishReachability(bool reachable) {
        std_msgs::Bool msg;
        msg.data = reachable;
        reachable_pub_.publish(msg);
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
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