#ifndef NUTURTLE_CONTROL_ODOMETRY_HPP
#define NUTURTLE_CONTROL_ODOMETRY_HPP
/// \file
/// \brief A ROS2 node that subscribes to joint states and publishes odometry information for the nuturtlebot.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "turtlelib/diff_drive.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include <string>
#include <cmath>

class Odometry : public rclcpp::Node {
public:
    /// \brief Construct an Odometry node
    Odometry();
private:
    /// \brief Callback function for joint state updates. Updates the robot's pose and publishes odometry information.
    /// \param msg - the incoming joint state message
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    /// \brief Callback function for the initial pose service. Resets the robot's pose to the requested initial pose.
    /// \param request - the incoming service request containing the desired initial pose
    /// \param response - the service response (empty in this case)
    void init_pose_callback(
        const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
        std::shared_ptr<nuturtle_control::srv::InitialPose::Response>);
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr odom_service_;
    turtlelib::DiffDrive dd;
    std::string body_id_;
    std::string odom_id_;
    std::string wheel_left_ = "left_wheel_joint";
    std::string wheel_right_ = "wheel_right_joint";
    double wheel_radius_ = 0.033;
    double track_width_ = 0.16;
};
#endif