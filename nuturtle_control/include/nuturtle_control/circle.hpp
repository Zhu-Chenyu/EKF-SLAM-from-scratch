#ifndef NUTURTLE_CONTROL_CIRCLE_HPP
#define NUTURTLE_CONTROL_CIRCLE_HPP
/// \file
/// \brief A ROS2 node that drives the robot in a circular trajectory.
///
/// PUBLISHES:
///     cmd_vel (geometry_msgs/msg/Twist): the velocity commands for the robot
/// SERVERS:
///     control (nuturtle_control/srv/Control): set the radius and velocity of the circular trajectory
///     reverse (std_srvs/srv/Empty): reverse the direction of the circular trajectory
///     stop (std_srvs/srv/Empty): stop the circular trajectory

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include <chrono>
#include <functional>


class Circle : public rclcpp::Node {
public:
    /// \brief Construct a Circle node that publishes velocity commands to follow a circular trajectory and provides services to control the trajectory.
  Circle();

private:
  int freq = 100;
  double radius_ = 0.0;
  double velocity_ = 0.0;   // angular velocity
    /// \brief Publish velocity commands to follow a circular trajectory at a specified frequency.
  void timer_callback();
    /// \brief Service callback to update the radius and velocity of the circular trajectory.
    /// \param request - the incoming service request containing the desired radius and velocity
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>);
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
};
#endif
