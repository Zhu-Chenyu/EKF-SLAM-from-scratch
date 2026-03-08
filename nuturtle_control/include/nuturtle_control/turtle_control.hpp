#ifndef NUTURTLE_CONTROL_TURTLE_CONTROL_HPP
#define NUTURTLE_CONTROL_TURTLE_CONTROL_HPP
/// \file
/// \brief A ROS2 node that controls each wheel and reads the encoders of wheels.

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

class TurtleControl : public rclcpp::Node {
public:
    /// \brief Construct a TurtleControl node
  TurtleControl();

private:
    /// \brief Callback function for twist messages. Converts the desired twist into wheel commands and publishes them.
    /// \param msg - the incoming twist message containing the desired linear and angular velocities
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    /// \brief Callback function for sensor data messages. Updates the robot's pose based on encoder readings and publishes joint states.
    /// \param msg - the incoming sensor data message containing encoder readings for the left and right
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg);
  turtlelib::DiffDrive dd_;
  rclcpp::Time prev_time_ = this->now();
  double prev_left_wheel_pos_ = 0.0;
  double prev_right_wheel_pos_ = 0.0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mv_subscription_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_subscription_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  double wheel_radius_;
  double track_width_;
  int motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
};
#endif
