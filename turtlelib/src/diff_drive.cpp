#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <utility>
#include <stdexcept>

namespace turtlelib
{
DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
  : wheel_track_(wheel_track),
    wheel_radius_(wheel_radius),
    left_wheel_position_(0.0),
    right_wheel_position_(0.0),
    x_(0.0),
    y_(0.0),
    theta_(0.0)
{}

void DiffDrive::forward_kinematics(double new_left_wheel_position, double new_right_wheel_position)
{
  double delta_left = new_left_wheel_position - left_wheel_position_;
  double delta_right = new_right_wheel_position - right_wheel_position_;

  double d_left = delta_left * wheel_radius_;
  double d_right = delta_right * wheel_radius_;

  double d_center = (d_left + d_right) / 2.0;
  double d_theta = (d_right - d_left) / wheel_track_;

  double R = (d_theta != 0.0) ? (d_center / d_theta) : 0.0;

  this->x_ += R * (std::sin(this->theta_ + d_theta) - std::sin(this->theta_));
  this->y_ += -R * (std::cos(this->theta_ + d_theta) - std::cos(this->theta_));
  this->theta_ = normalize_angle(this->theta_ + d_theta);

  this->left_wheel_position_ = new_left_wheel_position;
  this->right_wheel_position_ = new_right_wheel_position;
}

std::pair<double, double> DiffDrive::inverse_kinematics(const turtlelib::Twist2D & twist) const
{
  if (std::abs(twist.y) > 1e-6) {
    throw std::logic_error("DiffDrive inverse kinematics does not support lateral velocity.");
  }
  double v = twist.x;        // linear velocity
  double omega = twist.omega; // angular velocity

  double v_left = v - (wheel_track_ / 2.0) * omega;
  double v_right = v + (wheel_track_ / 2.0) * omega;

  double left_wheel_velocity = v_left / wheel_radius_;
  double right_wheel_velocity = v_right / wheel_radius_;

  return {left_wheel_velocity, right_wheel_velocity};
}
}