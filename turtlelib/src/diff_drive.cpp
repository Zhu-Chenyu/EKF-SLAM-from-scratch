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

  if (std::abs(d_theta) < 1e-10) {
    // Pure translation (straight line)
    x_ += d_center * std::cos(theta_);
    y_ += d_center * std::sin(theta_);
  } else {
    // Arc motion
    double R = d_center / d_theta;
    x_ += R * (std::sin(theta_ + d_theta) - std::sin(theta_));
    y_ += -R * (std::cos(theta_ + d_theta) - std::cos(theta_));
  }
  theta_ = normalize_angle(theta_ + d_theta);

  left_wheel_position_ = new_left_wheel_position;
  right_wheel_position_ = new_right_wheel_position;
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
