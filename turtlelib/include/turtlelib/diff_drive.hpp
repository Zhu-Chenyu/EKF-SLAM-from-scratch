#ifndef TURTELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
class DiffDrive
{
private:
  double wheel_track_;
  double wheel_radius_;
  double left_wheel_position_;
  double right_wheel_position_;
  double x_;
  double y_;
  double theta_;

public:
    /// \brief Get the wheel track (distance between wheels) and wheel radius
    /// \return the wheel track and wheel radius
  double get_wheel_track() const {return wheel_track_;}
  double get_wheel_radius() const {return wheel_radius_;}

    /// \brief Get the robot's current pose and wheel positions
    /// \return x, y, theta, left wheel position, right wheel position
  double get_x() const {return x_;}
  double get_y() const {return y_;}
  double get_theta() const {return theta_;}
  double get_left_wheel_position() const {return left_wheel_position_;}
  double get_right_wheel_position() const {return right_wheel_position_;}

    /// \brief Set the robot's current pose
  void set_x(double x) {x_ = x;}
  void set_y(double y) {y_ = y;}
  void set_theta(double theta) {theta_ = theta;}

    /// \brief Construct a differential drive kinematic model
    /// \param wheel_track - the distance between the wheels
    /// \param wheel_radius - the radius of the wheels
  DiffDrive(double wheel_track, double wheel_radius);

    /// \brief Default constructor
  DiffDrive() : DiffDrive(0.0, 0.0) {}

    /// \brief Update the robot's pose based on wheel movements
    /// \param new_left_wheel_position - the new position of the left wheel
    /// \param new_right_wheel_position - the new position of the right wheel
  void forward_kinematics(double new_left_wheel_position, double new_right_wheel_position);

    /// \brief Compute the required wheel velocities to achieve a given twist
    /// \param twist - the desired twist (angular and linear velocities)
  std::pair<double, double> inverse_kinematics(const turtlelib::Twist2D & twist) const;
};
}
#endif
