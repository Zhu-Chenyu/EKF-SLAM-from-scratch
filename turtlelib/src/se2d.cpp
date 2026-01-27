#include <iostream>
#include "turtlelib/se2d.hpp"
#include <cmath>
#include <string>

namespace turtlelib
{
std::istream & operator>>(std::istream & is, Twist2D & tw)
{
  if (is.peek() == '<') {
    is.get();
    is >> tw.omega;
    while (is.peek() == ' ') {
      is.get();
    }
    if (is.peek() == '[') {       // unit is specified
      is.get();           // remove the '['
      std::string unit;
      getline(is, unit, ']');
      if (unit[0] == 'd') {
        tw.omega = turtlelib::deg2rad(tw.omega);
      } else if (unit[0] != 'r') {
        is.setstate(std::ios::failbit);
      }
    }
    if (is.peek() == ',') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
    is >> tw.x;
    if (is.peek() == ',') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
    is >> tw.y;
    if (is.peek() == '>') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
  } else {
    is >> tw.omega;
    while (is.peek() == ' ') {
      is.get();
    }
    if (is.peek() == '[') {       // unit is specified
      is.get();           // remove the '['
      std::string unit;
      getline(is, unit, ']');
      if (unit[0] == 'd') {
        tw.omega = turtlelib::deg2rad(tw.omega);
      } else if (unit[0] != 'r') {
        is.setstate(std::ios::failbit);
      }
    }
    is >> tw.x >> tw.y;
  }
  return is;
}

Transform2D::Transform2D()
{
  translation_ = {0.0, 0.0};
  rotation_ = {1.0, 0.0};
}

Transform2D::Transform2D(Vector2D trans)
{
  translation_ = trans;
  rotation_ = {1.0, 0.0};
}

Transform2D::Transform2D(double radians)
{
  translation_ = {0.0, 0.0};
  rotation_ = {std::cos(radians), std::sin(radians)};
}

Transform2D::Transform2D(Vector2D trans, double radians)
{
  translation_ = trans;
  rotation_ = {std::cos(radians), std::sin(radians)};
}

Point2D Transform2D::operator()(Point2D p) const
{
  Point2D result;
  result.x = rotation_.x * p.x - rotation_.y * p.y + translation_.x;
  result.y = rotation_.y * p.x + rotation_.x * p.y + translation_.y;
  return result;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  Vector2D result;
  result.x = rotation_.x * v.x - rotation_.y * v.y;
  result.y = rotation_.y * v.x + rotation_.x * v.y;
  return result;
}

Twist2D Transform2D::operator()(Twist2D v) const
{
  Twist2D result;
  result.omega = v.omega;       // angular velocity remains unchanged
  result.x = v.x * rotation_.x - v.y * rotation_.y + translation_.y * v.omega;
  result.y = v.x * rotation_.y + v.y * rotation_.x - translation_.x * v.omega;
  return result;
}

Transform2D Transform2D::inv() const
{
  Transform2D result;
  result.rotation_.x = rotation_.x;
  result.rotation_.y = -rotation_.y;
  result.translation_.x = -( rotation_.x * translation_.x + rotation_.y * translation_.y );
  result.translation_.y = -(-rotation_.y * translation_.x + rotation_.x * translation_.y );
  return result;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  auto old_rot_x = rotation_.x;
  auto old_rot_y = rotation_.y;
  translation_.x = rotation_.x * rhs.translation_.x - rotation_.y * rhs.translation_.y +
    translation_.x;
  translation_.y = rotation_.y * rhs.translation_.x + rotation_.x * rhs.translation_.y +
    translation_.y;
  rotation_.x = old_rot_x * rhs.rotation_.x - old_rot_y * rhs.rotation_.y;
  rotation_.y = old_rot_y * rhs.rotation_.x + old_rot_x * rhs.rotation_.y;
  return *this;
}

Vector2D Transform2D::translation() const
{
  return translation_;
}

double Transform2D::rotation() const
{
  return std::atan2(rotation_.y, rotation_.x);
}

Twist2D operator*(double scalar, const Twist2D & tw)
{
  Twist2D result;
  result.omega = scalar * tw.omega;
  result.x = scalar * tw.x;
  result.y = scalar * tw.y;
  return result;
}

Twist2D & operator*=(Twist2D & tw, double scalar)
{
  tw.omega *= scalar;
  tw.x *= scalar;
  tw.y *= scalar;
  return tw;
}

Transform2D integrate_twist(Twist2D tw)
{
  Transform2D result;
  if (std::abs(tw.omega) < 1e-10) {       // pure translation
    result = Transform2D(Vector2D{tw.x, tw.y});
  } else {
    //////////////Citation [2]//////////////
    double r_x = tw.x / tw.omega;
    double r_y = tw.y / tw.omega;
    double theta = tw.omega;
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double trans_x = r_x * sin_theta - r_y * (1 - cos_theta);
    double trans_y = r_x * (1 - cos_theta) + r_y * sin_theta;
    result = Transform2D(Vector2D{trans_x, trans_y}, theta);
    ////////////////////////////////////////
  }
  return result;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  is >> std::ws;        // skip leading whitespace including newlines
  if (is.peek() == '{') {
    is.get();
    double angle;
    is >> angle;
    while (is.peek() == ' ') {
      is.get();
    }
    if (is.peek() == '[') {       // unit is specified
      is.get();           // remove the '['
      std::string unit;
      getline(is, unit, ']');
      if (unit[0] == 'd') {
        angle = turtlelib::deg2rad(angle);
      } else if (unit[0] != 'r') {
        is.setstate(std::ios::failbit);
      }
    }
    if (is.peek() == ',') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
    double x, y;
    is >> x >> y;
    if (is.peek() == '}') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
    tf = Transform2D(Vector2D{x, y}, angle);
  } else {
    double angle;
    is >> angle;
    while (is.peek() == ' ') {
      is.get();
    }
    if (is.peek() == '[') {       // unit is specified
      is.get();           // remove the '['
      std::string unit;
      getline(is, unit, ']');
      if (unit[0] == 'd') {
        angle = turtlelib::deg2rad(angle);
      } else if (unit[0] != 'r') {
        is.setstate(std::ios::failbit);
      }
    }
    double x, y;
    is >> x >> y;
    tf = Transform2D(Vector2D{x, y}, angle);
  }
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  lhs *= rhs;
  return lhs;
}
}
