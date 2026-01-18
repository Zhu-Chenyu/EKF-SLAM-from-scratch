#include <iostream>
#include <string>
#include "turtlelib/geometry2d.hpp"
#include <cmath>

namespace turtlelib
{
std::istream & operator>>(std::istream & is, Point2D & p)
{
        /// \brief Input a 2 dimensional point
        ///
        /// \param is An istream from which to read
        /// \param p [out] The Point2D object that will store the input
        /// \returns A reference to is. An error flag is set on the stream if the input cannot be parsed.
  is >> std::ws;        // skip leading whitespace including newlines
  if (is.peek() == '(') {
    is.get();
    is >> p.x;
    if (is.peek() == ',') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
    is >> p.y;
    if (is.peek() == ')') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
  } else {
    is >> p.x >> p.y;
  }
  return is;
}

Vector2D operator-(const Point2D & head, const Point2D & tail)
{
        /// \brief Subtracting one point from another yields a vector
        /// \param head point corresponding to the head of the vector
        /// \param tail point corresponding to the tail of the vector
        /// \return a vector that points from p1 to p2
        /// NOTE: this operator is not implemented in terms of -=
        /// because subtracting two Point2D yields a Vector2D not a Point2D
  Vector2D result;
  result.x = head.x - tail.x;
  result.y = head.y - tail.y;
  return result;
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
        /// \brief Adding a vector to a point yields a new point displaced by the vector
        /// \param tail The origin of the vector's tail
        /// \param disp The displacement vector
        /// \return A new point that is displaced from tail by disp
  Point2D result;
  result.x = tail.x + disp.x;
  result.y = tail.y + disp.y;
  return result;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
        /// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
        /// \param os - stream to output to
        /// \param v - the vector to print
  os << "[" << v.x << ", " << v.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
        /// \brief input a 2 dimensional vector
        ///
        /// \param is An istream from which to read
        /// \param v [out] - output vector
        /// \returns a reference to the istream, with any error flags set if
        /// a parsing error occurs
  is >> std::ws;        // skip leading whitespace including newlines
  if (is.peek() == '[') {
    is.get();
    is >> v.x;
    if (is.peek() == ',') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
    is >> v.y;
    if (is.peek() == ']') {
      is.get();
    } else {
      is.setstate(std::ios::failbit);
    }
  } else {
    is >> v.x >> v.y;
  }
  return is;
}

Vector2D normalize(Vector2D in)
{
        /// \brief Return a unit vector in the direction of v
        /// \param in The vector to normalize
        /// \return The normalized vector.
        /// \throws std::invalid_input if in is the zero vector
  double mag = std::sqrt(in.x * in.x + in.y * in.y);
  if (mag == 0.0) {
    throw std::invalid_argument("Cannot normalize the zero vector");
  }
  Vector2D result;
  result.x = in.x / mag;
  result.y = in.y / mag;
  return result;
}
}
