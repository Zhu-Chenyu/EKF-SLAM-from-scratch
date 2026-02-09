#include <iostream>
#include <string>
#include "turtlelib/geometry2d.hpp"
#include <cmath>

namespace turtlelib
{
std::istream & operator>>(std::istream & is, Point2D & p)
{
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
  Vector2D result;
  result.x = head.x - tail.x;
  result.y = head.y - tail.y;
  return result;
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  Point2D result;
  result.x = tail.x + disp.x;
  result.y = tail.y + disp.y;
  return result;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  os << "[" << v.x << ", " << v.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
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
  double mag = std::sqrt(in.x * in.x + in.y * in.y);
  if (mag == 0.0) {
    throw std::invalid_argument("Cannot normalize the zero vector");
  }
  Vector2D result;
  result.x = in.x / mag;
  result.y = in.y / mag;
  return result;
}

Vector2D operator+(const Vector2D & v1, const Vector2D & v2)
{
  Vector2D result;
  result.x = v1.x + v2.x;
  result.y = v1.y + v2.y;
  return result;
}

Vector2D & operator+=(Vector2D & v1, const Vector2D & v2)
{
  v1.x += v2.x;
  v1.y += v2.y;
  return v1;
}

Vector2D operator-(const Vector2D & v1, const Vector2D & v2)
{
  Vector2D result;
  result.x = v1.x - v2.x;
  result.y = v1.y - v2.y;
  return result;
}

Vector2D & operator-=(Vector2D & v1, const Vector2D & v2)
{
  v1.x -= v2.x;
  v1.y -= v2.y;
  return v1;
}

Vector2D operator*(double scalar, const Vector2D & v)
{
  Vector2D result;
  result.x = scalar * v.x;
  result.y = scalar * v.y;
  return result;
}

Vector2D & operator*=(Vector2D & v, double scalar)
{
  v.x *= scalar;
  v.y *= scalar;
  return v;
}

double dot(Vector2D v1, Vector2D v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

double magnitude(Vector2D v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

double angle(Vector2D v1, Vector2D v2)
{
  if (magnitude(v1) == 0.0 || magnitude(v2) == 0.0) {
    throw std::invalid_argument("Cannot compute angle with the zero vector");
  }
  double cos_theta = dot(v1, v2) / (magnitude(v1) * magnitude(v2));
  if (abs(cos_theta) > 1.0) {
    throw std::invalid_argument("Cosine of angle out of range due to numerical error");
  }
  return std::acos(cos_theta);
}
}
