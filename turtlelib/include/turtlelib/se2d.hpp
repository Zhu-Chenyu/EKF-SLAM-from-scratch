#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include <iosfwd>
#include <format>
#include "turtlelib/angle.hpp"
#include "turtlelib/geometry2d.hpp"


namespace turtlelib
{

    /// \brief represent a 2-Dimensional twist
struct Twist2D
{
        /// \brief the angular velocity
  double omega = 0.0;

        /// \brief the linear x velocity
  double x = 0.0;

        /// \brief the linear y velocity
  double y = 0.0;
};


    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
    /// The "" are not part of the input.
    /// The [<unit>] is optional and can be any string without spaces that starts with an r
    /// (for rad/s) and any string without spaces that starts with a d for deg/s)
    /// If the unit is omitted, assume rad/s.
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
std::istream & operator>>(std::istream & is, Twist2D & tw);


    /// \brief a rigid body transformation in 2 dimensions
class Transform2D
{
private:
  Vector2D translation_;        // the x,y translation
  Vector2D rotation_;           // unit vector representing rotation (cos(theta), sin(theta))

public:
        /// \brief Create an identity transformation
  Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
  explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
  explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
  Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a 2D Point
        /// \param p the point to transform
        /// \return a point in the new coordinate system
  Point2D operator()(Point2D p) const;

        /// \brief apply a transformation to a 2D Vector
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
  Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
  Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
  Transform2D inv() const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
  Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
  Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
  double rotation() const;

        /// \brief see std::formatter for the Transform2D
  template<class CharT>
  friend struct std::formatter;

};


    /// \brief Read a transformation from stdin
    /// Should be able to read input either as:
    ///  "theta [<unit>] dx dy" (i.e., three numbers separated by whitespace, angle assumed to be radians)
    //   "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  [<unit>] is optional and can be any string without spaces that starts with a d for deg or r for rad
    ///  If [<unit>] is omitted, assume the unit is radians
std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief multiply a Twist2D by a scalar
    /// \param tw - the twist to scale
    /// \param scalar - the scalar value
    /// \return the scaled twist
Twist2D operator*(double scalar, const Twist2D & tw);

    /// \brief multiply a Twist2D by a scalar in place
    /// \param tw - the twist to scale
    /// \param scalar - the scalar value
    /// \return the scaled twist
Twist2D & operator*=(Twist2D & tw, double scalar);

    /// \brief integrate a twist to produce a Transform2D
    /// \param tw - the twist to integrate
    /// \return the resulting Transform2D
Transform2D integrate_twist(Twist2D tw);
}

/// \brief A formatter for Transform2D
/// Creates a string representation of a Transform2D
/// as "{<angle> [<unit>], <x> <y>}"
/// An R at the beginning of the format-spec makes [<unit>] rad
/// A D  at the beginning of the format-spec makes [<unit>] deg
/// No R or D means no unit is printed but the angle is in radians.
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers that
/// are put into the string
namespace std
{
template<class CharT>
class formatter<turtlelib::Transform2D, CharT>
{
  mutable formatter<double, CharT> double_fmt;
  bool use_degree = false;
  bool print_unit = false;        // whether to print the unit or not

public:
  constexpr auto parse(basic_format_parse_context<CharT> & ctx)
  {
    auto it = ctx.begin();
    const auto end = ctx.end();
    if (it != end) {
      if (*it == 'R') {
        use_degree = false;
        print_unit = true;
        ++it;
      } else if (*it == 'D') {
        use_degree = true;
        print_unit = true;
        ++it;
      }
    }
    ctx.advance_to(it);
    return double_fmt.parse(ctx);
  }

  template<class FormatContext>
  auto format(const turtlelib::Transform2D & tf, FormatContext & ctx) const
  {
    auto out = ctx.out();
    *out++ = '{';
    auto theta = tf.rotation();
    if (use_degree) {
      theta = turtlelib::rad2deg(theta);
    }
    out = double_fmt.format(theta, ctx);
    if (print_unit) {
      *out++ = ' ';
      *out++ = '[';
      if (use_degree) {
        *out++ = 'd';
        *out++ = 'e';
        *out++ = 'g';
      } else {
        *out++ = 'r';
        *out++ = 'a';
        *out++ = 'd';
      }
      *out++ = ']';
      *out++ = '/';
      *out++ = 's';
    }
    *out++ = ',';
    *out++ = ' ';
    out = double_fmt.format(tf.translation().x, ctx);
    *out++ = ' ';
    out = double_fmt.format(tf.translation().y, ctx);
    *out++ = '}';
    return out;
  }
};

    /// \brief print the Twist2D as "<w [<unit>], x, y>"
    /// An R at the beginning of the format-spec makes [<unit>] rad/s
    /// A  D at the beginning of the format-spec makes [<unit>] deg/s
    /// No R or D means no unit is printed but the w is taken to be in rad/s
    ///
    /// After the optional "unit specifier" all double
    /// format-spec values are accepted and apply to all numbers inserted
    /// into the string.
template<class CharT>
class formatter<turtlelib::Twist2D, CharT>
{
  mutable formatter<double, CharT> double_fmt;
  bool use_degree = false;
  bool print_unit = false;        // whether to print the unit or not

public:
  constexpr auto parse(basic_format_parse_context<CharT> & ctx)
  {
    auto it = ctx.begin();
    const auto end = ctx.end();
    if (it != end) {
      if (*it == 'R') {
        use_degree = false;
        print_unit = true;
        ++it;
      } else if (*it == 'D') {
        use_degree = true;
        print_unit = true;
        ++it;
      }
    }
    ctx.advance_to(it);
    return double_fmt.parse(ctx);
  }

  template<class FormatContext>
  auto format(const turtlelib::Twist2D & tw, FormatContext & ctx) const
  {
    auto out = ctx.out();
    *out++ = '<';
    auto theta = tw.omega;
    if (use_degree) {
      theta = turtlelib::rad2deg(theta);
    }
    out = double_fmt.format(theta, ctx);
    if (print_unit) {
      *out++ = ' ';
      *out++ = '[';
      if (use_degree) {
        *out++ = 'd';
        *out++ = 'e';
        *out++ = 'g';
      } else {
        *out++ = 'r';
        *out++ = 'a';
        *out++ = 'd';
      }
      *out++ = ']';
      *out++ = '/';
      *out++ = 's';
    }
    *out++ = ',';
    *out++ = ' ';
    out = double_fmt.format(tw.x, ctx);
    *out++ = ',';
    *out++ = ' ';
    out = double_fmt.format(tw.y, ctx);
    *out++ = '>';
    return out;
  }
};
}
#endif
