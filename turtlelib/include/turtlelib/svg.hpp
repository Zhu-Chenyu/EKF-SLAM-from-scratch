#ifndef TURTLELIB_SVG_HPP
#define TURTLELIB_SVG_HPP
#include <cmath>
#include <string>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    class Svg
    {
    private:
        std::string svg_content;
    public:
        Svg();
        void draw_point(double x, double y, double radius=3.0, std::string color="black");
        void draw_vector(double x1, double y1, double x2, double y2, std::string color="black", int stroke_width=5);
        void draw_frame(const Transform2D & t, const std::string & name, int axis_length=1, int stroke_width=5);
        std::string to_string() const;
        void write_to_file(const std::string & filename) const;
    };
}
#endif