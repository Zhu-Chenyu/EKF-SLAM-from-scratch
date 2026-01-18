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
        /// \brief Construct an empty Svg object
        Svg();

        /// \brief Draw a point at (x,y) with given radius and color
        /// \param x - x coordinate of the point
        /// \param y - y coordinate of the point
        /// \param radius - radius of the point (default 3.0)
        /// \param color - color of the point (default "black")
        void draw_point(double x, double y, double radius=3.0, std::string color="black");

        /// \brief Draw a vector from (x1,y1) to (x2,y2) with given color and stroke width
        /// \param x1 - x coordinate of the start point
        /// \param y1 - y coordinate of the start point
        /// \param x2 - x coordinate of the end point
        /// \param y2 - y coordinate of the end point
        /// \param color - color of the vector (default "black")
        /// \param stroke_width - stroke width of the vector (default 5)
        void draw_vector(double x1, double y1, double x2, double y2, std::string color="black", int stroke_width=5);
        
        /// \brief Draw a coordinate frame represented by the given Transform2D
        /// \param t - the Transform2D representing the frame
        /// \param name - the name of the frame to be displayed at the origin
        /// \param axis_length - length of the axes (default 1)
        /// \param stroke_width - stroke width of the axes (default 5)
        void draw_frame(const Transform2D & t, const std::string & name, int axis_length=1, int stroke_width=5);
        
        /// \brief Convert the Svg object to a string representation
        /// \returns the string representation of the Svg object
        std::string to_string() const;

        /// \brief Write the Svg content to a file
        /// \param filename - the name of the file to write to
        void write_to_file(const std::string & filename) const;
    };
}
#endif