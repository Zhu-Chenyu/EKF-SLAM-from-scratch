#include <iostream>
#include <string>
#include "turtlelib/svg.hpp"
#include <cmath>
#include <fstream>
#include <sstream>

namespace turtlelib
{
Svg::Svg() {}

void Svg::draw_point(double x, double y, double radius, std::string color)
{
  double svg_x = 408.0 + (x * 96.0);
  double svg_y = 528.0 - (y * 96.0);
  std::ostringstream oss;
  oss             << "<circle cx=\"" << svg_x << "\" cy=\"" << svg_y << "\" r=\"" << radius
                  << "\" stroke=\"" << color << "\" fill=\"" << color <<
    "\" stroke-width=\"1\" />\n";
  svg_content += oss.str();
}

void Svg::draw_vector(
  double x1, double y1, double x2, double y2, std::string color,
  int stroke_width)
{
  double svg_x1 = 408.0 + (x1 * 96.0);
  double svg_y1 = 528.0 - (y1 * 96.0);
  double svg_x2 = 408.0 + (x2 * 96.0);
  double svg_y2 = 528.0 - (y2 * 96.0);
  std::ostringstream oss;
  oss             << "<line x1=\"" << svg_x1 << "\" x2=\"" << svg_x2 << "\" y1=\"" << svg_y1
                  << "\" y2=\"" << svg_y2 << "\" stroke=\"" << color
                  << "\" stroke-width=\"" << stroke_width <<
    "\" marker-start=\"url(#Arrow1Sstart)\" />\n";
  svg_content += oss.str();
}

void Svg::draw_frame(
  const Transform2D & t, const std::string & name, int axis_length,
  int stroke_width)
{
  Point2D origin = t(Point2D{0.0, 0.0});
  Point2D x_axis_end = t(Point2D{static_cast<double>(axis_length), 0.0});
  Point2D y_axis_end = t(Point2D{0.0, static_cast<double>(axis_length)});

  svg_content += "<g>\n";
  draw_vector(origin.x, origin.y, x_axis_end.x, x_axis_end.y, "red", stroke_width);
  draw_vector(origin.x, origin.y, y_axis_end.x, y_axis_end.y, "green", stroke_width);
  std::ostringstream oss;
  double svg_x = 408.0 + (origin.x * 96.0);
  double svg_y = 528.0 - (origin.y * 96.0);
  oss << "<text x=\"" << svg_x << "\" y=\"" << svg_y << "\">" << name << "</text>\n";
  oss << "</g>\n";
  svg_content += oss.str();
}

std::string Svg::to_string() const
{
  std::string result;
  result +=
    "<svg width=\"8.500000in\" height=\"11.000000in\"  viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";

  result += "<defs>\n";
  result += "<marker style=\"overflow:visible\" id=\"Arrow1Sstart\" "
    "refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n";
  result += "<path transform=\"scale(0.2) translate(6,0)\" "
    "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" "
    "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n";
  result += "</marker>\n";
  result += "</defs>\n";

  result += svg_content;

  result += "</svg>\n";

  return result;
}

void Svg::write_to_file(const std::string & filename) const
{
  std::ofstream file(filename);
  file << to_string();
  file.close();
}
}
