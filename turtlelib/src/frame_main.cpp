#include <iostream>
#include <string>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <cmath>

int main()
{
  using namespace turtlelib;

  Svg svg;

  std::cerr << "Enter Transform2D Tab:" << std::endl;
  Transform2D tab;
  std::cin >> tab;
  std::cerr << "Enter Transform2D tbc:" << std::endl;
  Transform2D tbc;
  std::cin >> tbc;
  std::cerr << "Ta, Tb, Tc will be shown in svg" << std::endl;
  Transform2D ta;
  Transform2D tba = tab.inv();
  Transform2D tcb = tbc.inv();
  Transform2D tac = tab * tbc;
  Transform2D tca = tac.inv();

  svg.draw_frame(ta, "Ta", 1, 5);
  svg.draw_frame(tab, "Tb", 1, 5);
  svg.draw_frame(tac, "Tc", 1, 5);
  std::cout << std::format("Transform2D Tab: {}\n", tab);
  std::cout << std::format("Transform2D Tbc: {}\n", tbc);
  std::cout << std::format("Transform2D Tac: {}\n", tac);
  std::cout << std::format("Transform2D Tba: {}\n", tba);
  std::cout << std::format("Transform2D Tcb: {}\n", tcb);
  std::cout << std::format("Transform2D Tca: {}\n", tca);

  std::cerr << "Enter a Point2D p in frame A:" << std::endl;
  Point2D p;
  std::cin >> p;
  svg.draw_point(p.x, p.y, 3.0, "purple");
  Point2D p_in_b = tab.inv()(p);
  Point2D p_in_c = tac.inv()(p);
  svg.draw_point(p_in_b.x, p_in_b.y, 3.0, "brown");
  svg.draw_point(p_in_c.x, p_in_c.y, 3.0, "orange");
  std::cout << std::format("Point p in frame A: {}\n", p);
  std::cout << std::format("Point p in frame B: {}\n", p_in_b);
  std::cout << std::format("Point p in frame C: {}\n", p_in_c);

  std::cerr << "Enter a Vector2D v in frame B:" << std::endl;
  Vector2D v;
  std::cin >> v;
  Vector2D v_norm = normalize(v);
  svg.draw_vector(tab.translation().x, tab.translation().y, v_norm.x, v_norm.y, "brown", 5);
  svg.draw_vector(tab.translation().x, tab.translation().y, v.x, v.y, "black", 5);

  Vector2D v_in_a = tba(v);
  Vector2D v_in_c = tcb(v);
  std::cout << std::format("Vector v in frame B: {}\n", v);
  std::cout << std::format("Vector v in frame A: {}\n", v_in_a);
  std::cout << std::format("Vector v in frame C: {}\n", v_in_c);
  svg.draw_vector(0.0, 0.0, v_in_a.x, v_in_a.y, "purple", 5);
  svg.draw_vector(tac.translation().x, tac.translation().y, v_in_c.x, v_in_c.y, "orange", 5);

  svg.write_to_file("/tmp/frames.svg");

  return 0;
}
