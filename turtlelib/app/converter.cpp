#include <iostream>
#include "turtlelib/angle.hpp"

using namespace std;

int main()
{
  cout << "Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)" << endl;
  double angle;
  string unit;
  auto converted_normal_angle = 0.0;
  while (cin >> angle >> unit) {
    if (unit == "deg") {
      converted_normal_angle = turtlelib::deg2rad(angle);
      converted_normal_angle = turtlelib::normalize_angle(converted_normal_angle);
      cout << angle << "deg is " << converted_normal_angle << "rad." << endl;
    } else if (unit == "rad") {
      converted_normal_angle = turtlelib::normalize_angle(angle);
      converted_normal_angle = turtlelib::rad2deg(converted_normal_angle);
      cout << angle << "rad is " << converted_normal_angle << "deg." << endl;
    } else {
      cerr << "Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)" << endl;
    }
    cout << endl;
  }
  return 0;
}
