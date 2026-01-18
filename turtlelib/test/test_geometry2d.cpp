# include <catch2/catch_test_macros.hpp>
# include <catch2/matchers/catch_matchers_floating_point.hpp>
# include <sstream>
# include "turtlelib/geometry2d.hpp"

using namespace turtlelib;

TEST_CASE("Vector2D input operator")
{
  std::istringstream iss1("[1.0, 2.0]");
  Vector2D v1;
  iss1 >> v1;
  REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(1.0, 1e-9));
  REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(2.0, 1e-9));

  std::istringstream iss2("3.5 4.5");
  Vector2D v2;
  iss2 >> v2;
  REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(3.5, 1e-9));
  REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(4.5, 1e-9));

  std::istringstream iss3("[5.0 6.0]"); // Missing comma
  Vector2D v3;
  iss3 >> v3;
  REQUIRE(iss3.fail());
}

TEST_CASE("Point2D input operator")
{
    std::istringstream iss1("(1.0, 2.0)");
    Point2D p1;
    iss1 >> p1;
    REQUIRE_THAT(p1.x, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(p1.y, Catch::Matchers::WithinAbs(2.0, 1e-9));

    std::istringstream iss2("3.5 4.5");
    Point2D p2;
    iss2 >> p2;
    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(3.5, 1e-9));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(4.5, 1e-9));

    std::istringstream iss3("(5.0 6.0)"); // Missing comma
    Point2D p3;
    iss3 >> p3;
    REQUIRE(iss3.fail());
}

TEST_CASE("Normalize Vector2D")
{
  Vector2D v1{3.0, 4.0};
  Vector2D norm_v1 = normalize(v1);
  REQUIRE_THAT(norm_v1.x, Catch::Matchers::WithinAbs(0.6, 1e-9));
  REQUIRE_THAT(norm_v1.y, Catch::Matchers::WithinAbs(0.8, 1e-9));

  Vector2D v2{0.0, 0.0};
  REQUIRE_THROWS_AS(normalize(v2), std::invalid_argument);
}

TEST_CASE("Point2D and Vector2D operations")
{
  Point2D p1{1.0, 2.0};
  Point2D p2{4.0, 6.0};
  Vector2D v = p2 - p1;
  REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(3.0, 1e-9));
  REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(4.0, 1e-9));

  Point2D p3 = p1 + v;
  REQUIRE_THAT(p3.x, Catch::Matchers::WithinAbs(4.0, 1e-9));
  REQUIRE_THAT(p3.y, Catch::Matchers::WithinAbs(6.0, 1e-9));
}

TEST_CASE("Output formatting")
{
  Point2D p{1.23456, 7.89012};
  std::string formatted_point = std::format("{:.2f}", p);
  REQUIRE(formatted_point == "(1.23, 7.89)");

  Vector2D v{3.14159, 2.71828};
  std::string formatted_vector = std::format("{:.3f}", v);
  REQUIRE(formatted_vector == "[3.142, 2.718]");
}
