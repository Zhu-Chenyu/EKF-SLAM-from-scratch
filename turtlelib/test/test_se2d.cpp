#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include "turtlelib/se2d.hpp"

using namespace turtlelib;

TEST_CASE("Twist2D input operator")
{
    std::istringstream iss1("<1.0 [rad/s], 2.0, 3.0>");
    Twist2D tw1;
    iss1 >> tw1;
    REQUIRE_THAT(tw1.omega, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(tw1.x, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(tw1.y, Catch::Matchers::WithinAbs(3.0, 1e-9));
    
    std::istringstream iss2("0.5 [deg/s] 4.0 5.0");
    Twist2D tw2;
    iss2 >> tw2;
    REQUIRE_THAT(tw2.omega, Catch::Matchers::WithinAbs(deg2rad(0.5), 1e-9));
    REQUIRE_THAT(tw2.x, Catch::Matchers::WithinAbs(4.0, 1e-9));
    REQUIRE_THAT(tw2.y, Catch::Matchers::WithinAbs(5.0, 1e-9));
    
    std::istringstream iss3("<2.0, 6.0, 7.0>"); // No unit specified
    Twist2D tw3;
    iss3 >> tw3;
    REQUIRE_THAT(tw3.omega, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(tw3.x, Catch::Matchers::WithinAbs(6.0, 1e-9));
    REQUIRE_THAT(tw3.y, Catch::Matchers::WithinAbs(7.0, 1e-9));
    
    std::istringstream iss4("<3.0 [xyz], 8.0, 9.0>"); // Invalid unit
    Twist2D tw4;
    iss4 >> tw4;
    REQUIRE(iss4.fail());
}

TEST_CASE("Transform2D input operator")
{
    std::istringstream iss1("1.0 2.0 3.0");
    Transform2D tf1;
    iss1 >> tf1;
    REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(3.0, 1e-9));
    
    std::istringstream iss2("{0.5 [deg], 4.0 5.0}");
    Transform2D tf2;
    iss2 >> tf2;
    REQUIRE_THAT(tf2.rotation(), Catch::Matchers::WithinAbs(deg2rad(0.5), 1e-9));
    REQUIRE_THAT(tf2.translation().x, Catch::Matchers::WithinAbs(4.0, 1e-9));
    REQUIRE_THAT(tf2.translation().y, Catch::Matchers::WithinAbs(5.0, 1e-9));
    
    std::istringstream iss3("{2.0, 6.0 7.0}"); // No unit specified
    Transform2D tf3;
    iss3 >> tf3;
    REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinAbs(6.0, 1e-9));
    REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinAbs(7.0, 1e-9));
    
    std::istringstream iss4("{3.0 [xyz], 8.0 9.0}"); // Invalid unit
    Transform2D tf4;
    iss4 >> tf4;
    REQUIRE(iss4.fail());
}

TEST_CASE("Translation create transform")
{
    Vector2D trans{3.0, 4.0};
    Transform2D tf(trans);
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(3.0, 1e-9));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(4.0, 1e-9));
    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("Rotation create transform")
{
    double angle = deg2rad(90.0);
    Transform2D tf(angle);
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(angle, 1e-9));
}

TEST_CASE("Translation and Rotation create transform")
{
    Vector2D trans{1.0, 2.0};
    double angle = deg2rad(45.0);
    Transform2D tf(trans, angle);
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(angle, 1e-9));
}

TEST_CASE("Transform2D apply to Point2D")
{
    Transform2D tf(Vector2D{1.0, 0.0}, deg2rad(90.0));
    Point2D p{1.0, 1.0};
    Point2D p_transformed = tf(p);
    REQUIRE_THAT(p_transformed.x, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(p_transformed.y, Catch::Matchers::WithinAbs(1.0, 1e-9));
}

TEST_CASE("Transform2D apply to Vector2D")
{
    Transform2D tf(Vector2D{0.0, 0.0}, deg2rad(90.0));
    Vector2D v{1.0, 0.0};
    Vector2D v_transformed = tf(v);
    REQUIRE_THAT(v_transformed.x, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(v_transformed.y, Catch::Matchers::WithinAbs(1.0, 1e-9));
}

TEST_CASE("Transform2D apply to Twist2D")
{
    Transform2D tf(Vector2D{1.0, 0.0}, deg2rad(90.0));
    Twist2D tw{deg2rad(45.0), 1.0, 0.0};
    Twist2D tw_transformed = tf(tw);
    REQUIRE_THAT(tw_transformed.omega, Catch::Matchers::WithinAbs(deg2rad(45.0), 1e-9));
    REQUIRE_THAT(tw_transformed.x, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(tw_transformed.y, Catch::Matchers::WithinAbs(1.0+deg2rad(45.0), 1e-9));
}

TEST_CASE("Transform2D inverse")
{
    Transform2D tf(Vector2D{2.0, 3.0}, deg2rad(90.0));
    Transform2D tf_inv = tf.inv();
    REQUIRE_THAT(tf_inv.translation().x, Catch::Matchers::WithinAbs(-3.0, 1e-9));
    REQUIRE_THAT(tf_inv.translation().y, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(tf_inv.rotation(), Catch::Matchers::WithinAbs(deg2rad(-90.0), 1e-9));
}

TEST_CASE("Transform2D *=")
{
    Transform2D tf1(Vector2D{1.0, 0.0}, deg2rad(90.0));
    Transform2D tf2(Vector2D{0.0, 1.0}, deg2rad(90.0));
    tf1 *= tf2;
    REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(deg2rad(180.0), 1e-9));
}

TEST_CASE("Translation of Transform2D")
{
    Transform2D tf(Vector2D{5.0, 7.0}, deg2rad(30.0));
    Vector2D trans = tf.translation();
    REQUIRE_THAT(trans.x, Catch::Matchers::WithinAbs(5.0, 1e-9));
    REQUIRE_THAT(trans.y, Catch::Matchers::WithinAbs(7.0, 1e-9));
}

TEST_CASE("Rotation of Transform2D")
{
    Transform2D tf(Vector2D{0.0, 0.0}, deg2rad(60.0));
    double angle = tf.rotation();
    REQUIRE_THAT(angle, Catch::Matchers::WithinAbs(deg2rad(60.0), 1e-9));
}

TEST_CASE("Transform2D *")
{
    Transform2D tf1(Vector2D{1.0, 2.0}, deg2rad(45.0));
    Transform2D tf2(Vector2D{3.0, 4.0}, deg2rad(30.0));
    Transform2D tf3 = tf1 * tf2;
    REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinAbs(1.0 + (std::cos(deg2rad(45.0)) * 3.0 - std::sin(deg2rad(45.0)) * 4.0), 1e-9));
    REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinAbs(2.0 + (std::sin(deg2rad(45.0)) * 3.0 + std::cos(deg2rad(45.0)) * 4.0), 1e-9));
    REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinAbs(deg2rad(75.0), 1e-9));
}