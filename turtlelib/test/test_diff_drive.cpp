# include <catch2/catch_test_macros.hpp>
# include <catch2/matchers/catch_matchers_floating_point.hpp>
# include "turtlelib/diff_drive.hpp"

using namespace turtlelib;

TEST_CASE("DiffDrive moving forward")
{
    DiffDrive robot(0.5, 0.1); // wheel_track = 0.5m, wheel_radius = 0.1m

    robot.forward_kinematics(1.0, 1.0);
    REQUIRE_THAT(robot.get_x(), Catch::Matchers::WithinAbs(0.1, 1e-9));
    REQUIRE_THAT(robot.get_y(), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(robot.get_theta(), Catch::Matchers::WithinAbs(0.0, 1e-9));

    Twist2D twist;
    twist.x = 0.2;
    twist.y = 0.0;
    twist.omega = 0.0;
    auto [left_vel, right_vel] = robot.inverse_kinematics(twist);
    REQUIRE_THAT(left_vel, Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(right_vel, Catch::Matchers::WithinAbs(2.0, 1e-9));
}

TEST_CASE("DiffDrive rotating in place")
{
    DiffDrive robot(0.5, 0.1);

    robot.forward_kinematics(1.0, -1.0);
    REQUIRE_THAT(robot.get_x(), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(robot.get_y(), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(robot.get_theta(), Catch::Matchers::WithinAbs(-0.4, 1e-9));

    Twist2D twist;
    twist.x = 0.0;
    twist.y = 0.0;
    twist.omega = 1.0;
    auto [left_vel, right_vel] = robot.inverse_kinematics(twist);
    REQUIRE_THAT(left_vel, Catch::Matchers::WithinAbs(-2.5, 1e-9));
    REQUIRE_THAT(right_vel, Catch::Matchers::WithinAbs(2.5, 1e-9));
}

TEST_CASE("DiffDrive moving in an arc")
{
    DiffDrive robot(0.5, 0.1);

    robot.forward_kinematics(2.0, 4.0);
    REQUIRE_THAT(robot.get_x(), Catch::Matchers::WithinAbs(1.5, 1e-9));
    REQUIRE_THAT(robot.get_y(), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(robot.get_theta(), Catch::Matchers::WithinAbs(0.4, 1e-9));

    Twist2D twist;
    twist.x = 0.1;
    twist.y = 0.0;
    twist.omega = 0.5;
    auto [left_vel, right_vel] = robot.inverse_kinematics(twist);
    REQUIRE_THAT(left_vel, Catch::Matchers::WithinAbs(-0.25, 1e-9));
    REQUIRE_THAT(right_vel, Catch::Matchers::WithinAbs(2.25, 1e-9));
}

TEST_CASE("DiffDrive inverse kinematics with lateral velocity")
{
    DiffDrive robot(0.5, 0.1);

    Twist2D twist;
    twist.x = 0.1;
    twist.y = 0.1; // non-zero lateral velocity
    twist.omega = 0.0;

    REQUIRE_THROWS_AS(robot.inverse_kinematics(twist), std::logic_error);
}
