#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "catch_ros2/catch_ros2.hpp"

TEST_CASE("Twist check")
{
    auto node = rclcpp::Node::make_shared("turtle_circle_test");

    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.5;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 1.0;

    bool received = false;
    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&received](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            REQUIRE_THAT(msg->linear.x, Catch::Matchers::WithinAbs(0.5, 1e-9));
            REQUIRE_THAT(msg->linear.y, Catch::Matchers::WithinAbs(0.0, 1e-9));
            REQUIRE_THAT(msg->angular.z, Catch::Matchers::WithinAbs(1.0, 1e-9));
            received = true;
        }
    );

    auto start_time = node->now();
    while (rclcpp::ok() && !received && (node->now() - start_time).seconds() < 5.0) {
        pub->publish(twist_msg);
        rclcpp::spin_some(node);
    }
    REQUIRE(received);
}