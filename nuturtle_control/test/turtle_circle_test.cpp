#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "catch_ros2/catch_ros2.hpp"

TEST_CASE("Twist check")
{
    auto node = rclcpp::Node::make_shared("turtle_circle_test");

    ////////////////citation 2/////////////////////////////
    const size_t num_messages = 10;
    std::vector<rclcpp::Time> timestamps;

    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
    [&](const geometry_msgs::msg::Twist::SharedPtr)
    {
      timestamps.push_back(node->now());
      }
    );

    auto start_time = node->now();
    while (rclcpp::ok() && timestamps.size() < num_messages &&
    (node->now() - start_time).seconds() < 5.0)
  {
    rclcpp::spin_some(node);
    }

    REQUIRE(timestamps.size() >= num_messages);
    auto total_time = (timestamps.back() - timestamps.front()).seconds();
    ///////////////////citation 2//////////////////////////

    auto interval = total_time / static_cast<double>(num_messages - 1);
    auto expected_interval = 1.0 / 100.0;

    REQUIRE_THAT(interval, Catch::Matchers::WithinAbs(expected_interval, 0.005));
}
