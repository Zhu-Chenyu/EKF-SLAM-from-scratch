#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/se2d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

TEST_CASE("Pure translation forward")
{
    auto node = rclcpp::Node::make_shared("turtle_control_test");

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 1.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    bool received = false;
    auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd", 10,
        [&received](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
        {
            REQUIRE_THAT(msg->left_velocity, Catch::Matchers::WithinAbs(265, 1e-9));
            REQUIRE_THAT(msg->right_velocity, Catch::Matchers::WithinAbs(265, 1e-9));
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

TEST_CASE("Pure rotation in place")
{
    auto node = rclcpp::Node::make_shared("turtle_control_test");

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 1.0;
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    bool received = false;
    auto sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd", 10,
        [&received](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
        {
            REQUIRE_THAT(msg->left_velocity, Catch::Matchers::WithinAbs(-101, 1e-9));
            REQUIRE_THAT(msg->right_velocity, Catch::Matchers::WithinAbs(101, 1e-9));
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

TEST_CASE("Convert sensor encoding to joint states")
{
    auto node = rclcpp::Node::make_shared("turtle_control_test");

    nuturtlebot_msgs::msg::SensorData sensor_msg;
    sensor_msg.left_encoder = 1304;
    sensor_msg.right_encoder = 652;

    auto pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
    bool received = false;
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [&received](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            REQUIRE_THAT(msg->position[0], Catch::Matchers::WithinAbs(2.0, 1e-2));
            REQUIRE_THAT(msg->position[1], Catch::Matchers::WithinAbs(1.0, 1e-2));
            received = true;
        }
    );
    auto start_time = node->now();
    while (rclcpp::ok() && !received && (node->now() - start_time).seconds() < 5.0) {
        pub->publish(sensor_msg);
        rclcpp::spin_some(node);
    }
    REQUIRE(received);
}