#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

TEST_CASE("Initial pose service")
{
    auto node = rclcpp::Node::make_shared("odometry_test_init");

    auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            FAIL("Service not available");
            return;
        }
    }

    auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
    request->x = 1.0;
    request->y = 2.0;
    request->theta = 0.5;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        FAIL("Failed to call service");
        return;
    }

    SUCCEED();
}

TEST_CASE("TF from odom to base_footprint")
{
    auto node = rclcpp::Node::make_shared("odometry_test_tf");

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

    // Publish a JointState to trigger the odometry node to broadcast TF
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.name = {"left_wheel_joint", "wheel_right_joint"};
    msg.position = {0.0, 0.0};
    msg.velocity = {0.0, 0.0};

    // Wait for the transform to be available
    geometry_msgs::msg::TransformStamped transform;
    bool received = false;
    auto start_time = node->now();
    while (rclcpp::ok() && !received && (node->now() - start_time).seconds() < 5.0) {
        pub->publish(msg);
        try {
            transform = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            received = true;
        } catch (const tf2::TransformException &) {}
        rclcpp::spin_some(node);
    }

    REQUIRE(received);
    REQUIRE(transform.header.frame_id == "odom");
    REQUIRE(transform.child_frame_id == "base_footprint");
}