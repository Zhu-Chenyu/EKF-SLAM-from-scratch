#include "nuturtle_control/odometry.hpp"


Odometry::Odometry()
: Node("odometry_node")
{
  declare_parameter<std::string>("body_id", "base_footprint");
  declare_parameter<std::string>("odom_id", "odom");
  declare_parameter<std::string>("wheel_left");
  declare_parameter<std::string>("wheel_right");

  get_parameter("body_id", this->body_id_);
  get_parameter("odom_id", this->odom_id_);

  declare_parameter<double>("wheel_radius");
  declare_parameter<double>("track_width");

  try {
    get_parameter("wheel_radius", wheel_radius_);
    get_parameter("track_width", track_width_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  try {
    get_parameter("wheel_left", this->wheel_left_);
    get_parameter("wheel_right", this->wheel_right_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&Odometry::joint_callback, this, std::placeholders::_1)
  );

  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  odom_service_ = this->create_service<nuturtle_control::srv::InitialPose>(
            "initial_pose",
            std::bind(&Odometry::init_pose_callback, this, std::placeholders::_1,
    std::placeholders::_2)
  );

  dd = turtlelib::DiffDrive(track_width_, wheel_radius_);
}


void Odometry::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  nav_msgs::msg::Odometry odom_msg;

  if (msg->position.size() >= 2) {
    double new_left_wheel_position_ = msg->position[0];
    double new_right_wheel_position_ = msg->position[1];
    this->dd.forward_kinematics(new_left_wheel_position_, new_right_wheel_position_);
  }
  if (msg->velocity.size() >= 2) {
    double linear_velocity = (msg->velocity[0] + msg->velocity[1]) * this->dd.get_wheel_radius() /
      2.0;
    double angular_velocity = (msg->velocity[1] - msg->velocity[0]) * this->dd.get_wheel_radius() /
      this->dd.get_wheel_track();
    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = angular_velocity;
  }

    // Publish odometry message
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = odom_id_;
  odom_msg.child_frame_id = body_id_;

  odom_msg.pose.pose.position.x = this->dd.get_x();
  odom_msg.pose.pose.position.y = this->dd.get_y();
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(this->dd.get_theta() / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(this->dd.get_theta() / 2.0);

  this->odom_publisher_->publish(odom_msg);

    // Broadcast TF transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = odom_id_;
  transform_stamped.child_frame_id = body_id_;
  transform_stamped.transform.translation.x = this->dd.get_x();
  transform_stamped.transform.translation.y = this->dd.get_y();
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = std::sin(this->dd.get_theta() / 2.0);
  transform_stamped.transform.rotation.w = std::cos(this->dd.get_theta() / 2.0);
  this->tf_broadcaster_->sendTransform(transform_stamped);

}

void Odometry::init_pose_callback(
  const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
  std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
{
  this->dd.set_x(request->x);
  this->dd.set_y(request->y);
  this->dd.set_theta(request->theta);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Odometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
