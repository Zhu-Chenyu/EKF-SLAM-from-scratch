#include "nuturtle_control/turtle_control.hpp"

TurtleControl::TurtleControl()
    : Node("turtle_control")
    {
        mv_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&TurtleControl::twist_callback, this, std::placeholders::_1)
        );

        wheel_publisher_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

        sensor_subscription_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
            "sensor_data",
            10,
            std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1)
        );

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // get paramters
        try {
            declare_parameter<double>("wheel_radius");
            declare_parameter<double>("track_width");
            declare_parameter<int>("motor_cmd_max");
            declare_parameter<double>("motor_cmd_per_rad_sec");
            declare_parameter<double>("encoder_ticks_per_rad");
            declare_parameter<double>("collision_radius");

            get_parameter("wheel_radius", wheel_radius_);
            get_parameter("track_width", track_width_);
            get_parameter("motor_cmd_max", motor_cmd_max_);
            get_parameter("motor_cmd_per_rad_sec", motor_cmd_per_rad_sec_);
            get_parameter("encoder_ticks_per_rad", encoder_ticks_per_rad_);
            get_parameter("collision_radius", collision_radius_);

            RCLCPP_INFO_STREAM(this->get_logger(), "wheel_radius: " << wheel_radius_);
            RCLCPP_INFO_STREAM(this->get_logger(), "track_width: " << track_width_);
            RCLCPP_INFO_STREAM(this->get_logger(), "motor_cmd_max: " << motor_cmd_max_);
            RCLCPP_INFO_STREAM(this->get_logger(), "motor_cmd_per_rad_sec: " << motor_cmd_per_rad_sec_);
            RCLCPP_INFO_STREAM(this->get_logger(), "encoder_ticks_per_rad: " << encoder_ticks_per_rad_);
            RCLCPP_INFO_STREAM(this->get_logger(), "collision_radius: " << collision_radius_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        dd = turtlelib::DiffDrive(track_width_, wheel_radius_);
        prev_time_ = this->now();
    }


void TurtleControl::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    turtlelib::Twist2D twist;
    twist.x = msg->linear.x;
    twist.y = msg->linear.y;
    twist.omega = msg->angular.z;

    auto [left_wheel_speed, right_wheel_speed] = this->dd.inverse_kinematics(twist);

    int left_motor_cmd = static_cast<int>(left_wheel_speed / motor_cmd_per_rad_sec_);
    int right_motor_cmd = static_cast<int>(right_wheel_speed / motor_cmd_per_rad_sec_);

    // Clamp motor commands to max
    left_motor_cmd = std::max(std::min(left_motor_cmd, motor_cmd_max_), -motor_cmd_max_);
    right_motor_cmd = std::max(std::min(right_motor_cmd, motor_cmd_max_), -motor_cmd_max_);

    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
    wheel_cmd_msg.left_velocity = left_motor_cmd;
    wheel_cmd_msg.right_velocity = right_motor_cmd;

    wheel_publisher_->publish(wheel_cmd_msg);
}

void TurtleControl::sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
{
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};
    double left_wheel_pos = static_cast<double>(msg->left_encoder) / encoder_ticks_per_rad_;
    double right_wheel_pos = static_cast<double>(msg->right_encoder) / encoder_ticks_per_rad_;
    joint_state_msg.position = {left_wheel_pos, right_wheel_pos};

    double curr_time = this->now().seconds();
    double dt = curr_time - prev_time_.seconds();
    dt = std::max(dt, 1e-6); // prevent division by zero
    joint_state_msg.velocity = {(left_wheel_pos - prev_left_wheel_pos_) / dt, (right_wheel_pos - prev_right_wheel_pos_) / dt};
    joint_state_publisher_->publish(joint_state_msg);
    prev_time_ = this->now();
    prev_left_wheel_pos_ = left_wheel_pos;
    prev_right_wheel_pos_ = right_wheel_pos;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}