#include "nuturtle_control/circle.hpp"

Circle::Circle()
    : Node("circle_node")
    {
        declare_parameter<int>("frequency", 100);
        get_parameter("frequency", freq);
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000/freq)),
            std::bind(&Circle::timer_callback, this)
        );

        control_service_ = this->create_service<nuturtle_control::srv::Control>(
            "control",
            std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        reverse_service_ = this->create_service<std_srvs::srv::Empty>(
            "reverse",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>)
            {
                velocity_ = -velocity_;
            }
        );

        stop_service_ = this->create_service<std_srvs::srv::Empty>(
            "stop",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>)
            {
                velocity_ = 0.0;
            }
        );
    }

void Circle::timer_callback()
{
    auto message = geometry_msgs::msg::Twist();
    message.angular.z = velocity_;  // Angular velocity
    message.linear.x = velocity_ * radius_; // Linear velocity for circular motion
    cmd_publisher_->publish(message);
}

void Circle::control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
{
    radius_ = request->radius;
    velocity_ = request->velocity;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Circle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}