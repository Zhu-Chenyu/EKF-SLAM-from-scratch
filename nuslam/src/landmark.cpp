/// \file
/// \brief Landmark detection node

///PUBLISH: /landmark
///SUBSCRIBE: /red/scan
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nuslam/circle_fitting.hpp"

class Landmark : public rclcpp::Node {
public:
    Landmark() : rclcpp::Node("landmark") {
        declare_parameter("cluster_threshold", 0.1);
        cluster_threshold_ = get_parameter("cluster_threshold").as_double();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Landmark::scan_callback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("landmark", 10);
    }
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        clusters_.x.clear();
        clusters_.y.clear();
        auto angle_increment = msg->angle_increment;
        auto angle_start = msg->angle_min;

        //process the first scan point
        if (std::isinf(msg->ranges.at(0))) {
            msg->ranges.at(0) = 1e9;
        }
        auto x = msg->ranges.at(0) * std::cos(angle_start);
        auto y = msg->ranges.at(0) * std::sin(angle_start);
        clusters_.x.push_back(std::vector<double>{x});
        clusters_.y.push_back(std::vector<double>{y});
        auto prev_id = 0;
        auto prev_x = x;
        auto prev_y = y;

        for (int i = 1; i < int(msg->ranges.size()); i++) {
            if (std::isinf(msg->ranges.at(i))) {
                msg->ranges.at(i) = 1e9;
            }
            auto x = msg->ranges.at(i) * std::cos(angle_start + i * angle_increment);
            auto y = msg->ranges.at(i) * std::sin(angle_start + i * angle_increment);
            if (std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2)) < cluster_threshold_) {
                clusters_.x.at(prev_id).push_back(x);
                clusters_.y.at(prev_id).push_back(y);
            } else {
                clusters_.x.push_back(std::vector<double>{x});
                clusters_.y.push_back(std::vector<double>{y});
                prev_id++;
            }
            prev_x = x;
            prev_y = y;
        }

        //check if the last cluster is connected to the first cluster
        if (std::sqrt(std::pow(clusters_.x.at(0).at(0) - clusters_.x.back().back(), 2) + std::pow(clusters_.y.at(0).at(0) - clusters_.y.back().back(), 2)) < cluster_threshold_) {
            clusters_.x.at(0).insert(clusters_.x.at(0).end(), clusters_.x.back().begin(), clusters_.x.back().end());
            clusters_.y.at(0).insert(clusters_.y.at(0).end(), clusters_.y.back().begin(), clusters_.y.back().end());
            clusters_.x.pop_back();
            clusters_.y.pop_back();
        }

        for (auto i=0; i<int(clusters_.x.size()); i++) {
            if (clusters_.x.at(i).size() < 4 || !CircleFitting::is_circle(clusters_.x.at(i), clusters_.y.at(i))) {
                clusters_.x.erase(clusters_.x.begin() + i);
                clusters_.y.erase(clusters_.y.begin() + i);
                i--;
            }
        }

        visualization_msgs::msg::MarkerArray marker_array;
        // Tell RViz to delete all previously published markers
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);
        for (auto i=0; i<int(clusters_.x.size()); i++) {
            auto obs = CircleFitting::fit(clusters_.x.at(i), clusters_.y.at(i));
            if (obs.at(2) < 0.5 && obs.at(2) > 0.01) {
                obs_.x.push_back(obs.at(0));
                obs_.y.push_back(obs.at(1));
                obs_.radius = obs.at(2);

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "green/base_footprint";
                marker.header.stamp = this->now();
                marker.ns = "basic_shapes";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = obs.at(0);
                marker.pose.position.y = obs.at(1);
                marker.pose.position.z = 0.1;
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 1;
                marker.scale.x = obs.at(2) * 2;
                marker.scale.y = obs.at(2) * 2;
                marker.scale.z = 0.2;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);
    }

    struct point_clusters {
        std::vector<std::vector<double>> x;
        std::vector<std::vector<double>> y;
    };
    point_clusters clusters_;
    
    struct Obstacle
    {
        std::vector<double> x;
        std::vector<double> y;
        double radius;
    };
    Obstacle obs_;

    double cluster_threshold_ = 0.1;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Landmark>());
    rclcpp::shutdown();
    return 0;
}
