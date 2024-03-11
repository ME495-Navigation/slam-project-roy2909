/// \file 
/// \brief A node responsible for detecting and publishing the locations of landmarks in the robot's environment.
///
/// PARAMETERS:
///   \param estimated_landmark_positions (visualization_msgs::MarkerArray) The estimated positions of the landmarks.
/// SUBSCRIBES:
///   \param /scan (sensor_msgs::LaserScan) The laser scan data.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <armadillo>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/landmark_est.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Create a publisher for the estimated landmark positions.
    estimated_landmark_positions_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("estimated_landmark_positions", 10);
    // Create a subscriber for the laser scan data.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Landmarks::scan_callback, this, _1));
  }

private:

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
std::vector<float> ranges = msg->ranges;
  turtlelib::Landmark landmark= turtlelib::Landmark(ranges, msg->angle_increment, msg->angle_min);
    std::vector<std::vector<turtlelib::Vector2D>> clusters = landmark.cluster_points(0.1);
    visualization_msgs::msg::MarkerArray marker_array;
    auto marker = visualization_msgs::msg::Marker();
    auto cluster_count=1;
    auto cluster_id=0;
    for(const auto &cluster: clusters)
    {  for(const auto &point: cluster)
        {
            RCLCPP_INFO(this->get_logger(), "Landmark: %f, %f", point.x, point.y);
            marker.header.frame_id = "red/base_scan";
            marker.header.stamp = get_clock()->now();
            marker.id = cluster_id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = 0;
            marker.scale.x = 2*0.038;
            marker.scale.y = 2*0.038;
            marker.scale.z = 0.25;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker_array.markers.push_back(marker);
            cluster_id++;
        }
        cluster_count++;
        RCLCPP_INFO(this->get_logger(), "Cluster: %d", cluster_count);
    }
    estimated_landmark_positions_pub_->publish(marker_array);

    }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr estimated_landmark_positions_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}