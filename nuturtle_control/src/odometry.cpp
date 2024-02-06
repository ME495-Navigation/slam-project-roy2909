/// \file
/// \brief The node publishes odometry messages and odometry transform
///
/// PARAMETERS:
///     \param body_id (std::string): body frame of the robot
///     \param odom_id (std::string):  odometry frame(odom)
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///     \param wheel_radius (double): Radius of wheels (m)
///     \param track_width (double): Distance between wheels (m)
///
/// PUBLISHES:
///     \param ~/odom (nav_msgs::msg::Odometry): Publishes odometry of robot
///
/// SUBSCRIBES:
///     \param /joint_states (sensor_msgs::msg::JointState): Gets the jpint states of robot
///
/// SERVERS:
///     \param /initial_pose (std_srvs::srv::Empty): Sets initial pose of the robot
///
/// CLIENTS:
///     None
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief The node publishes odometry messages and odometry transform
///        It has an initial pose service to set the initial position of the robot.
///
///  \param body_id_ (std::string): body frame of the robot
///  \param odom_id_ (std::string): odometry frame(odom)
///  \param wheel_left_ (std::string): The name of the left wheel joint
///  \param wheel_right_ (std::string): The name of the right wheel joint
///  \param wheel_radius_ (double): Radius of wheels (m)
///  \param track_width_ (double): Distance between wheels (m)

class odometry : public rclcpp::Node
{
public:
  odometry()
  : Node("odometry")
  { // body_id
    auto body_id_desc = rcl_interfaces::msg::ParameterDescriptor{};
    body_id_desc.description = "body frame of the robot";
    this->declare_parameter("body_id", "blue/base_footprint", body_id_desc);
    body_id_ = this->get_parameter("body_id").as_string();
    // odom_id
    auto odom_id_desc = rcl_interfaces::msg::ParameterDescriptor{};
    odom_id_desc.description = "odometry frame(odom)";
    this->declare_parameter("odom_id", "odom", odom_id_desc);
    odom_id_ = this->get_parameter("odom_id").as_string();
    // wheel_left
    auto wheel_left_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_left_desc.description = "The name of the left wheel joint";
    this->declare_parameter("wheel_left", "", wheel_left_desc);
    wheel_left_ = this->get_parameter("wheel_left").as_string();

    //  wheel_right
    auto wheel_right_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_right_desc.description = "The name of the right wheel joint ";
    this->declare_parameter("wheel_right", " ", wheel_right_desc);
    wheel_right_ = this->get_parameter("wheel_right").as_string();

    // wheel_radius
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_radius_desc.description = "Radius of wheels (m)";
    this->declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();

    // track_width
    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
    track_width_desc.description = "Distance between wheels (m)";
    this->declare_parameter("track_width", -1.0, track_width_desc);
    track_width_ = this->get_parameter("track_width").as_double();


    if (body_id_ == "" || odom_id_ == "" || wheel_left_ == "" ||
      wheel_right_ == "")
    {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined ");
      throw std::runtime_error("Parameters not defined!");
    }

    if (track_width_ == -1.0 || wheel_radius_ == -1.0) {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined ");
      throw std::runtime_error("Parameters not defined!");
    }

    robot_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

    // Publishers
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Subscribers
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &odometry::odometry_callback,
        this, _1));
    //Service
    initial_pose_server_ = this->create_service<nuturtle_control::srv::InitialPose>(
      "~/initial_pose",
      std::bind(&odometry::initial_pose_callback, this, _1, _2));

  }

private:
  /// \brief Publishes the odometry of robot
  void odometry_callback(const sensor_msgs::msg::JointState & msg)
  {
    new_wheel_.left = msg.position[0] - prev_wheel_.left;
    new_wheel_.right = msg.position[1] - prev_wheel_.right;
    robot_.ForwardKinematics(new_wheel_);
    twistb_ = robot_.BodyTwist(new_wheel_);
    q_.setRPY(0, 0, robot_.get_config().theta);
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    odom_.header.stamp = get_clock()->now();
    odom_.pose.pose.position.x = robot_.get_config().x;
    odom_.pose.pose.position.y = robot_.get_config().y;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation.x = q_.x();
    odom_.pose.pose.orientation.y = q_.y();
    odom_.pose.pose.orientation.z = q_.z();
    odom_.pose.pose.orientation.w = q_.w();
    odom_.twist.twist.linear.x = twistb_.x;
    odom_.twist.twist.linear.y = twistb_.y;
    odom_.twist.twist.angular.z = twistb_.omega;
    odometry_publisher_->publish(odom_);
    broadcast_odom_transform();
    prev_wheel_.left = msg.position[0];
    prev_wheel_.right = msg.position[1];
  }
  /// \brief sets inital pose of robot
  void initial_pose_callback(
    nuturtle_control::srv::InitialPose::Request::SharedPtr request,
    nuturtle_control::srv::InitialPose::Response::SharedPtr)
  {
    robot_ = turtlelib::DiffDrive(
      wheel_radius_, track_width_, {0.0, 0.0}, {request->x, request->y,
        request->theta});
  }
  /// \brief Broadcasts odometry transform
  void broadcast_odom_transform()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;
    t.transform.translation.x = robot_.get_config().x;
    t.transform.translation.y = robot_.get_config().y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q_.x();
    t.transform.rotation.y = q_.y();
    t.transform.rotation.z = q_.z();
    t.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
  // Variables
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double wheel_radius_;
  double track_width_;
  turtlelib::DiffDrive robot_;
  turtlelib::WheelPos prev_wheel_{0.0, 0.0};
  turtlelib::WheelPos new_wheel_;
  turtlelib::Twist2D twistb_;
  nav_msgs::msg::Odometry odom_;
  tf2::Quaternion q_;

  //Pubslishers and Subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_server_;
};
/// \brief main funtion of node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odometry>());
  rclcpp::shutdown();
  return 0;
}
