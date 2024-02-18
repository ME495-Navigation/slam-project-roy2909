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
///     \param ~/joint_states (sensor_msgs::msg::JointState): Gets the jpint states of robot
///
/// SERVERS:
///     \param ~/initial_pose (std_srvs::srv::Empty): Sets initial pose of the robot
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
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
    declare_parameter("body_id", "blue/base_footprint");
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    // odom_id
    declare_parameter("odom_id", "odom");
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();

    declare_parameter("wheel_left", "wheel_left_joint");
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    declare_parameter("wheel_right", "wheel_right_joint");
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    declare_parameter("wheel_radius", 0.033);
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();

    declare_parameter("track_width", 0.16);
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();


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

    robot_ = turtlelib::DiffDrive{wheel_radius_, track_width_};

    // Publishers
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    bpath_publisher_ =create_publisher<nav_msgs::msg::Path>("blue/path",10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Subscribers
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &odometry::odometry_callback,
        this, _1));
    //Service
    initial_pose_server_ = create_service<nuturtle_control::srv::InitialPose>(
      "~/initial_pose",
      std::bind(&odometry::initial_pose_callback, this, _1, _2));
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;

  }

private:
  /// \brief Publishes the odometry of robot
  void odometry_callback(const sensor_msgs::msg::JointState & msg)
  {
    //update odometry
    odom_.header.stamp = get_clock()->now();
    new_wheel_.left = msg.position.at(0) - prev_wheel_.left;
    new_wheel_.right = msg.position.at(1) - prev_wheel_.right;
    prev_wheel_.left = msg.position.at(0);
    prev_wheel_.right = msg.position.at(1);
    robot_.ForwardKinematics(new_wheel_);
    Q = robot_.get_config();
    twistb_ = robot_.BodyTwist(new_wheel_);
    q_.setRPY(0, 0, Q.theta);
    odom_.pose.pose.position.x = Q.x;
    odom_.pose.pose.position.y = Q.y;
    odom_.pose.pose.orientation.x = q_.x();
    odom_.pose.pose.orientation.y = q_.y();
    odom_.pose.pose.orientation.z = q_.z();
    odom_.pose.pose.orientation.w = q_.w();

    odom_.twist.twist.linear.x = twistb_.x;
    odom_.twist.twist.linear.y = twistb_.y;
    odom_.twist.twist.angular.z = twistb_.omega;


    geometry_msgs::msg::TransformStamped t;
    //create transform
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;
    t.transform.translation.x = Q.x;
    t.transform.translation.y = Q.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q_.x();
    t.transform.rotation.y = q_.y();
    t.transform.rotation.z = q_.z();
    t.transform.rotation.w = q_.w();
    blue_pose_.header.frame_id="odom";
    blue_pose_.header.stamp=get_clock()->now();
    blue_pose_.pose.position.x=Q.x;
    blue_pose_.pose.position.y=Q.y;
    blue_pose_.pose.position.z=0.0;
    blue_pose_.pose.orientation.x=q_.x();
    blue_pose_.pose.orientation.y=q_.y();
    blue_pose_.pose.orientation.z=q_.z();
    blue_pose_.pose.orientation.w=q_.w();
    blue_path_.header.frame_id="odom";
    blue_path_.header.stamp=get_clock()->now();
    blue_path_.poses.push_back(blue_pose_);
    

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    odometry_publisher_->publish(odom_);
    //publish path
    bpath_publisher_->publish(blue_path_);
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
  turtlelib::RobotConfig Q;
  nav_msgs::msg::Path blue_path_;
  geometry_msgs::msg::PoseStamped blue_pose_;
  
  //Pubslishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bpath_publisher_;
  //Service
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
