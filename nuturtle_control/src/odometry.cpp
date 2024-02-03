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

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("Odometry")
  { // body_id
    auto body_id_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_radius_desc.description = "The name of the body frame of the robot";
    this->declare_parameter("body_id", "blue/base_footprint" , body_id_desc);
    body_id_ = this->get_parameter("body_id").as_string();
    // odom_id
    auto odom_id_desc = rcl_interfaces::msg::ParameterDescriptor{};
    odom_id_desc.description = "The name of the odometry frame";
    this->declare_parameter("odom_id","odom", odom_id_desc);
    odom_id_ = this->get_parameter("odom_id").as_string();
    // wheel_left
    auto wheel_left_desc = rcl_interfaces::msg::ParameterDescriptor{};
    motor_cmd_max_desc.description = "The name of the left wheel joint";
    this->declare_parameter("wheel_left","blue/wheel_left_link", wheel_left_desc);
    wheel_left_ = this->get_parameter("wheel_left").as_string();

    //  wheel_right
    auto  wheel_right_desc = rcl_interfaces::msg::ParameterDescriptor{};
     wheel_right_desc.description = "The name of the right wheel joint ";
    this->declare_parameter("wheel_right", "blue/wheel_right_link", wheel_right_desc);
    wheel_right_ = this->get_parameter("wheel_right").as_string();

    // wheel_radius
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_radius_desc.description = "Radius of wheels (m)";
    this->declare_parameter("wheel_radius", 0.0 , wheel_radius_desc);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();

    // track_width
    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
    track_width_desc.description = "Distance between wheels (m)";
    this->declare_parameter("track_width", 0.0, track_width_desc);
    track_width_ = this->get_parameter("track_width").as_double();

    
    if(body_id_==""||odom_id_=="" || wheel_left_==""||
      wheel_right_=="" )
    {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined " );
      throw std::runtime_error("Parameters not defined!");
    } 

    robot_= turtlelib::DiffDrive(wheel_radius_, track_width_);

    // Publishers
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Subscribers
    joint_state_subscriber_ = this->ccreate_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &Odometry::odometry_callback, 
        this,_1));
    //Service
    initial_pose_server_ = this->create_service<nuturtle_control::srv::InitialPose>(
      "~/initial_pose",
      std::bind(&Odometry::initial_pose_callback, this, _1, _2));
    
      }

private:

void odometry_callback(const sensor_msgs::msg::JointState &msg)
{
    new_wheel_pos.left=msg.position[0] -prev_wheel.left;
    new_wheel_pos.right=msg.position[1] - prev_wheel.right;
    robot_.ForwardKinematics(new_wheel_pos);
    

}

void initial_pose_callback(nuturtle_control::srv::InitialPose::Request::SharedPtr request,
    nuturtle_control::srv::InitialPose::Response::SharedPtr)
    {
        robot_= turtlelib::DiffDrive{wheelradius_,track_width_,{0.0,0.0},{request->x,request->y,request->theta}};
    }

  void broadcast_odom_transform()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;
    t.transform.translation.x = robot_.get_config().x;
    t.transform.translation.y = obot_.get_config().y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q_.x();
    t.transform.rotation.y = q_.y();
    t.transform.rotation.z = q_.z();
    t.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
  // Variables
  double body_id_;
  double odom_id_;
  double wheel_left_;
  double wheel_right_;
  double wheelradius_;
  double track_width_;
  turtlelib::DiffDrive robot_;
  turtlelib::WheelPos prev_wheel{0.0,0.0};
  turtlelib::WheelPos new_wheel_pos;
  tf2::Quaternion q_;

  //Pubslishers and Subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}