/// \file
/// \brief The turtle_control node enables control of the turtlebot
///        via geometry_msgs/msg/Twist messages on the cmd_vel topic.
///
/// PARAMETERS:
///     \param wheel_radius (double): Radius of wheels (m)
///     \param track_width (double): Distance between wheels (m)
///     \param motor_cmd_max (int): The motor command maximum value
///     \param motor_cmd_per_rad_sec (double): Each motor command unit (mcu) is 0.024 (rad/sec)
///     \param encoder_ticks_per_rad(double): The number of encoder ticks per radian (ticks/rad)
///     \param collision_radius(double): simplified geometry used for collision detection (m)

/// PUBLISHES:
///     \param ~/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel commands
///     \param ~/joint_states (sensor_msgs::msg::JointState): joint states of robot
/// SUBSCRIBES:
///     \param ~/cmd_vel (geometry_msgs::msg::Twist): Twist values to make move
///     \param ~/sensor_data (nuturtlebot_msgs::msg::SensorData): Wheel encoder data
/// SERVERS:
///      None
/// CLIENTS:
///     None
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
using std::placeholders::_1;

/// \brief The class subscribes to  cmd_vel (geometry_msgs/msg/Twist) and
///        publishes wheel_cmd (nuturtlebot_msgs/msg/WheelCommands) that will make the turtlebot3
///        follow the specified twist. It also subscribes to sensor_data and publishes joinr_states
///        and velocity.
///
///  \param wheel_radius_ (double): Radius of wheels (m)
///  \param track_width_ (double): Distance between wheels (m)
///  \param motor_cmd_max_ (int): The motor command maximum value
///  \param motor_cmd_per_rad_sec_ (double): Each motor command unit (mcu) is 0.024 (rad/sec)
///  \param encoder_ticks_per_rad_ (double): The number of encoder ticks per radian (ticks/rad)
///  \param collision_radius_ (double): simplified geometry used for collision detection (m)

class Turtle_control : public rclcpp::Node
{
public:
  Turtle_control()
  : Node("Turtle_control")
  { // wheel_radius
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_radius_desc.description = "Radius of wheels (m)";
    this->declare_parameter("wheel_radius", -1.0, wheel_radius_desc);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    // track_width
    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
    track_width_desc.description = "Distance between wheels (m)";
    this->declare_parameter("track_width", -1.0, track_width_desc);
    track_width_ = this->get_parameter("track_width").as_double();
    // motor_cmd_max
    auto motor_cmd_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
    motor_cmd_max_desc.description = "The motor command maximum value";
    this->declare_parameter("motor_cmd_max", -1, motor_cmd_max_desc);
    motor_cmd_max_ = this->get_parameter("motor_cmd_max").as_int();

    //  motor_cmd_per_rad_sec
    auto motor_cmd_per_rad_sec_desc = rcl_interfaces::msg::ParameterDescriptor{};
    motor_cmd_per_rad_sec_desc.description = "Each motor command unit (mcu) is 0.024 (rad/sec) ";
    this->declare_parameter("motor_cmd_per_rad_sec", -1.0, motor_cmd_per_rad_sec_desc);
    motor_cmd_per_rad_sec_ = this->get_parameter("motor_cmd_per_rad_sec").as_double();

    // encoder_ticks_per_rad
    auto encoder_ticks_per_rad_desc = rcl_interfaces::msg::ParameterDescriptor{};
    encoder_ticks_per_rad_desc.description = "The number of encoder ticks per radian (ticks/rad)";
    this->declare_parameter("encoder_ticks_per_rad", -1.0, encoder_ticks_per_rad_desc);
    encoder_ticks_per_rad_ = this->get_parameter("encoder_ticks_per_rad").as_double();

    // collision_radius
    auto collision_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
    collision_radius_desc.description =
      " This is some simplified geometry used for collision detection (m)";
    this->declare_parameter("collision_radius", -1.0, collision_radius_desc);
    collision_radius_ = this->get_parameter("collision_radius").as_double();

    if (wheel_radius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1.0 ||
      motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1.0 ||
      collision_radius_ == -1.0)
    {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined ");
      throw std::runtime_error("Parameters not defined!");
    }

    robot_ = turtlelib::DiffDrive(wheel_radius_, track_width_);
    // Publishers
    wheel_cmd_publisher_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd",
      10);
    joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    // Subscribers
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(
        &Turtle_control::cmd_vel_callback,
        this, _1));
    sensor_data_subscriber_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &Turtle_control::sensor_data_callback,
        this, _1));

  }

private:
  /// \brief limits the max motor command value
  double Max_limit(double wheel_vel)
  {
    return (wheel_vel >
           motor_cmd_max_) ? motor_cmd_max_ : ((wheel_vel <
           -motor_cmd_max_) ? -motor_cmd_max_ : wheel_vel);
  }

  /// \brief callback for cmd_vel
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    twist_.x = msg.linear.x;
    twist_.y = msg.linear.y;
    twist_.omega = msg.angular.z;

    wheel_vel_ = robot_.InverseKinematics(twist_);
    wheel_cmd_.right_velocity = static_cast<int>(wheel_vel_.right / motor_cmd_per_rad_sec_);
    wheel_cmd_.left_velocity = static_cast<int>( wheel_vel_.left / motor_cmd_per_rad_sec_);
    wheel_cmd_.right_velocity = Max_limit(wheel_cmd_.right_velocity);
    wheel_cmd_.left_velocity = Max_limit(wheel_cmd_.left_velocity);
    wheel_cmd_publisher_->publish(wheel_cmd_);
  }
  /// \brief sensor data callback
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    joint_states_.header.stamp = msg.stamp;
    joint_states_.name = {"wheel_left_joint", "wheel_right_joint"};

    if (last_time_stamp_ != -1.0) {
      joint_states_.position = {msg.left_encoder / encoder_ticks_per_rad_,
        msg.right_encoder / encoder_ticks_per_rad_};

      double duration_passed = msg.stamp.sec + (msg.stamp.nanosec * 1e-9) - last_time_stamp_;

      // Encoder ticks to rad/s
      joint_states_.velocity = {joint_states_.position.at(0) / duration_passed,
        joint_states_.position.at(1) / duration_passed};
    }

    last_time_stamp_ = msg.stamp.sec + msg.stamp.nanosec * 1e-9;
    joint_states_publisher_->publish(joint_states_);
  }

  // Variables
  double wheel_radius_;
  double track_width_;
  int motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  double last_time_stamp_ = -1.0;
  turtlelib::Twist2D twist_;
  turtlelib::WheelPos wheel_vel_;
  turtlelib::DiffDrive robot_;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;
  sensor_msgs::msg::JointState joint_states_;

  //Pubslishers and Subscribers
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;

};
/// \brief main function for node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_control>());
  rclcpp::shutdown();
  return 0;
}
