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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

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
  : Node("turtle_control")
  { // wheel_radius
    auto wheel_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
    wheel_radius_desc.description = "Radius of wheels (m)";
    declare_parameter("wheel_radius", 0.033, wheel_radius_desc);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    // track_width
    auto track_width_desc = rcl_interfaces::msg::ParameterDescriptor{};
    track_width_desc.description = "Distance between wheels (m)";
    declare_parameter("track_width", 0.16, track_width_desc);
    track_width_ = get_parameter("track_width").as_double();
    // motor_cmd_max
    auto motor_cmd_max_desc = rcl_interfaces::msg::ParameterDescriptor{};
    motor_cmd_max_desc.description = "The motor command maximum value";
    declare_parameter("motor_cmd_max", 265, motor_cmd_max_desc);
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_int();

    //  motor_cmd_per_rad_sec
    auto motor_cmd_per_rad_sec_desc = rcl_interfaces::msg::ParameterDescriptor{};
    motor_cmd_per_rad_sec_desc.description = "Each motor command unit (mcu) is 0.024 (rad/sec) ";
    declare_parameter("motor_cmd_per_rad_sec", 0.024, motor_cmd_per_rad_sec_desc);
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();

    // encoder_ticks_per_rad
    auto encoder_ticks_per_rad_desc = rcl_interfaces::msg::ParameterDescriptor{};
    encoder_ticks_per_rad_desc.description = "The number of encoder ticks per radian (ticks/rad)";
    declare_parameter("encoder_ticks_per_rad", 651.898646904, encoder_ticks_per_rad_desc);
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();

    // collision_radius
    auto collision_radius_desc = rcl_interfaces::msg::ParameterDescriptor{};
    collision_radius_desc.description =
      " This is some simplified geometry used for collision detection (m)";
    declare_parameter("collision_radius", -1.0, collision_radius_desc);
    collision_radius_ = get_parameter("collision_radius").as_double();
    declare_parameter("rate", 200.0);
    rate_ = get_parameter("rate").as_double();


    if (wheel_radius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1.0 ||
      motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1.0 ||
      collision_radius_ == -1.0)
    {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined ");
      throw std::runtime_error("Parameters not defined!");
    }
    
    


    // Publishers
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(
        &Turtle_control::cmd_vel_callback, this,
        std::placeholders::_1));

    sensor_data_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10, std::bind(
        &Turtle_control::sensor_data_callback_, this,
        std::placeholders::_1));

    // Timer
    timer_ = create_wall_timer(1s / rate_, std::bind(&Turtle_control::timer_callback, this));
    joint_states_.header.frame_id = "red/base_link";
    robot_=turtlelib::DiffDrive{wheel_radius_,track_width_};
    duration = 0.0;
    prev_time_step_ = this->now();
    joint_states_.name={"wheel_left_joint","wheel_right_joint"};
    joint_states_.position={0.0,0.0};
    joint_states_.velocity={0.0,0.0};
    
  }

private:
  // Parameters
  double wheel_radius_, track_width_, motor_cmd_max_;
  double motor_cmd_per_rad_sec_, encoder_ticks_per_rad_, collision_radius_;
  double rate_;
  turtlelib::DiffDrive robot_;
  turtlelib::Twist2D twist_;
  turtlelib::WheelPos vels_;
  nuturtlebot_msgs::msg::WheelCommands wheel_cmd_;
  sensor_msgs::msg::JointState joint_states_;
  double duration;
  rclcpp::Time prev_time_step_;
  double count=1.0;

  // Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscriber_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {

    wheel_cmd_publisher_->publish(wheel_cmd_);

    joint_states_.header.stamp = this->now();
    joint_states_publisher_->publish(joint_states_);
  }
  /// \brief limits the max motor command value
  double Max_limit(double wheel_vel)
  {
    return (wheel_vel >
           motor_cmd_max_) ? motor_cmd_max_ : ((wheel_vel <
           -motor_cmd_max_) ? -motor_cmd_max_ : wheel_vel);
  }
  void cmd_vel_callback(const geometry_msgs::msg::Twist &msg)
  {

    twist_ = {
      static_cast<double>(msg.angular.z),
      static_cast<double>(msg.linear.x),
      static_cast<double>(msg.linear.y)
    };
    // RCLCPP_ERROR(this->get_logger(),"[cmd_vel] Angular: %f    x: %f", twist_.omega, twist_.x);

    vels_ = robot_.InverseKinematics(twist_);
    // RCLCPP_ERROR(this->get_logger(),"[after IK] Left: %f    Right: %f", vels_.left, vels_.right);

    vels_.left = static_cast<int>(vels_.left / motor_cmd_per_rad_sec_);
    vels_.right = static_cast<int>(vels_.right / motor_cmd_per_rad_sec_);
    // RCLCPP_ERROR(this->get_logger(),"[after motor] motor: %f ", motor_cmd_per_rad_sec_);
    // RCLCPP_ERROR(this->get_logger(),"[after static cast] Left vel: %f    Right vel: %f", vels_.left, vels_.right);

    vels_.left = Max_limit(vels_.left);
    vels_.right = Max_limit(vels_.right);
    wheel_cmd_.left_velocity = vels_.left;
    wheel_cmd_.right_velocity = vels_.right;
    // wheel_cmd_publisher_->publish(wheel_cmd_);
    // RCLCPP_ERROR(this->get_logger(),"[published wheel_cmd] Left vel: %d    Right vel: %d", wheel_cmd_.left_velocity, wheel_cmd_.right_velocity);

  }

  void sensor_data_callback_(const nuturtlebot_msgs::msg::SensorData &msg)
  {
    duration = (this->now() - prev_time_step_).seconds();
    RCLCPP_ERROR(this->get_logger(),"timer %f",duration);
   
    joint_states_.position = {
      msg.left_encoder / encoder_ticks_per_rad_,
      msg.right_encoder / encoder_ticks_per_rad_
    };
     joint_states_.velocity = {
      (msg.left_encoder / encoder_ticks_per_rad_) / duration,
      (msg.right_encoder / encoder_ticks_per_rad_) / duration
    };
    prev_time_step_ = this->now();
}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_control>());
  rclcpp::shutdown();
  return 0;
}
