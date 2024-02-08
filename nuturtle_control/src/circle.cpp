/// \file
/// \brief The circle node publishes to cmd_vel commands to cause the robot.
///        to drive in a circle of a specified radius at a specified speed.
///
/// PARAMETERS:
///     \param frequency (int): Timer frequency (Hz)
///
/// PUBLISHES:
///     \param ~/cmd_vel (geometry_msgs::msg::Twist): Twist message to cmd_vel topic
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     \param ~/reverse (std_srvs::srv::Empty): REverses direction of robot
///     \param ~/stop (std_srvs::srv::Empty): Stops robot
///     \param ~/control (nuturtle_control::srv::Control): Sets radius and velocity of robot
///
/// CLIENTS:
///     None
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief This class publishes cmd_vel commands to make the robot
///        drive in a circle at fixed speed and radius. It has three services
///        stop,revese and control as described above.
///

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("Circle")
  {
    // Frequency
    auto frequency_desc = rcl_interfaces::msg::ParameterDescriptor{};
    frequency_desc.description = "Timer Frequency (Hz)";
    this->declare_parameter("frequency", 100 , frequency_desc);
    frequency_ = this->get_parameter("frequency").as_int();
    //Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //Service
    reverse_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/reverse",
      std::bind(&Circle::reverse_callback, this, _1, _2));

    stop_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/stop", std::bind(&Circle::stop_callback, this, _1, _2));

    control_server_ = this->create_service<nuturtle_control::srv::Control>(
      "~/control",
      std::bind(&Circle::control_callback, this, _1, _2));
    //timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency_),
      std::bind(&Circle::timer_callback, this));
  }

private:
  /// \brief Sets radius and velocity
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
   
    T_.linear.x = request->radius * request->velocity;
    T_.linear.y = 0.0;
    T_.angular.z = request->velocity;
  }
  /// \brief reverses the robot
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    
    T_.angular.z = -T_.angular.z;
    T_.linear.x = -T_.linear.x;
  }
  /// \brief stops the robot
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    
    T_.linear.x = 0.0;
    T_.linear.y = 0.0;
    T_.angular.z = 0.0;

  }
  /// \brief updates at fixed rate
  void timer_callback()
  {
     
      cmd_vel_pub_->publish(T_);
     
    
  }

  int frequency_;
  double radius_, vel_;
   geometry_msgs::msg::Twist T;
  geometry_msgs::msg::Twist T_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
  rclcpp::TimerBase::SharedPtr timer_;

};
/// \brief main funtion of node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
