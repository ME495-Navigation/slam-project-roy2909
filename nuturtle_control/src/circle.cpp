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

class Circle:public rclcpp::Node
{
public:
    Circle()
    :Node("Circle")
    {
    // Frequency
    auto frequency_desc = rcl_interfaces::msg::ParameterDescriptor{};
    frequency_desc.description = "Timer Frequency (Hz)";
    this->declare_parameter("frequency", "100" , frequency_desc);
    frequency_ = this->get_parameter("frequency").as_int();
    //Publisher
    cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //Service
    reverse_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/reverse",
      std::bind(&Circle::reverse_callback, this, _1, _2));

     stop_server_ = this->create_service<std_srvs::srv::Empty>(
      "~/stop",
      std::bind(&Circle::stop_callback, this, _1, _2));
    
    control_server_ = this->create_service<nuturtle_control::srv::Control>(
      "~/control",
      std::bind(&Circle::control_callback, this, _1, _2));
    //timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency_),
      std::bind(&Circle::timer_callback, this));
    
    }
private:

    void control_callback(nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr){
    
    flag_=true;
    vel_=-(request->velocity);
    radius_=request->radius;
    }

    void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
    {
        flag_=true;
        vel_= -vel_;
    }
    void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    flag_=false;
    T_.linear.x=0.0;
    T_.linear.y=0.0;
    T_.angular.z=0.0;
    cmd_vel_pub_->publish(T_);  
  }
  void timer_callback()
  {
    if (flag_==true)
    {
        T_.linear.x=radius_*vel_;
        T_.linear.y=0.0;
        T_.angular.z=vel_;
        cmd_vel_pub_->publish(T_);
        flag_ = false; 
    }
  }

    int frequency_;
    double radius_,vel_;
    bool flag_=false;
    geometry_msgs::msg::Twist T_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_server_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_server_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}