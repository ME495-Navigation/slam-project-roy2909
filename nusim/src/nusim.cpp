#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Nusim : public rclcpp::Node
{
public:
    Nusim() : Node("Nusim")

    {

        auto rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
        rate_desc.description = "Timer Frequency(Hz)";
        this->declare_parameter("rate", 200, rate_desc);
        rate_ = this->get_parameter("rate").as_int();
        reset_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&Nusim::Reset_callback, this, _1, _2));
        timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
                                  timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / rate_),
            std::bind(&Nusim::timer_callback, this));
    }

private:
    void Reset_callback(std_srvs::srv::Empty::Request::SharedPtr,
                        std_srvs::srv::Empty::Response::SharedPtr)
    {
        timestep_ = 0;
        RCLCPP_INFO(this->get_logger(), "Reset service is called");
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = timestep_++;
        timestep_publisher_->publish(message);
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
    int rate_;
    size_t timestep_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nusim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}