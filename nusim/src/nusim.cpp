#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Nusim : public rclcpp::Node
{
public:
    Nusim() : Node("Nusim")

    { // rate
        auto rate_desc = rcl_interfaces::msg::ParameterDescriptor{};
        rate_desc.description = "Timer Frequency(Hz)";
        this->declare_parameter("rate", 200, rate_desc);
        rate_ = this->get_parameter("rate").as_int();
        // x0
        auto x0_desc = rcl_interfaces::msg::ParameterDescriptor{};
        x0_desc.description = "Initail x coordiante of robot (m)";
        this->declare_parameter("x0", 0.0, x0_desc);
        x0_ = this->get_parameter("x0").as_double();
        // y0
        auto y0_desc = rcl_interfaces::msg::ParameterDescriptor{};
        y0_desc.description = "Initail y coordiante of robot (m)";
        this->declare_parameter("y0", 0.0, y0_desc);
        y0_ = this->get_parameter("y0").as_double();

        // theta
        auto theta0_desc = rcl_interfaces::msg::ParameterDescriptor{};
        theta0_desc.description = "Initail theta coordiante of robot (m)";
        this->declare_parameter("theta0", 0.0, theta0_desc);
        theta0_ = this->get_parameter("theta0").as_double();

        // arena_walls
        auto arena_x_desc = rcl_interfaces::msg::ParameterDescriptor{};
        arena_x_desc.description = "Length of arena in world x direction(m)";
        this->declare_parameter("walls.arena_x_length", 0.0, arena_x_desc);
        arena_x_length_ = this->get_parameter("walls.arena_x_length").as_double();

        auto arena_y_desc = rcl_interfaces::msg::ParameterDescriptor{};
        arena_y_desc.description = "Length of arena in world y direction(m)";
        this->declare_parameter("walls.arena_y_length", 0.0, arena_y_desc);
        arena_y_length_ = this->get_parameter("walls.arena_y_length").as_double();

        //obstacles
        auto obstacle_y_desc = rcl_interfaces::msg::ParameterDescriptor{};
        obstacle_y_desc.description = "List of the obstacles' y coordinates";
        this->declare_parameter("obstacles.y", std::vector<double>{}, obstacle_y_desc);
        obstacles_y_ = this->get_parameter("obstacles.y").as_double_array();

        auto obstacle_x_desc = rcl_interfaces::msg::ParameterDescriptor{};
        obstacle_x_desc.description = "List of the obstacles' x coordinates";
        this->declare_parameter("obstacles.x", std::vector<double>{}, obstacle_x_desc);
        obstacles_x_ = this->get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();

         auto obstacle_r_desc = rcl_interfaces::msg::ParameterDescriptor{};
        obstacle_r_desc.description = "obstacles' radius";
        this->declare_parameter("obstacles.r", 0.0, obstacle_r_desc);
        obstacles_r_ = this->get_parameter("obstacles.r").as_double();

        reset_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&Nusim::Reset_callback, this, _1, _2));
        teleport_ = this->create_service<nusim::srv::Teleport>(
            "~/teleport",
            std::bind(&Nusim::Teleport_callback, this, _1, _2));
        timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
        obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / rate_),
            std::bind(&Nusim::timer_callback, this));
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        x_ = x0_;
        y_ = y0_;
        theta_ = theta0_;
        thickness_ = 0.10;
        height_=0.25;
        x_pos_ = {
            0.0,
            0.0,
            arena_x_length_ / 2 + thickness_ / 2,
            -arena_x_length_ / 2 - thickness_ / 2,
        };
        y_pos_ = {
            arena_y_length_ / 2 + thickness_ / 2,
            -arena_y_length_ / 2 - thickness_ / 2,
            0.0,
            0.0,
        };
        x_scale_ = {
            arena_x_length_ + 2.0 * thickness_,
            arena_x_length_ + 2.0 * thickness_,
            thickness_,
            thickness_,
        };
        y_scale_ = {
            thickness_,
            thickness_,
            arena_y_length_,
            arena_y_length_,
        };
        wall_marker();
        obstacle_marker();
    }

private:
    void Reset_callback(std_srvs::srv::Empty::Request::SharedPtr,
                        std_srvs::srv::Empty::Response::SharedPtr)
    {
        timestep_ = 0;
        x_ = x0_;
        y_ = y0_;
        theta_ = theta0_;
        RCLCPP_INFO(this->get_logger(), "Reset service is called");
    }

    void Teleport_callback(nusim::srv::Teleport::Request::SharedPtr request,
                           nusim::srv::Teleport::Response::SharedPtr)
    {
        x_ = request->x;
        y_ = request->y;
        theta_ = request->theta;
    }

    void wall_marker()
    {
        for (int i = 0; i < 4; i++)
        {
            visualization_msgs::msg::Marker walls;
            walls.header.frame_id = "nusim/world";
            walls.header.stamp = get_clock()->now();
            walls.id = i;
            walls.type = visualization_msgs::msg::Marker::CUBE;
            walls.action = visualization_msgs::msg::Marker::ADD;
            walls.pose.position.x = x_pos_.at(i);
            walls.pose.position.y = y_pos_.at(i);
            walls.scale.x = x_scale_.at(i);
            walls.scale.y = y_scale_.at(i);
            walls.color.r = 1.0;
            walls.color.g = 0.0;
            walls.color.b = 0.0;
            walls.color.a = 1.0;
            walls.pose.position.z = 0.0;
            walls.scale.z = 0.25;
            wall_array_.markers.push_back(walls);
        }
    }

    void obstacle_marker()
    {


    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Lengths of obstacles/x and obstacles/y must be the same.");
            rclcpp::shutdown();  
            return;
    }

    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      visualization_msgs::msg::Marker obs;
      obs.header.frame_id = "nusim/world";
      obs.header.stamp = get_clock()->now();
      obs.id = i;
      obs.type = visualization_msgs::msg::Marker::CYLINDER;
      obs.action = visualization_msgs::msg::Marker::ADD;
      obs.pose.position.x = obstacles_x_.at(i);
      obs.pose.position.y = obstacles_y_.at(i);
      obs.pose.position.z = 0.0;
      obs.scale.x = 2.0 * obstacles_r_;
      obs.scale.y = 2.0 * obstacles_r_;
      obs.scale.z = 0.25;
      obs.color.r = 1.0;
      obs.color.g = 0.0;
      obs.color.b = 0.0;
      obs.color.a = 1.0;
      obstacle_array_.markers.push_back(obs);
    }
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::UInt64();
        message.data = timestep_++;
        timestep_publisher_->publish(message);
        broadcast_red_transform();
        walls_publisher_->publish(wall_array_);
        obstacles_publisher_->publish(obstacle_array_);
    }

    void broadcast_red_transform()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "nusim/world";
        t.child_frame_id = "red/base_footprint";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    };

    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
    int rate_;
    size_t timestep_;
    double x0_ = 0.0;
    double y0_ = 0.0;
    double theta0_ = 0.0;
    double x_, y_, theta_;
    visualization_msgs::msg::MarkerArray wall_array_;
    visualization_msgs::msg::MarkerArray obstacle_array_;
    double arena_x_length_;
    double thickness_;
    double arena_y_length_;
    std::vector<double> x_pos_, y_pos_, x_scale_, y_scale_;
    std::vector<double> obstacles_x_ , obstacles_y_;
    double obstacles_r_;
    double height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nusim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}