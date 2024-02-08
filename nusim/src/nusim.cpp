/// \file
/// \brief The nusim node provides a simulated robot environment.
///        It uses Rviz2 for simulation. It is capable of creating
///        stationary walls and tracking the position of a robot.
///
/// PARAMETERS:
///     \param rate (int): Timer frequency (Hz)
///     \param x0 (double): Initial x coordinate of the robot (m)
///     \param y0 (double): Initial y coordinate of the robot (m)
///     \param theta0 (double): Initial theta angle of the robot (radians)
///     \param walls.arena_x_length (double): Length of arena in world x direction (m)
///     \param walls.arena_y_length (double): Length of arena in world y direction (m)
///     \param obstacles.x (std::vector<double>): List of the obstacles' x coordinates (m)
///     \param obstacles.y (std::vector<double>): List of the obstacles' y coordinates (m)
///     \param obstacles.r (double): Radius of cylindrical obstacles (m)

/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): current timestep of the simulation
///     \param ~/walls (visualization_msgs::msg::MarkerArray): MarkerArray of Walls in Rviz2
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): MarkerArray of cylindrial obstacles in Rviz2
/// SUBSCRIBES:
///     None
/// SERVERS:
///      \param ~/reset (std_srvs::srv::Empty): Resets simulation
///      \param ~/teleport (nusim::srv::Teleport): Teleports robot to a dessired position
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
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

/// \brief This class provides a simulator environment for the robot.
///        It publishes the current timestep of the simulation, along
///        with an arena and cylindrical obstacles as markers in Rviz2.
///        It offers services like reset and teleport.
///
///  \param rate (int): Timer frequency (Hz)
///  \param x0_ (double): Initial x coordinate of the robot (m)
///  \param y0_ (double): Initial y coordinate of the robot (m)
///  \param theta0_ (double): Initial theta angle of the robot (radians)
///  \param x_ (double): Current x coordinate of the robot (m)
///  \param y_ (double): Current y coordinate of the robot (m)
///  \param theta_ (double): Current theta angle of the robot (radians)
///  \param arena_x_length_ (double): Length of arena in world x direction (m)
///  \param arena_y_length_ (double): Length of arena in world y direction (m)
///  \param obstacles_x_ (std::vector<double>): List of the obstacles' x coordinates (m)
///  \param obstacles_y_ (std::vector<double>): List of the obstacles' y coordinates (m)
///  \param obstacles_r_ (double): Radius of cylindrical obstacles (m)
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    // Parameters
    declare_parameter("rate", 200);
    declare_parameter("x0", -0.5);
    declare_parameter("y0", 0.7);
    declare_parameter("theta0", 1.28);
    declare_parameter("walls.arena_x_length", 10.0);
    declare_parameter("walls.arena_y_length", 10.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.038);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.8986469);

    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    arena_x_length_ = get_parameter("walls.arena_x_length").get_parameter_value().get<double>();
    arena_y_length_ = get_parameter("walls.arena_y_length").get_parameter_value().get<double>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();

    // Publishers
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    walls_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);

    obstacles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", 10);

    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);


    // Subscribers
    wheel_cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));

    // Services
    reset_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    teleport_ = this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&Nusim::timer_callback, this));

    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;

    thickness_ = 0.10;
    height_ = 0.25;

    // X coordinates of walls
    x_pos_ = {
      0.0,
      0.0,
      arena_x_length_ / 2 + thickness_ / 2,
      -arena_x_length_ / 2 - thickness_ / 2,
    };
    // Y coordinates of walls
    y_pos_ = {
      arena_y_length_ / 2 + thickness_ / 2,
      -arena_y_length_ / 2 - thickness_ / 2,
      0.0,
      0.0,
    };
    // X coordinate scaling of walls
    x_scale_ = {
      arena_x_length_ + 2.0 * thickness_,
      arena_x_length_ + 2.0 * thickness_,
      thickness_,
      thickness_,
    };
    // y coordinate scaling of walls
    y_scale_ = {
      thickness_,
      thickness_,
      arena_y_length_,
      arena_y_length_,
    };
    //
    // Walls
    wall_marker();
    // Obstacles
    obstacle_marker();
  }

private:
  /// \brief Main timer callback function
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;

    // Publish the UInt64 message
    timestep_publisher_->publish(message);

    // Read message content and assign it to corresponding tf variables
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    // Turtle only exists in 2D, so we set z coordinate to 0
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    // Likewise, turtle can only rotate around one axis -- z
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transform
    tf_broadcaster_->sendTransform(t);

    walls_publisher_->publish(wall_array_);
    obstacles_publisher_->publish(obstacle_array_);

    update_robot_position();
    update_wheel_positions();

  }

  void update_robot_position()
  {
    // Do forward kinematics
    turtlelib::WheelPos delta_wheels;
    delta_wheels.left = updated_wheel_pos_.left;
    delta_wheels.right = updated_wheel_pos_.right;
    robot_.ForwardKinematics(delta_wheels);

    // Extract new positions
    x_ = robot_.get_config().x;
    y_ = robot_.get_config().y;
    theta_ = robot_.get_config().theta;


    // RCLCPP_ERROR(this->get_logger(),"x = %f", x_);
    // printf("Updated robot position: x = %f, y = %f, theta = %f\n", x_, y_, theta_);


  }

  void update_wheel_positions()
  {
    double unit_per_run = 1.0 / rate_;

    // Find the updated wheel position
    updated_wheel_pos_.left = prev_wheel_pos_.left + (wheel_vel_.left * unit_per_run);
    updated_wheel_pos_.right = prev_wheel_pos_.right + (wheel_vel_.right * unit_per_run);

    // Format as sensor data (in ticks)
    sensor_data_msg_.left_encoder = updated_wheel_pos_.left * encoder_ticks_per_rad_;
    sensor_data_msg_.right_encoder = updated_wheel_pos_.right * encoder_ticks_per_rad_;
    sensor_data_msg_.stamp = get_clock()->now();

    // Publish on red/sensor_data
    sensor_data_publisher_->publish(sensor_data_msg_);

    // Reset previous wheel positions
    prev_wheel_pos_.left = updated_wheel_pos_.left;
    prev_wheel_pos_.right = updated_wheel_pos_.right;

  }

  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    // Left and right wheel velocity, in "motor command units" (mcu)
    // For the turtlebot, each motor can be command with an integer velocity of between
    // -265 mcu and 265 mcu, and 1 mcu = 0.024 rad/sec


    wheel_vel_.left = static_cast<int>(msg.left_velocity * motor_cmd_per_rad_sec_);
    wheel_vel_.right = static_cast<int>(msg.right_velocity * motor_cmd_per_rad_sec_);
  }

  /// \brief Resets the simulation to initial configuration
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

  /// \brief Moves the robot to a desired pose
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  /// \brief Creates arena walls as a MarkerArray
  void wall_marker()
  {
    for (int i = 0; i < 4; i++) {
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

  /// \brief Creates cylindrical obstacles as a MarkerArray
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

  // Declare member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  visualization_msgs::msg::MarkerArray wall_array_;
  visualization_msgs::msg::MarkerArray obstacle_array_;

  // Publishers
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;

  // Subscribers
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;

  int timestep_;
  int rate_;
  double x0_;
  double y0_;
  double theta0_;
  double x_;
  double y_;
  double theta_;
  double height_;
  double arena_x_length_;
  double arena_y_length_;
  double thickness_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  double obstacles_r_;
  turtlelib::WheelPos wheel_vel_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  turtlelib::WheelPos updated_wheel_pos_;
  turtlelib::WheelPos prev_wheel_pos_{0.0, 0.0};
  nuturtlebot_msgs::msg::SensorData sensor_data_msg_;
  turtlelib::DiffDrive robot_;
  std::vector<double> x_pos_, y_pos_, x_scale_, y_scale_;
  
  

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}