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
///     \param /red/sensor_data (nuturtlebot_msgs::msg::SensorData): Wheel encoder ticks
/// SUBSCRIBES:
///     \param /red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel command velocity
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
#include "nav_msgs/msg/path.hpp"
#include <random>

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
///  \param motor_cmd_max (int): The motor command maximum value
///  \param motor_cmd_per_rad_sec (double): Each motor command unit (mcu) is 0.024 (rad/sec)
///  \param encoder_ticks_per_rad(double): The number of encoder ticks per radian (ticks/rad)
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    // Parameters
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("walls.arena_x_length", 10.0);
    declare_parameter("walls.arena_y_length", 10.0);
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.038);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rad", 651.8986469);
    declare_parameter("draw_only", false);
    declare_parameter("input_noise",0.0);
    declare_parameter("slip_fraction",0.0);

    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    arena_x_length_ = get_parameter("walls.arena_x_length").get_parameter_value().get<double>();
    arena_y_length_ = get_parameter("walls.arena_y_length").get_parameter_value().get<double>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();

    draw_only_= get_parameter("draw_only").get_parameter_value().get<bool>();
    input_noise_=get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction_=get_parameter("slip_fraction").get_parameter_value().get<double>();
    


    // Publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", 10);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path",10);


    // Subscribers
    wheel_cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::Wheel_cmd_callback, this, std::placeholders::_1));

    // Services
    reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::Reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    teleport_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::Teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&Nusim::timer_callback, this));
    //Set initial position
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
    RCLCPP_ERROR(this->get_logger(), "x_ initial = %f", x0_);

    thickness_ = 0.10;
    height_ = 0.25;

    // X coordinates of walls
    x_pos_ = {
      0.0,
      0.0,
      arena_x_length_ / 2.0 + thickness_ / 2.0,
      -arena_x_length_ / 2.0 - thickness_ / 2.0,
    };
    // Y coordinates of walls
    y_pos_ = {
      arena_y_length_ / 2.0 + thickness_ / 2.0,
      -arena_y_length_ / 2.0 - thickness_ / 2.0,
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
    update_wheel_pos();
    update_robot_pos();

    input_noise_dist_ = std::normal_distribution<double>(0.0, std::sqrt(input_noise_));
    slip_dist_=std::uniform_real_distribution<double>(-slip_fraction_,slip_fraction_);
  }

private:
  /// \brief Updates the robot psoition in environmnet
  void update_robot_pos()
  {// ############################ Begin_Citation [4]  #############################
    turtlelib::WheelPos wheel_del;
    wheel_del.left = updated_wheel_pos_.left - prev_wheel_pos_.left;
    wheel_del.right = updated_wheel_pos_.right - prev_wheel_pos_.right;
    robot_.ForwardKinematics(wheel_del);

    // Extract new CONFIGURTION
    x_ = robot_.get_config().x;
    y_ = robot_.get_config().y;
    theta_ = robot_.get_config().theta;
    //update the previous position
    prev_wheel_pos_.left = updated_wheel_pos_.left;
    prev_wheel_pos_.right = updated_wheel_pos_.right;
    // ############################ End_Citation [4]  #############################

   

  }
  /// \brief Updates the wheel positions based on sensor data
  void update_wheel_pos()
  { //add  slip fraction to the wheel position
    // auto slip_noise=slip_dist_(get_random()); not sure if we need same slip for both wheels
    updated_wheel_pos_.left = prev_wheel_pos_.left + (wheel_vel_.left * (1.0 +slip_dist_(get_random()))* (1.0 / rate_));
    updated_wheel_pos_.right = prev_wheel_pos_.right + (wheel_vel_.right * (1.0 +slip_dist_(get_random()))* (1.0 / rate_));

    sensor_data_msg_.left_encoder = updated_wheel_pos_.left * encoder_ticks_per_rad_;
    sensor_data_msg_.right_encoder = updated_wheel_pos_.right * encoder_ticks_per_rad_;
    sensor_data_msg_.stamp = get_clock()->now();
    sensor_data_publisher_->publish(sensor_data_msg_);

  }
  /// \brief Wheel command callbacks
  void Wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    
    wheel_vel_.left = static_cast<double>(msg.left_velocity) * motor_cmd_per_rad_sec_;
    wheel_vel_.right = static_cast<double>(msg.right_velocity) * motor_cmd_per_rad_sec_;
    // auto wheel_noise =input_noise_dist_(get_random()); not sure if we need same noise for both wheels
    //add noise to the wheel commands when wheel commands are not zero
    if (wheel_vel_.left!=0.0 )
    {
      wheel_vel_.left+=input_noise_dist_(get_random());
    }
    if (wheel_vel_.right!=0.0)
    {
      wheel_vel_.right+=input_noise_dist_(get_random());
    }
    

  }

  /// \brief Resets the simulation
  void Reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0.0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
    RCLCPP_ERROR(this->get_logger(), "x_= %f",x0_);
    robot_.set_config({x_,y_,theta_});
  }

  /// \brief Teleport the robot to a desired pose
  void Teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
    robot_.set_config({x_,y_,theta_});
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
      throw std::runtime_error("Lengths of obstacles/x and obstacles/y must be the same.");
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

  /// \brief Main timer callback function
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;
    timestep_publisher_->publish(message);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
    walls_publisher_->publish(wall_array_);
    obstacles_publisher_->publish(obstacle_array_);
    //update the robot position
    update_robot_pos();
    //add red path
    pose.header.stamp=get_clock()->now();
    pose.header.frame_id="nusim/world";
    pose.pose.position.x=x_;
    pose.pose.position.y=y_;
    pose.pose.position.z=0.0;
    pose.pose.orientation.x=q.x();
    pose.pose.orientation.y=q.y();
    pose.pose.orientation.z=q.z();
    pose.pose.orientation.w=q.w();
    red_path.poses.push_back(pose);
    red_path.header.stamp=get_clock()->now();
    red_path.header.frame_id="nusim/world";
    path_publisher_->publish(red_path);
    //update the wheel position
    update_wheel_pos();

  }

  /// \brief Random number generator
  std::mt19937 & get_random()
  {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  visualization_msgs::msg::MarkerArray wall_array_;
  visualization_msgs::msg::MarkerArray obstacle_array_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  geometry_msgs::msg::PoseStamped pose;
  nav_msgs::msg::Path red_path;
  std::normal_distribution<double> input_noise_dist_;
  std::uniform_real_distribution<double> slip_dist_;

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
  bool draw_only_ = false;
  double input_noise_ = 0.0;
  double slip_fraction_ = 0.0;
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
/// \brief Main FUNCTION TO START NODE
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
