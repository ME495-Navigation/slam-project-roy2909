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
///     \param motor_cmd_per_rad_sec (double): Each motor command unit (mcu) is 0.024 (rad/sec)
///     \param encoder_ticks_per_rad(double): The number of encoder ticks per radian (ticks/rad)
///     \param draw_only (bool): If true, only draws the environment without simulating the robot
///     \param input_noise (double): The noise added to the wheel commands
///     \param slip_fraction (double): The fraction of slip added to the wheel commands
///     \param max_range (double): The maximum range of the sensor
///     \param basic_sensor_variance (double): The basic sensor variance


/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): current timestep of the simulation
///     \param ~/walls (visualization_msgs::msg::MarkerArray): MarkerArray of Walls in Rviz2
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): MarkerArray of cylindrial obstacles in Rviz2
///     \param /red/sensor_data (nuturtlebot_msgs::msg::SensorData): Wheel encoder ticks
///     \param /red/path (nav_msgs::msg::Path): The path of the robot
///     \param /fake_sensor (visualization_msgs::msg::MarkerArray): The fake sensor data
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
#include "sensor_msgs/msg/laser_scan.hpp"

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
///  \param draw_only_ (bool): If true, only draws the environment without simulating the robot
///  \param input_noise_ (double): The noise added to the wheel commands
///  \param slip_fraction_ (double): The fraction of slip added to the wheel commands
///  \param max_range_ (double): The maximum range of the sensor
///  \param basic_sensor_variance_ (double): The basic sensor variance
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
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("max_range", 2.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("collision_radius", 0.10);
    declare_parameter("range_min", 0.11999999731779099);
    declare_parameter("range_max", 3.5);
    declare_parameter("num_samples", 360);
    declare_parameter("angle_increment", 0.01745329238474369);
    declare_parameter("angle_min", 0.0);
    declare_parameter("angle_max", 6.2657318115234375);
    declare_parameter("time_increment", 0.0005574136157520115);
    declare_parameter("scan_time", 0.2066);


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

    draw_only_ = get_parameter("draw_only").get_parameter_value().get<bool>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction_ = get_parameter("slip_fraction").get_parameter_value().get<double>();
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();
    basic_sensor_variance_ =
      get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();
    range_min_ = get_parameter("range_min").get_parameter_value().get<double>();
    range_max_ = get_parameter("range_max").get_parameter_value().get<double>();
    num_samples_ = get_parameter("num_samples").get_parameter_value().get<int>();
    angle_increment_ = get_parameter("angle_increment").get_parameter_value().get<double>();
    angle_min_ = get_parameter("angle_min").get_parameter_value().get<double>();
    angle_max_ = get_parameter("angle_max").get_parameter_value().get<double>();
    time_increment_ = get_parameter("time_increment").get_parameter_value().get<double>();
    scan_time_ = get_parameter("scan_time").get_parameter_value().get<double>();


    // Publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    walls_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    obstacles_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", 10);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    fake_sensor_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor",
      10);
    laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);


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
    //fake sensor timer
    timer2_ = create_wall_timer(
      0.2s,
      std::bind(&Nusim::timer2_callback, this));
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
    slip_dist_ = std::uniform_real_distribution<double>(-slip_fraction_, slip_fraction_);
    input_sensor_dist_ = std::normal_distribution<double>(0.0, std::sqrt(basic_sensor_variance_));
    // Lidar noise
    lidar_noise_dist_ = std::normal_distribution<double>(0.0, std::sqrt(basic_sensor_variance_));

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
    updated_wheel_pos_.left = prev_wheel_pos_.left +
      (wheel_vel_.left * (1.0 + slip_dist_(get_random())) * (1.0 / rate_));
    updated_wheel_pos_.right = prev_wheel_pos_.right +
      (wheel_vel_.right * (1.0 + slip_dist_(get_random())) * (1.0 / rate_));

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
    if (wheel_vel_.left != 0.0) {

      wheel_vel_.left += input_noise_dist_(get_random());
    }
    if (wheel_vel_.right != 0.0) {
      wheel_vel_.right += input_noise_dist_(get_random());
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
    RCLCPP_ERROR(this->get_logger(), "x_= %f", x0_);
    robot_.set_config({x_, y_, theta_});
  }

  /// \brief Teleport the robot to a desired pose
  void Teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
    robot_.set_config({x_, y_, theta_});
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

  /// \brief Creates fake sensor as a MarkerArray
  void fake_sensor_marker()
  {
    sensor_array_.markers.clear(); // Clear the MarkerArray


    const auto sensor_random = input_sensor_dist_(get_random());
    for (size_t i = 0; i < obstacles_x_.size(); ++i) {
      visualization_msgs::msg::Marker sensor;
      sensor.header.frame_id = "red/base_footprint";
      sensor.header.stamp = get_clock()->now();
      sensor.id = i;
      sensor.ns = "fake_sensor";
      sensor.type = visualization_msgs::msg::Marker::CYLINDER;
      sensor.scale.x = 2.0 * obstacles_r_;
      sensor.scale.y = 2.0 * obstacles_r_;
      sensor.scale.z = 0.25;
      sensor.color.r = 1.0;
      sensor.color.g = 1.0;
      sensor.color.b = 0.0;
      sensor.color.a = 1.0;
      const turtlelib::Transform2D world_to_robot{turtlelib::Vector2D{x_, y_}, theta_};
      const auto robot_to_world = world_to_robot.inv();
      turtlelib::Point2D obstcles_position{obstacles_x_.at(i) + sensor_random,
        obstacles_y_.at(i) + sensor_random};
      const auto obstcles_position_in_robot = robot_to_world(obstcles_position);
      sensor.pose.position.x = obstcles_position_in_robot.x;
      sensor.pose.position.y = obstcles_position_in_robot.y;
      sensor.pose.position.z = 0.0;
      auto distance =
        std::sqrt(
        std::pow(
          obstcles_position_in_robot.x,
          2) + std::pow(obstcles_position_in_robot.y, 2));
      if (distance <= max_range_) {
        sensor.action = visualization_msgs::msg::Marker::ADD;

      } else {
        sensor.action = visualization_msgs::msg::Marker::DELETE;


      }
      sensor_array_.markers.push_back(sensor);

    }

    fake_sensor_publisher_->publish(sensor_array_);


  }
  /// \brief Calculates the distance between two points
  double calculateDistance(const turtlelib::Point2D & p1, const turtlelib::Point2D & p2)
  {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
  }
/// ### Begin_Citation [5]  ###
///  Used https://mathworld.wolfram.com/Circle-LineIntersection.html
/// @brief Finds the distance to the intersection point of a line segment and a circle
/// @param lineStart start point of the line segment
/// @param lineEnd end point of the line segment
/// @param circleCenter center of the circle
/// @param radius radius of the circle
/// @param lidar_angle  angle of the Lidar beam
/// @param robot_angle angle of the robot
/// @return Intersection distance
  double findIntersectionDistance(
    const turtlelib::Point2D & lineStart, const turtlelib::Point2D & lineEnd,
    const turtlelib::Point2D & circleCenter, double radius,
    double lidar_angle, double robot_angle)
  {
    // Translate coordinates
    turtlelib::Point2D translatedLineStart =
    {lineStart.x - circleCenter.x, lineStart.y - circleCenter.y};
    turtlelib::Point2D translatedLineEnd = {lineEnd.x - circleCenter.x, lineEnd.y - circleCenter.y};

    // Define the line segment
    double dx = translatedLineEnd.x - translatedLineStart.x;
    double dy = translatedLineEnd.y - translatedLineStart.y;
    double dr = std::sqrt(dx * dx + dy * dy);
    double D = translatedLineStart.x * translatedLineEnd.y - translatedLineEnd.x *
      translatedLineStart.y;

    // Calculate discriminant
    double disc = radius * radius * dr * dr - D * D;

    // No intersection
    if (disc < 0) {
      return -1.0;
    }

    // Determines intersection point to use
    const auto signDy = (dy < 0) ? -1 : 1;

    // Calculate intersection points
    turtlelib::Point2D intersection1 = {
      (D * dy + signDy * dx * std::sqrt(disc)) / (dr * dr),
      (-D * dx + std::abs(dy) * std::sqrt(disc)) / (dr * dr)
    };

    turtlelib::Point2D intersection2 = {
      (D * dy - signDy * dx * std::sqrt(disc)) / (dr * dr),
      (-D * dx - std::abs(dy) * std::sqrt(disc)) / (dr * dr)
    };

    intersection1.x += circleCenter.x;
    intersection1.y += circleCenter.y;

    intersection2.x += circleCenter.x;
    intersection2.y += circleCenter.y;

    //  Lidar beam in world coordinates
    double lidar_x = std::cos(lidar_angle + robot_angle);
    double lidar_y = std::sin(lidar_angle + robot_angle);

    // Calculate dot products to determine whether the intersection points are in the direction of the Lidar beam
    double dot1 = (intersection1.x - lineStart.x) * lidar_x + (intersection1.y - lineStart.y) *
      lidar_y;
    double dot2 = (intersection2.x - lineStart.x) * lidar_x + (intersection2.y - lineStart.y) *
      lidar_y;

    // start to intersection distance
    double distance1 = calculateDistance(lineStart, intersection1);
    double distance2 = calculateDistance(lineStart, intersection2);

    // Determine the minimum distance
    if (dot1 > 0.0 && (dot2 <= 0.0 || distance1 < distance2)) {
      return distance1;
    } else if (dot2 > 0.0 && (dot1 <= 0.0 || distance2 < distance1)) {
      return distance2;
    }

    return -1.0;
  }

/// \brief Creates a laser scan
  void lidar_scan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = get_clock()->now();
    scan.header.frame_id = "red/base_scan";
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = time_increment_;
    scan.scan_time = scan_time_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.resize(num_samples_);

    for (size_t sampleIndex = 0; sampleIndex < num_samples_; sampleIndex++) {
      // Compute current angle
      double currentAngle = angle_min_ + sampleIndex * angle_increment_ + theta_;

      // Initialize minimum distance
      double minimumDistance = range_max_;

      // Wall Collision Detection
      double xComponent = cos(currentAngle);
      double yComponent = sin(currentAngle);

      // distances to each wall
      std::vector<double> wallDistances;

      // Calculate distances to top and bottom walls
      if (yComponent != 0) {
        double topWallDistance = (arena_x_length_ / 2 - y_) / yComponent;
        double bottomWallDistance = (-arena_y_length_ / 2 - y_) / yComponent;

        // Add valid distances to the vector
        if (topWallDistance > 0) {wallDistances.push_back(topWallDistance);}
        if (bottomWallDistance > 0) {wallDistances.push_back(bottomWallDistance);}
      }

      // Calculate distances to the right and left walls
      if (xComponent != 0) {
        double rightWallDistance = (arena_x_length_ / 2 - x_) / xComponent;
        double leftWallDistance = (-arena_x_length_ / 2 - x_) / xComponent;

        // Add valid distances to the vector
        if (rightWallDistance > 0) {wallDistances.push_back(rightWallDistance);}
        if (leftWallDistance > 0) {wallDistances.push_back(leftWallDistance);}
      }

      // Find the minimum distance to a wall
      if (!wallDistances.empty()) {
        double minWallDistance = std::numeric_limits<double>::max();
        for (auto distance : wallDistances) {
          minWallDistance = std::min(minWallDistance, distance);
        }
        minimumDistance = minWallDistance;
      }

      // Check for intersection with obstacles
      for (size_t obstacleIndex = 0; obstacleIndex < obstacles_x_.size(); obstacleIndex++) {
        turtlelib::Point2D lineStart = {x_, y_};
        turtlelib::Point2D lineEnd = {x_ + range_max_ * std::cos(currentAngle),
          y_ + range_max_ * std::sin(currentAngle)};
        turtlelib::Point2D circleCenter =
        {obstacles_x_[obstacleIndex], obstacles_y_[obstacleIndex]};

        double obstacleDistance = findIntersectionDistance(
          lineStart, lineEnd, circleCenter, obstacles_r_,
          angle_min_ + sampleIndex * angle_increment_, theta_);

        if (obstacleDistance < minimumDistance && obstacleDistance >= range_min_) {
          minimumDistance = obstacleDistance;
        }
      }

      // Update Lidar message
      if (minimumDistance < range_max_) {
        scan.ranges[sampleIndex] = minimumDistance + lidar_noise_dist_(get_random());
      }
    }

// Publish Lidar scan
    laser_scan_publisher_->publish(scan);
  }


  /// \brief collison detection
  void collision_detection()
  {
    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      auto distance =
        std::sqrt(std::pow(obstacles_x_.at(i) - x_, 2) + std::pow(obstacles_y_.at(i) - y_, 2));
      if (distance <= collision_radius_ + obstacles_r_) {
        // Collision detected
        // Compute the line between the robot center and the obstacle center
        auto line_dist{
          turtlelib::Vector2D{x_, y_} -
          turtlelib::Vector2D{obstacles_x_.at(i), obstacles_y_.at(i)}
        };
        // Normalize the direction
        auto norm = turtlelib::normalize_vector(line_dist);

        // Move the robot's center along this line so that the collision circles are tangent
        x_ = obstacles_x_.at(i) + norm.x * (collision_radius_ + obstacles_r_);
        y_ = obstacles_y_.at(i) + norm.y * (collision_radius_ + obstacles_r_);
        robot_.set_config({x_, y_, theta_});

      }
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
    collision_detection();
    update_robot_pos();
    //add red path
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    red_path.poses.push_back(pose);
    red_path.header.stamp = get_clock()->now();
    red_path.header.frame_id = "nusim/world";
    path_publisher_->publish(red_path);
    //update the wheel position
    update_wheel_pos();

  }
  /// \brief Fake sensor timer callback function
  void timer2_callback()
  {
    fake_sensor_marker();
    lidar_scan();
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
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  visualization_msgs::msg::MarkerArray wall_array_;
  visualization_msgs::msg::MarkerArray obstacle_array_;
  visualization_msgs::msg::MarkerArray sensor_array_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  geometry_msgs::msg::PoseStamped pose;
  nav_msgs::msg::Path red_path;
  std::normal_distribution<double> input_noise_dist_, input_sensor_dist_;
  std::uniform_real_distribution<double> slip_dist_;
  std::normal_distribution<double> lidar_noise_dist_;


  int timestep_;
  int rate_;
  double x0_;
  double y0_;
  double theta0_, basic_sensor_variance_;
  double x_, collision_radius_;
  double y_;
  double range_min_ = 0.0;
  double range_max_ = 0.0;
  double theta_;
  double angle_increment_;
  double height_;
  double arena_x_length_;
  double arena_y_length_;
  double thickness_, max_range_;
  bool draw_only_ = false;
  double input_noise_ = 0.0;
  double slip_fraction_ = 0.0;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  double obstacles_r_;
  double num_samples_;
  turtlelib::WheelPos wheel_vel_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  turtlelib::WheelPos updated_wheel_pos_;
  turtlelib::WheelPos prev_wheel_pos_{0.0, 0.0};
  nuturtlebot_msgs::msg::SensorData sensor_data_msg_;
  turtlelib::DiffDrive robot_;
  std::vector<double> x_pos_, y_pos_, x_scale_, y_scale_;
  double angle_min_, angle_max_, time_increment_, scan_time_;


};

/// \brief Main FUNCTION TO START NODE
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
