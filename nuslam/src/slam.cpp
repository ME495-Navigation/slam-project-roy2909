/// \file
/// \brief The node is responsible for performing slam on the robot
///
/// PARAMETERS:
///     \param body_id (std::string): body frame of the robot
///     \param odom_id (std::string):  odometry frame(odom)
///     \param wheel_left (std::string): The name of the left wheel joint
///     \param wheel_right (std::string): The name of the right wheel joint
///     \param wheel_radius (double): Radius of wheels (m)
///     \param track_width (double): Distance between wheels (m)
///     \param obstacles.r (double): Radius of obstacles
///
/// PUBLISHES:
///     \param ~/odom (nav_msgs::msg::Odometry): Publishes odometry of robot
///      \param ~/path (nav_msgs::msg::Path): Publishes path of robot
///
/// SUBSCRIBES:
///     \param ~/joint_states (sensor_msgs::msg::JointState): Gets the jpint states of robot
///     \param ~/fake_sensor (visualization_msgs::msg::MarkerArray): Gets the sensor data
///
/// SERVERS:
///     None
///
/// CLIENTS:
///     None
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <armadillo>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

/// \brief The node is responsible for performing slam on the robot
///        It subscribes to the joint states of the robot and the fake sensor data
///
///  \param body_id_ (std::string): body frame of the robot
///  \param odom_id_ (std::string): odometry frame(odom)
///  \param wheel_left_ (std::string): The name of the left wheel joint
///  \param wheel_right_ (std::string): The name of the right wheel joint
///  \param wheel_radius_ (double): Radius of wheels (m)
///  \param track_width_ (double): Distance between wheels (m)
///  \param obs_num (int): Number of obstacles


class slam : public rclcpp::Node
{
public:
  slam()
  : Node("slam")
  { // body_id
    declare_parameter("body_id", "green/base_footprint");
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    // odom_id
    declare_parameter("odom_id", "green/odom");
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();

    declare_parameter("wheel_left", "wheel_left_joint");
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    declare_parameter("wheel_right", "wheel_right_joint");
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    declare_parameter("wheel_radius", 0.033);
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();

    declare_parameter("track_width", 0.16);
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    declare_parameter("obstacles.r", 0.038);
    obstacle_radius_ = get_parameter("obstacles.r").get_parameter_value().get<double>();


    if (body_id_ == "" || odom_id_ == "" || wheel_left_ == "" ||
      wheel_right_ == "")
    {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined ");
      throw std::runtime_error("Parameters not defined!");
    }

    if (track_width_ == -1.0 || wheel_radius_ == -1.0) {
      RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "Parameters not defined ");
      throw std::runtime_error("Parameters not defined!");
    }

    robot_ = turtlelib::DiffDrive{wheel_radius_, track_width_};

    intialized_obstacles_ = std::vector<bool>(obs_num, false);
    system_state_ = arma::vec(3 + 2 * obs_num, arma::fill::zeros);
    system_state_prev_ = arma::vec(3 + 2 * obs_num, arma::fill::zeros);
    system_covariance_ = arma::mat(3 + 2 * obs_num, 3 + 2 * obs_num, arma::fill::zeros);

    for (int i = 0; i < 2 * obs_num; i++) {
      system_covariance_.at(i + 3, i + 3) = 999999;
    }

    Q_mat = arma::mat(3, 3, arma::fill::zeros);
    Q_mat.diag() += 1e-3;
    RCLCPP_INFO_STREAM(get_logger(), "Q:\n" << Q_mat);
    R = arma::mat(2 * obs_num, 2 * obs_num, arma::fill::zeros);
    R.diag() += 1e-1;
    Q_bar = arma::mat(3 + 2 * obs_num, 3 + 2 * obs_num, arma::fill::zeros);
    Q_bar.submat(0, 0, 2, 2) = Q_mat;
    I = arma::mat(3 + 2 * obs_num, 3 + 2 * obs_num, arma::fill::eye);
    T_map_odom_.header.frame_id = "map";


    // Publishers
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("green/odom", 10);
    green_path_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_map_odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Subscribers
    joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &slam::odometry_callback,
        this, _1));

    sensor_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10, std::bind(
        &slam::sensor_callback,
        this, _1));
    landmark_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/landmarks", 10);
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    T_map_odom_.header.frame_id = "map";
    T_map_odom_.child_frame_id = odom_id_;


  }

private:
  /// \brief Publishes the odometry of robot
  void odometry_callback(const sensor_msgs::msg::JointState & msg)
  {
    //update odometry
    odom_.header.stamp = get_clock()->now();
    new_wheel_.left = msg.position.at(0) - prev_wheel_.left;
    new_wheel_.right = msg.position.at(1) - prev_wheel_.right;
    prev_wheel_.left = msg.position.at(0);
    prev_wheel_.right = msg.position.at(1);
    robot_.ForwardKinematics(new_wheel_);
    Q = robot_.get_config();
    twistb_ = robot_.BodyTwist(new_wheel_);
    q_.setRPY(0, 0, Q.theta);
    odom_.pose.pose.position.x = Q.x;
    odom_.pose.pose.position.y = Q.y;
    odom_.pose.pose.orientation.x = q_.x();
    odom_.pose.pose.orientation.y = q_.y();
    odom_.pose.pose.orientation.z = q_.z();
    odom_.pose.pose.orientation.w = q_.w();

    odom_.twist.twist.linear.x = twistb_.x;
    odom_.twist.twist.linear.y = twistb_.y;
    odom_.twist.twist.angular.z = twistb_.omega;


    geometry_msgs::msg::TransformStamped t;
    //create transform
    t.header.stamp = get_clock()->now();
    t.header.frame_id = odom_id_;
    t.child_frame_id = body_id_;
    t.transform.translation.x = Q.x;
    t.transform.translation.y = Q.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q_.x();
    t.transform.rotation.y = q_.y();
    t.transform.rotation.z = q_.z();
    t.transform.rotation.w = q_.w();


    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    odometry_publisher_->publish(odom_);


  }
  /// @brief sensor callback, responsible for updating the state of the robot and performing slam
  /// @param msg   The sensor data
  void sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    turtlelib::RobotConfig q_i = robot_.get_config();
    T_ob_ = {{q_i.x, q_i.y}, q_i.theta};
    T_mr_ = T_mo_ * T_ob_;
    //update system state
    system_state_.at(0) = turtlelib::normalize_angle(T_mr_.rotation());
    system_state_.at(1) = T_mr_.translation().x;
    system_state_.at(2) = T_mr_.translation().y;

    const auto dx = system_state_.at(1) - system_state_prev_.at(1);
    const auto dy = system_state_.at(2) - system_state_prev_.at(2);
    arma::mat A_t = arma::mat(3 + 2 * obs_num, 3 + 2 * obs_num, arma::fill::zeros);
    A_t.at(1, 0) = -dy;
    A_t.at(2, 0) = dx;
    A_t.diag() += 1.0;
    system_covariance_ = A_t * system_covariance_ * A_t.t() + Q_bar;

    for (int i = 0; i < static_cast<double>(msg->markers.size()); i++) {
      const auto m_x = msg->markers.at(i).pose.position.x;
      const auto m_y = msg->markers.at(i).pose.position.y;
      const auto m_id = msg->markers.at(i).id;
      const auto action = msg->markers.at(i).action;
      if (action == 2) {
        continue;
      }

      const auto d_x = m_x;
      const auto d_y = m_y;
      const auto d = d_x * d_x + d_y * d_y;
      const auto rj = std::sqrt(d);
      const auto phij = turtlelib::normalize_angle(std::atan2(d_y, d_x));
      arma::vec z_j(2, arma::fill::zeros);
      z_j.at(0) = rj;
      z_j.at(1) = phij;

      if (!intialized_obstacles_.at(m_id)) {
        intialized_obstacles_.at(m_id) = true;
        system_state_.at(3 + 2 * m_id) = system_state_.at(1) + rj *
          std::cos(
          turtlelib::normalize_angle(
            system_state_.at(
              0) + phij));
        system_state_.at(4 + 2 * m_id) = system_state_.at(2) + rj *
          std::sin(
          turtlelib::normalize_angle(
            system_state_.at(
              0) + phij));
      }
      const auto d_xhat = system_state_.at(3 + 2 * m_id) - system_state_.at(1);
      const auto d_yhat = system_state_.at(4 + 2 * m_id) - system_state_.at(2);
      const auto d_hat = d_xhat * d_xhat + d_yhat * d_yhat;
      const auto r_jhat = std::sqrt(d_hat);
      const auto phijhat =
        turtlelib::normalize_angle(std::atan2(d_yhat, d_xhat)) - system_state_.at(0);
      arma::vec z_jhat(2, arma::fill::zeros);
      z_jhat.at(0) = r_jhat;
      z_jhat.at(1) = phijhat;
      arma::mat H_j = arma::mat(2, 3 + 2 * obs_num, arma::fill::zeros);
      H_j.at(1, 0) = -1.0;
      H_j.at(0, 1) = -d_xhat / r_jhat;
      H_j.at(1, 1) = d_yhat / d_hat;
      H_j.at(0, 2) = -d_yhat / r_jhat;
      H_j.at(1, 2) = -d_xhat / d_hat;
      H_j.at(0, 3 + 2 * m_id) = d_xhat / r_jhat;
      H_j.at(1, 3 + 2 * m_id) = -d_yhat / d_hat;
      H_j.at(0, 4 + 2 * m_id) = d_yhat / r_jhat;
      H_j.at(1, 4 + 2 * m_id) = d_xhat / d_hat;
      arma::mat Ri = R.submat(2 * m_id, 2 * m_id, 2 * m_id + 1, 2 * m_id + 1);
      arma::mat K_st = H_j * system_covariance_ * H_j.t() + Ri;
      arma::mat K = system_covariance_ * H_j.t() * K_st.i();
      arma::vec z_diff = z_j - z_jhat;
      z_diff.at(1) = turtlelib::normalize_angle(z_diff.at(1));
      system_state_ = system_state_ + K * z_diff;
      system_state_.at(0) = turtlelib::normalize_angle(system_state_.at(0));
      system_covariance_ = (I - K * H_j) * system_covariance_;
    }

    T_map_odom_.header.stamp = get_clock()->now();
    T_ob_ = {{Q.x, Q.y}, Q.theta};
    T_mr_ =
      turtlelib::Transform2D{turtlelib::Vector2D{system_state_.at(1), system_state_.at(2)},
      system_state_.at(0)};
    T_mo_ = T_mr_ * T_ob_.inv();
    T_map_odom_.transform.translation.x = T_mo_.translation().x;
    T_map_odom_.transform.translation.y = T_mo_.translation().y;
    T_map_odom_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, turtlelib::normalize_angle(T_mo_.rotation()));
    T_map_odom_.transform.rotation.x = q.x();
    T_map_odom_.transform.rotation.y = q.y();
    T_map_odom_.transform.rotation.z = q.z();
    T_map_odom_.transform.rotation.w = q.w();
    tf_map_odom_->sendTransform(T_map_odom_);
    green_pose_.header.frame_id = "map";
    green_pose_.header.stamp = get_clock()->now();
    green_pose_.pose.position.x = system_state_.at(1);
    green_pose_.pose.position.y = system_state_.at(2);
    green_pose_.pose.position.z = 0.0;
    tf2::Quaternion q_robot_;
    //adding path to robot
    green_pose_.pose.orientation.x = q_robot_.x();
    green_pose_.pose.orientation.y = q_robot_.y();
    green_pose_.pose.orientation.z = q_robot_.z();
    green_pose_.pose.orientation.w = q_robot_.w();
    green_path_.poses.push_back(green_pose_);
    if (green_path_.poses.size() > 100) {
      green_path_.poses.erase(green_path_.poses.begin());
    }
    green_path_.header.stamp = get_clock()->now();
    green_path_.header.frame_id = "nusim/world";
    green_path_publisher_->publish(green_path_);
    // creating markers for landmarks(measured)
    visualization_msgs::msg::MarkerArray landmarks;
    for (int i = 0; i < obs_num; i++) {
      if (intialized_obstacles_.at(i)) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = get_clock()->now();
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = system_state_.at(3 + 2 * i);
        marker.pose.position.y = system_state_.at(4 + 2 * i);
        marker.pose.position.z = 0.0;
        marker.scale.x = 2 * obstacle_radius_;
        marker.scale.y = 2 * obstacle_radius_;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        landmarks.markers.push_back(marker);
      }
    }
    landmark_publisher_->publish(landmarks);

    system_state_prev_ = system_state_;

  }


  // Variables
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double wheel_radius_;
  double track_width_;
  int obs_num = 3;
  turtlelib::DiffDrive robot_;
  turtlelib::WheelPos prev_wheel_{0.0, 0.0};
  turtlelib::WheelPos new_wheel_;
  turtlelib::Twist2D twistb_;
  nav_msgs::msg::Odometry odom_;
  tf2::Quaternion q_;
  turtlelib::RobotConfig Q;
  nav_msgs::msg::Path green_path_;
  geometry_msgs::msg::PoseStamped green_pose_;
  turtlelib::Transform2D T_map_robot_;
  arma::vec system_state_prev_;
  arma::vec system_state_;
  arma::mat system_covariance_;
  arma::mat Q_mat;
  arma::mat R;
  arma::mat Q_bar;
  arma::mat I;
  std::vector<bool> intialized_obstacles_;
  geometry_msgs::msg::TransformStamped T_odom_body_, T_map_odom_;
  turtlelib::Transform2D T_ob_, T_mr_, T_mo_;
  double obstacle_radius_;


  //Pubslishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_publisher_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_map_odom_;

};
/// \brief main funtion of node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam>());
  rclcpp::shutdown();
  return 0;
}
