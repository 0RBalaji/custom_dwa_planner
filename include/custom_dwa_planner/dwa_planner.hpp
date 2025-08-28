#ifndef CUSTOM_DWA_PLANNER_HPP_
#define CUSTOM_DWA_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <memory>

struct Velocity
{
  double linear;
  double angular;
  
  Velocity(double l = 0.0, double a = 0.0) : linear(l), angular(a) {}
};

struct Pose
{
  double x, y, theta;
  
  Pose(double x = 0.0, double y = 0.0, double theta = 0.0) : x(x), y(y), theta(theta) {}
};

struct Trajectory
{
  std::vector<Pose> poses;
  Velocity velocity;
  double cost;
  
  Trajectory() : cost(std::numeric_limits<double>::max()) {}
};


class DWAPlanner : public rclcpp::Node
{
public:

  DWAPlanner();

  ~DWAPlanner();
  
private:
  // ROS2 Communication
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
    
  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // State Variables
  Pose current_pose_;
  Velocity current_velocity_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  Pose goal_pose_;

  // Pose has only x, y, z and Quaternions -> for internal computation
  // PoseStamped has Pose + Header (frame_id + timestamp) -> For publishing data format

  bool goal_received_;
  
  // Parameters
  double max_linear_vel_, min_linear_vel_;
  double max_angular_vel_, min_angular_vel_;
  double max_linear_accel_, max_angular_accel_;
  int velocity_samples_;
  double prediction_time_, dt_;
  double goal_weight_, obstacle_weight_, velocity_weight_, smoothness_weight_;
  double goal_tolerance_, obstacle_threshold_, robot_radius_;
  double control_frequency_;
  bool publish_trajectories_;
  int max_trajectories_to_show_;
  
  // Methods
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void controlLoop();

  // Core
  std::vector<Velocity> sampleVelocities();
  Trajectory predictTrajectory(const Velocity& vel, const Pose& start_pose);
  double calculateCost(const Trajectory& trajectory);
  double getMinObstacleDistance(const Pose& pose);
};

#endif  // CUSTOM_DWA_PLANNER__DWA_PLANNER_HPP_