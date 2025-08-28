#include "custom_dwa_planner/dwa_planner.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <limits>

DWAPlanner::DWAPlanner() : Node("dwa_planner"), goal_received_(false)
{
  // Load parameters
  loadParameters();
  
  // Create subscriptions
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&DWAPlanner::odomCallback, this, std::placeholders::_1));
  
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&DWAPlanner::scanCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&DWAPlanner::goalCallback, this, std::placeholders::_1));
  
  // Create publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_trajectories", 10);
  
  // Create control timer
  auto control_period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    control_period, std::bind(&DWAPlanner::controlLoop, this));
  
  RCLCPP_INFO(this->get_logger(), "DWA Planner initialized");
}

void DWAPlanner::loadParameters()
{
  this->declare_parameter("max_linear_vel", 0.22);
  this->declare_parameter("min_linear_vel", -0.22);
  this->declare_parameter("max_angular_vel", 2.84);
  this->declare_parameter("min_angular_vel", -2.84);
  this->declare_parameter("max_linear_accel", 2.5);
  this->declare_parameter("max_angular_accel", 3.2);

  this->declare_parameter("velocity_samples", 20);

  this->declare_parameter("prediction_time", 3.0);
  this->declare_parameter("dt", 0.1);
  this->declare_parameter("goal_weight", 1.0);
  this->declare_parameter("obstacle_weight", 2.0);
  this->declare_parameter("goal_tolerance", 0.2);

  this->declare_parameter("robot_radius", 0.165);
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("max_trajectories_to_show", 15);
  
  max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
  min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
  min_angular_vel_ = this->get_parameter("min_angular_vel").as_double();
  max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
  max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
  
  velocity_samples_ = this->get_parameter("velocity_samples").as_int();
  
  prediction_time_ = this->get_parameter("prediction_time").as_double();
  dt_ = this->get_parameter("dt").as_double();
  goal_weight_ = this->get_parameter("goal_weight").as_double();
  obstacle_weight_ = this->get_parameter("obstacle_weight").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  max_trajectories_to_show_ = this->get_parameter("max_trajectories_to_show").as_int();
}

void DWAPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
  
  current_velocity_.linear = msg->twist.twist.linear.x;
  current_velocity_.angular = msg->twist.twist.angular.z;
}

void DWAPlanner::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  current_scan_ = msg;
}

void DWAPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_pose_.x = msg->pose.position.x;
  goal_pose_.y = msg->pose.position.y;
  goal_pose_.theta = tf2::getYaw(msg->pose.orientation);
  goal_received_ = true;
  
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_pose_.x, goal_pose_.y);
}

void DWAPlanner::controlLoop()
{
  if (!goal_received_ || !current_scan_) {
    return;
  }
  
  if (isGoalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    goal_received_ = false;
    return;
  }
  
  // DWA Algorithm
  std::vector<Velocity> velocity_samples = sampleVelocities();
  std::vector<Trajectory> trajectories;
  
  for (const auto& vel : velocity_samples) {
    Trajectory traj = predictTrajectory(vel, current_pose_);
    if (isTrajectoryValid(traj)) {
      traj.cost = calculateCost(traj);
      trajectories.push_back(traj);
    }
  }
  
  if (trajectories.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid trajectories found! Stopping.");
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }
  
  // Find best trajectory
  auto best_it = std::min_element(trajectories.begin(), trajectories.end(),
    [](const Trajectory& a, const Trajectory& b) { return a.cost < b.cost; });
  
  // Publish velocity command
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = best_it->velocity.linear;
  cmd_vel.angular.z = best_it->velocity.angular;
  cmd_vel_pub_->publish(cmd_vel);

  publishTrajectories(trajectories);
  
  // Debug logging
  logDebugInfo(best_it->velocity, best_it->cost);
}

std::vector<Velocity> DWAPlanner::sampleVelocities()
{
  // Improve by adaptive sampling: fewer samples when close to goal or obstacles.
  std::vector<Velocity> velocities;
  
  // Calculate dynamic window based on current velocity and acceleration limits
  double dt_control = 1.0 / control_frequency_;
  double min_v = std::max(min_linear_vel_, current_velocity_.linear - max_linear_accel_ * dt_control);
  double max_v = std::min(max_linear_vel_, current_velocity_.linear + max_linear_accel_ * dt_control);
  double min_w = std::max(min_angular_vel_, current_velocity_.angular - max_angular_accel_ * dt_control);
  double max_w = std::min(max_angular_vel_, current_velocity_.angular + max_angular_accel_ * dt_control);
  
  // Sample velocities
  double v_step = (max_v - min_v) / velocity_samples_;
  double w_step = (max_w - min_w) / velocity_samples_;
  
  for (int i = 0; i <= velocity_samples_; ++i) {
    for (int j = 0; j <= velocity_samples_; ++j) {
      double v = min_v + i * v_step;
      double w = min_w + j * w_step;
      velocities.emplace_back(v, w);
    }
  }
  
  return velocities;
}

Trajectory DWAPlanner::predictTrajectory(const Velocity& vel, const Pose& start_pose)
{
  Trajectory trajectory;
  trajectory.velocity = vel;
  
  Pose current = start_pose;
  double time = 0.0;
  
  while (time <= prediction_time_) {
    trajectory.poses.push_back(current);
    
    // Simple motion model for differential drive
    current.x += vel.linear * cos(current.theta) * dt_;
    current.y += vel.linear * sin(current.theta) * dt_;
    current.theta += vel.angular * dt_;
    current.theta = normalizeAngle(current.theta);
    
    time += dt_;
  }
  
  return trajectory;
}

double DWAPlanner::calculateCost(const Trajectory& trajectory)
{
  // very steep near obstacles, flat otherwise. => can starve goal progress when obstacles are nearby
  // Try exponential decay or quadratic penalty
  double total_cost = 0.0;
  
  total_cost += goal_weight_ * goalCost(trajectory);
  total_cost += obstacle_weight_ * obstacleCost(trajectory);
  
  return total_cost;
}

double DWAPlanner::goalCost(const Trajectory& trajectory)
{
  if (trajectory.poses.empty()) return std::numeric_limits<double>::max();
  
  const Pose& final_pose = trajectory.poses.back();
  return euclideanDistance(final_pose, goal_pose_);
}

double DWAPlanner::obstacleCost(const Trajectory& trajectory)
{
  double min_distance = std::numeric_limits<double>::max();
  
  for (const auto& pose : trajectory.poses) {
    double distance = getMinObstacleDistance(pose);
    min_distance = std::min(min_distance, distance);
  }
  
  return 0.0;
}

bool DWAPlanner::isTrajectoryValid(const Trajectory& trajectory)
{
  for (const auto& pose : trajectory.poses) {
    if (getMinObstacleDistance(pose) < robot_radius_ + 0.02) {
      return false;
    }
  }
  return true;
}

double DWAPlanner::getMinObstacleDistance(const Pose& pose)
{
  // In this, instead of getting the current pose, get the predicted_pose for future, on the path obstacle
  // 
  if (!current_scan_) return std::numeric_limits<double>::max();
  
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < current_scan_->ranges.size(); ++i) {
    if (current_scan_->ranges[i] < current_scan_->range_min || 
      current_scan_->ranges[i] > current_scan_->range_max) {
      continue;
    }
    
    double angle = current_scan_->angle_min + i * current_scan_->angle_increment;
    double obs_x = current_pose_.x + current_scan_->ranges[i] * cos(current_pose_.theta + angle);
    double obs_y = current_pose_.y + current_scan_->ranges[i] * sin(current_pose_.theta + angle);
    
    double distance = sqrt(pow(pose.x - obs_x, 2) + pow(pose.y - obs_y, 2));
    min_distance = std::min(min_distance, distance);
  }
  
  return min_distance;
}

void DWAPlanner::publishTrajectories(const std::vector<Trajectory>& trajectories)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Sort trajectories by cost and show only the best ones
  std::vector<Trajectory> sorted_trajectories = trajectories;
  std::sort(sorted_trajectories.begin(), sorted_trajectories.end(),
      [](const Trajectory& a, const Trajectory& b) { return a.cost < b.cost; });
  
  int num_to_show = std::min(max_trajectories_to_show_, static_cast<int>(sorted_trajectories.size()));
  
  for (int i = 0; i < num_to_show; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "dwa_trajectories";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    
    // Color based on cost (green for best, red for worst)
    // marker.color.r = static_cast<float>(i) / num_to_show;
    // marker.color.g = 1.0 - static_cast<float>(i) / num_to_show;
    
    marker.color.r = 0.0;
    marker.color.g = 1.0;

    marker.color.b = 0.0;
    marker.color.a = 0.7;
    
    for (const auto& pose : sorted_trajectories[i].poses) {
      geometry_msgs::msg::Point p;
      p.x = pose.x;
      p.y = pose.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(marker);
  }
  
  trajectory_pub_->publish(marker_array);
}

double DWAPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

double DWAPlanner::euclideanDistance(const Pose& p1, const Pose& p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool DWAPlanner::isGoalReached()
{
  return euclideanDistance(current_pose_, goal_pose_) <= goal_tolerance_;
}

void DWAPlanner::logDebugInfo(const Velocity& best_vel, double best_cost)
{
  static int log_counter = 0;
  if (++log_counter % 20 == 0) { // Log every 2 seconds at 10Hz
    RCLCPP_DEBUG(this->get_logger(), 
      "Best velocity: linear=%.2f, angular=%.2f, cost=%.3f, goal_dist=%.2f",
      best_vel.linear, best_vel.angular, best_cost, 
      euclideanDistance(current_pose_, goal_pose_));
  }
}

DWAPlanner::~DWAPlanner()
{

}