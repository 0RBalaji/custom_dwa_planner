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
  this->declare_parameter("obstacle_threshold", 0.5);
  this->declare_parameter("robot_radius", 0.165);
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("publish_trajectories", true);
  this->declare_parameter("max_trajectories_to_show", 10);
  
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

  obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
  
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  publish_trajectories_ = this->get_parameter("publish_trajectories").as_bool();
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
  
  // if (isGoalReached()) {
  //   RCLCPP_INFO(this->get_logger(), "Goal reached!");
  //   geometry_msgs::msg::Twist stop_cmd;
  //   cmd_vel_pub_->publish(stop_cmd);
  //   goal_received_ = false;
  //   return;
  // }
  
  // DWA Algorithm
  std::vector<Velocity> velocity_samples = sampleVelocities();
  std::vector<Trajectory> trajectories;
  
  traj.cost = calculateCost(traj);
  // Publish velocity command - Dummy - For testing
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

std::vector<Velocity> DWAPlanner::sampleVelocities()
{
  RCLCPP_INFO(this->get_logger(), "Sampling velocities");
  return {};
}

Trajectory DWAPlanner::predictTrajectory(const Velocity& vel, const Pose& start_pose)
{
  RCLCPP_INFO(this->get_logger(), "Predicting Trajectorie";
}

double DWAPlanner::calculateCost(const Trajectory& trajectory)
{
  RCLCPP_INFO(this->get_logger(), "Calculating cost");
  return 0.0;
}

DWAPlanner::~DWAPlanner()
{

}