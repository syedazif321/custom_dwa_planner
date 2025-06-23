#include "dwa_planner.hpp"
#include <cmath>
#include <tf2/utils.h>

using namespace std::chrono_literals;

DWAPlanner::DWAPlanner()
: Node("dwa_planner"),
  goal_received_(false),
  max_vel_(0.22),
  max_w_(2.84),
  max_acc_(0.2),
  max_ang_acc_(2.0),
  current_linear_vel_(0.0),
  current_angular_vel_(0.0),
  sim_time_(2.0),
  dt_(0.1)
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&DWAPlanner::odomCallback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&DWAPlanner::scanCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&DWAPlanner::goalCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_trajectories", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&DWAPlanner::timerCallback, this));
}

void DWAPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  current_linear_vel_ = msg->twist.twist.linear.x;
  current_angular_vel_ = msg->twist.twist.angular.z;
}

void DWAPlanner::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = *msg;
}

void DWAPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
  goal_ = msg->pose;
  goal_received_ = true;
}

Trajectory DWAPlanner::simulateTrajectory(double v, double w)
{
  Trajectory traj;
  traj.cmd = {v, w};

  double x = current_pose_.position.x;
  double y = current_pose_.position.y;
  double theta = tf2::getYaw(current_pose_.orientation);

  for (double t = 0; t < sim_time_; t += dt_) {
    x += v * cos(theta) * dt_;
    y += v * sin(theta) * dt_;
    theta += w * dt_;

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));
    traj.poses.push_back(pose);
  }

  traj.cost = calculateTrajectoryCost(traj);
  return traj;
}

double DWAPlanner::calculateTrajectoryCost(const Trajectory& traj)
{
  if (traj.poses.size() < 2) return 1e6;

  // Goal distance cost
  const auto& last_pose = traj.poses.back();
  double dx = goal_.position.x - last_pose.position.x;
  double dy = goal_.position.y - last_pose.position.y;
  double goal_cost = std::sqrt(dx * dx + dy * dy);

  // Path length (optional)
  double path_length = 0.0;
  for (size_t i = 1; i < traj.poses.size(); ++i) {
    double dx = traj.poses[i].position.x - traj.poses[i-1].position.x;
    double dy = traj.poses[i].position.y - traj.poses[i-1].position.y;
    path_length += std::sqrt(dx * dx + dy * dy);
  }

  // --- Obstacle avoidance using LaserScan ---
  double obstacle_cost = 0.0;
  double collision_threshold = 0.2;

  for (const auto& pose : traj.poses) {
    double px = pose.position.x;
    double py = pose.position.y;

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
      double range = latest_scan_.ranges[i];

      if (range < latest_scan_.range_min || range > latest_scan_.range_max) continue;

      // Laser point in odom frame (approximate)
      double ox = current_pose_.position.x + range * cos(tf2::getYaw(current_pose_.orientation) + angle);
      double oy = current_pose_.position.y + range * sin(tf2::getYaw(current_pose_.orientation) + angle);

      double dist = std::hypot(px - ox, py - oy);
      if (dist < collision_threshold) {
        obstacle_cost += 1000.0; // Large penalty
        break;  // No need to check more for this pose
      }
    }
  }

  // Velocity penalties
  double short_path_penalty = path_length < 0.15 ? 50.0 : 0.0;
  double low_velocity_penalty = traj.cmd.v < 0.05 ? 10.0 : 0.0;
  double turn_penalty = std::abs(traj.cmd.w) > 0.6 ? 1.0 : 0.0;

  // Total cost
  double total_cost = goal_cost + obstacle_cost + short_path_penalty + low_velocity_penalty + turn_penalty;

  return total_cost;
}


void DWAPlanner::publishTrajectoriesMarkers(const std::vector<Trajectory>& trajs, const VelocitySample& best_cmd)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  for (const auto& traj : trajs) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "trajectories";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;

    if (std::abs(traj.cmd.v - best_cmd.v) < 1e-3 &&
        std::abs(traj.cmd.w - best_cmd.w) < 1e-3) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
    }

    marker.color.a = 0.8;

    for (const auto& pose : traj.poses) {
      geometry_msgs::msg::Point p;
      p.x = pose.position.x;
      p.y = pose.position.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

VelocitySample DWAPlanner::findBestVelocity()
{
  std::vector<Trajectory> all_trajs;
  double best_cost = 1e6;
  VelocitySample best_cmd{0.0, 0.0};

  double min_v = std::max(0.0, current_linear_vel_ - max_acc_ * dt_);
  double max_v = std::min(max_vel_, current_linear_vel_ + max_acc_ * dt_);
  double min_w = std::max(-max_w_, current_angular_vel_ - max_ang_acc_ * dt_);
  double max_w = std::min(max_w_, current_angular_vel_ + max_ang_acc_ * dt_);

  for (double v = min_v; v <= max_v; v += 0.01) {
    for (double w = min_w; w <= max_w; w += 0.1) {
      Trajectory traj = simulateTrajectory(v, w);
      double cost = traj.cost;

      all_trajs.push_back(traj);

      if (cost < best_cost) {
        best_cost = cost;
        best_cmd = {v, w};
      }
    }
  }

  publishTrajectoriesMarkers(all_trajs, best_cmd);
  RCLCPP_INFO(this->get_logger(), "Dynamic Window: v(%.2f to %.2f), w(%.2f to %.2f)", min_v, max_v, min_w, max_w);

  return best_cmd;
}

void DWAPlanner::timerCallback()
{
  if (!goal_received_) {
    RCLCPP_WARN(this->get_logger(), "No goal received.");
    return;
  }

  double dx = goal_.position.x - current_pose_.position.x;
  double dy = goal_.position.y - current_pose_.position.y;
  double distance = std::hypot(dx, dy);

  if (distance < 0.05) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  VelocitySample cmd = findBestVelocity();
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.v;
  twist.angular.z = cmd.w;
  cmd_vel_pub_->publish(twist);
}
