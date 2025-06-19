#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <random>
#include <functional>

using std::placeholders::_1;

class DWAPlanner : public rclcpp::Node {
  public:
    DWAPlanner() : Node("dwa_planner_node"), gen_(rd_()) {
      loadParams();
  
      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&DWAPlanner::goalCallback, this, _1));
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&DWAPlanner::odomCallback, this, _1));
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&DWAPlanner::scanCallback, this, _1));
  
      timer_ = create_wall_timer(
        std::chrono::milliseconds(int(step_time_ * 1000)),
        std::bind(&DWAPlanner::planLoop, this)
      );
  
      RCLCPP_INFO(get_logger(), "DWA Planner node initialized.");
    }
  
  private:
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
  
    // Internal state
    nav_msgs::msg::Odometry::SharedPtr odom_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_;
    bool goal_received_ = false;
  
    // RNG
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> speed_dist_, turn_dist_;
  
    // Parameters
    double max_speed_, max_turn_, step_time_, sim_steps_;
    int num_paths_;
    double goal_tolerance_, obstacle_clearance_;
  
    void loadParams() {
      declare_parameter("max_speed", 0.22);
      declare_parameter("max_turn", 2.84);
      declare_parameter("step_time", 0.1);
      declare_parameter("sim_steps", 30.0);
      declare_parameter("num_paths", 3000);
      declare_parameter("goal_tolerance", 0.2);
      declare_parameter("obstacle_clearance", 0.3);
  
      get_parameter("max_speed", max_speed_);
      get_parameter("max_turn", max_turn_);
      get_parameter("step_time", step_time_);
      get_parameter("sim_steps", sim_steps_);
      get_parameter("num_paths", num_paths_);
      get_parameter("goal_tolerance", goal_tolerance_);
      get_parameter("obstacle_clearance", obstacle_clearance_);
  
      speed_dist_ = std::uniform_real_distribution<double>(0.0, max_speed_);
      turn_dist_ = std::uniform_real_distribution<double>(-max_turn_, max_turn_);
  
      RCLCPP_INFO(get_logger(), "Parameters loaded:");
      RCLCPP_INFO(get_logger(), "  max_speed: %.2f", max_speed_);
      RCLCPP_INFO(get_logger(), "  max_turn: %.2f", max_turn_);
      RCLCPP_INFO(get_logger(), "  step_time: %.2f", step_time_);
      RCLCPP_INFO(get_logger(), "  sim_steps: %.2f", sim_steps_);
      RCLCPP_INFO(get_logger(), "  num_paths: %d", num_paths_);
      RCLCPP_INFO(get_logger(), "  goal_tolerance: %.2f", goal_tolerance_);
      RCLCPP_INFO(get_logger(), "  obstacle_clearance: %.2f", obstacle_clearance_);
    }
  
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      goal_ = msg;
      goal_received_ = true;
    }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; }
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = msg; }
  
    void planLoop() {
    if (!goal_received_ || !odom_ || !scan_) return;
  
      auto [vx, vw] = chooseBestPath();
  
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = vx;
      cmd.angular.z = vw;
      cmd_pub_->publish(cmd);
  
      RCLCPP_DEBUG(get_logger(), "Published cmd_vel: linear=%.2f, angular=%.2f", vx, vw);
    }
  
    std::pair<double, double> chooseBestPath() {
      double best_score = -1e9;
      std::pair<double, double> best = {0.0, 0.0};
  
      double ox = odom_->pose.pose.position.x;
      double oy = odom_->pose.pose.position.y;
      tf2::Quaternion q(
        odom_->pose.pose.orientation.x,
        odom_->pose.pose.orientation.y,
        odom_->pose.pose.orientation.z,
        odom_->pose.pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
      double gx = goal_->pose.position.x;
      double gy = goal_->pose.position.y;
  
      double dist_to_goal = std::hypot(gx - ox, gy - oy);
      RCLCPP_DEBUG(get_logger(), "Distance to goal: %.2f", dist_to_goal);
  
      if (dist_to_goal < goal_tolerance_) {
        RCLCPP_INFO(get_logger(), "Goal reached. Distance: %.2f", dist_to_goal);
        goal_received_ = false;
        return best;
      }
  
      for (int i = 0; i < num_paths_; ++i) {
        double v = speed_dist_(gen_);
        double w = turn_dist_(gen_);
        auto path = predictTrajectory(ox, oy, yaw, v, w);
        if (isCollision(path)) continue;
  
        auto &last = path.back();
        double goal_dist = -std::hypot(last.first - gx, last.second - gy);
        if (goal_dist > best_score) {
          best_score = goal_dist;
          best = {v, w};
        }
      }
  
      return best;
    }
  
    std::vector<std::pair<double, double>> predictTrajectory(double x, double y, double yaw, double v, double w) {
      std::vector<std::pair<double, double>> path;
      for (int i = 0; i < sim_steps_; ++i) {
        yaw += w * step_time_;
        x += v * std::cos(yaw) * step_time_;
        y += v * std::sin(yaw) * step_time_;
        path.emplace_back(x, y);
      }
      return path;
    }
  
    bool isCollision(const std::vector<std::pair<double, double>>& path) {
      const double angle_min = scan_->angle_min;
      const double angle_increment = scan_->angle_increment;
      const auto& ranges = scan_->ranges;
  
      double robot_x = odom_->pose.pose.position.x;
      double robot_y = odom_->pose.pose.position.y;
  
      for (const auto& pt : path) {
        double dx = pt.first - robot_x;
        double dy = pt.second - robot_y;
        double r = std::hypot(dx, dy);
        double theta = std::atan2(dy, dx);
  
      // convert angle to scan index
        int index = static_cast<int>((theta - angle_min) / angle_increment);
        if (index >= 0 && index < static_cast<int>(ranges.size())) {
          double obs_range = ranges[index];
          if (!std::isinf(obs_range) && r < obs_range - obstacle_clearance_) {
            RCLCPP_DEBUG(get_logger(), "Collision detected at (%.2f, %.2f), range: %.2f", pt.first, pt.second, obs_range);
            return true;
          }
        }
      }
      return false;
    }
  };
  
  int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWAPlanner>());
    rclcpp::shutdown();
    return 0;
  }
