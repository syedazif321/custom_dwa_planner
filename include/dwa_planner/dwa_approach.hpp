// dwa_approach.hpp
#ifndef DWA_PLANNER__DWA_APPROACH_HPP_
#define DWA_PLANNER__DWA_APPROACH_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>

namespace dwa_planner
{

struct DWAParams
{
  double max_speed;
  double min_speed;
  double max_yaw_rate;
  double max_accel;
  double max_delta_yaw_rate;
  double velocity_resolution;
  double yaw_rate_resolution;
  double dt;
  double predict_time;
  double to_goal_cost_gain;
  double speed_cost_gain;
  double obstacle_cost_gain;
  double robot_radius;
};

class DWAPlanner
{
public:
  DWAPlanner();
  void setParams(const DWAParams &params);
  void setGoal(const geometry_msgs::msg::PoseStamped & goal);
  void setScan(const sensor_msgs::msg::LaserScan::SharedPtr & scan);
  geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Odometry::SharedPtr & odom);

private:
  DWAParams params_;
  geometry_msgs::msg::PoseStamped goal_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  double calcObstacleCost(double vx, double vyaw) const;
  double calcHeadingCost(double vx, double vyaw) const;
  double calcSpeedCost(double vx, double vyaw) const;
};

}  

#endif  
