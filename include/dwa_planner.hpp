#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


struct VelocitySample {
  double v;      // linear velocity
  double w;      // angular velocity
};

struct Trajectory {
  std::vector<geometry_msgs::msg::Pose> poses;
  double cost;
  VelocitySample cmd;
};

class DWAPlanner : public rclcpp::Node {
public:
  DWAPlanner();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void timerCallback();

  Trajectory simulateTrajectory(double v, double w);
  double calculateTrajectoryCost(const Trajectory& traj);
  VelocitySample findBestVelocity();
  void publishTrajectoriesMarkers(const std::vector<Trajectory>& trajs, const VelocitySample& best_cmd);


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  geometry_msgs::msg::Pose goal_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool goal_received_ = false;

  // Parameters
  double max_vel_, max_w_;
  double max_acc_, max_ang_acc_;
  double current_linear_vel_, current_angular_vel_;
  double sim_time_, dt_;
  
};
