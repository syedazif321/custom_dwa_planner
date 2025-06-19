#  Custom DWA Local Planner for TurtleBot3 (ROS 2 Humble)

This package provides a **fully custom Dynamic Window Approach (DWA)** local planner implementation for **TurtleBot3 Burger** using **ROS 2 Humble**. It performs velocity-based trajectory simulation, obstacle avoidance, and local path planning using LaserScan data — without relying on `nav2` or the default `dwb_controller`.

---

##  Package Overview

**Package Name:** `dwa_planner`

### Features

 Fully implemented from scratch in C++  
 Simulates multiple velocity trajectories and selects the best one  
 Integrates with `/odom`, `/scan`, `/cmd_vel`, `/goal_pose`  
 Uses laser scan to perform obstacle avoidance  
 Works with RViz2 and Gazebo simulation  
 Easily configurable via YAML

---

## 🛠️ Prerequisites

- ROS 2 Humble
- `turtlebot3` packages installed
- RViz2
- Gazebo (for simulation)
- TF2 properly configured (e.g., `odom → base_link → base_scan`)

---

## 📦 Installation

```bash
# Clone into your ROS 2 workspace
cd ~/robot_ws/src
git clone https://github.com/syedazif321/dwa_planner.git

# Install dependencies if required
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select dwa_planner

# Source the workspace
source install/setup.bash
