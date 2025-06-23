# 🌀 Custom Dynamic Window Approach (DWA) Local Planner

This package implements a **fully custom Dynamic Window Approach (DWA)** local planner for **TurtleBot3 Burger** using **ROS 2 Humble**. It performs velocity-based trajectory simulation, real-time obstacle avoidance using laser data, and smooth local navigation — all built from scratch without using `nav2` or the `dwb_controller`.

---

## ✨ Features

- 🛠️ Written in pure C++ (ROS 2 style)
- 🚗 Velocity sampling & trajectory simulation
- 🧠 Obstacle avoidance using `/scan`
- 🧭 Goal tracking via `/goal_pose`
- ✅ Works with Gazebo & RViz2
- 📌 Minimal dependencies, fully configurable

---

## 🛠️ Prerequisites

- ROS 2 Humble
- TurtleBot3 installed
- Gazebo (TurtleBot3 simulation)
- RViz2
- Proper TF setup: `odom → base_link → base_scan`

---

## 📦 Installation

```bash
# Create a ROS 2 workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone this repo
git clone https://github.com/syedazif321/custom_dwa_planner.git

# Install dependencies
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select custom_dwa_planner
source install/setup.bash
````

---

## 🚀 Launch Instructions

### 1. Launch TurtleBot3 in Gazebo

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Launch the DWA Planner

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch custom_dwa_planner dwa_planner.launch.py
```

---

## 📌 Topics Used

| Topic               | Type                                 | Description                       |
| ------------------- | ------------------------------------ | --------------------------------- |
| `/odom`             | `nav_msgs/msg/Odometry`              | Provides robot position           |
| `/scan`             | `sensor_msgs/msg/LaserScan`          | Used for obstacle detection       |
| `/goal_pose`        | `geometry_msgs/msg/PoseStamped`      | Receives target goal from RViz    |
| `/cmd_vel`          | `geometry_msgs/msg/Twist`            | Publishes robot velocity commands |
| `/dwa_trajectories` | `visualization_msgs/msg/MarkerArray` | Visualizes sampled trajectories   |

---

## 🧠 Planner Logic

* **Dynamic Window**: Sample linear/angular velocities based on current velocity and acceleration limits.
* **Simulation**: Predict future trajectories using velocity samples.
* **Scoring**: Each trajectory is scored based on distance to goal, obstacle proximity, and turning effort.
* **Best Path**: The lowest-cost, obstacle-free trajectory is executed.

---

## 🔧 Configuration

You can customize parameters like velocity limits, acceleration, simulation time, and scoring weights in `params.yaml`.

---

## 📸 RViz Tips

* Use the **2D Nav Goal** tool to set a goal.
* Check the `/dwa_trajectories` markers to visualize all sampled paths.
* The green path is the selected best trajectory.

---

## 📬 Maintainer

**Mohammed Azif**
🔗 [LinkedIn](https://www.linkedin.com/in/your-profile)
📧 Contact via GitHub Issues

---

## 📝 License

This project is licensed under the [MIT License](LICENSE).