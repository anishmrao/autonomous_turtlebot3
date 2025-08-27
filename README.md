# 🚀 Autonomous TurtleBot3

[![ROS 2](https://img.shields.io/badge/ROS-2-FC521D?logo=ros)](https://docs.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Advanced autonomous navigation and exploration system for TurtleBot3 using ROS 2, featuring SLAM, semantic reasoning, and path planning capabilities.

## 📋 Table of Contents
- [Features](#-key-features)
- [Setup](#-setup)
- [Usage](#-usage)
  - [Autonomous Exploration](#-autonomous-exploration)
  - [Agentic Semantic Reasoning](#-agentic-semantic-reasoning)
  - [RRT* Global Planner](#-rrt-global-planner)

## ✨ Key Features

- **Autonomous Exploration**: Frontier-based exploration in unknown environments
- **Semantic Navigation**: Natural language command processing
- **Path Planning**: RRT* algorithm for optimal path finding
- **Simulation Ready**: Pre-configured for Gazebo
- **Modular Design**: Easy to extend with new features

## 🛠️ Setup

### Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress
- Python 3.8+

### Dependencies

Install the following core dependencies:

- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [nav2](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection)

### Quick Installation

```bash
# Install ROS 2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install dependencies
sudo apt update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs

# Clone and build the workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### 🌍 Autonomous Exploration

In different terminals, run the following:

```bash
cd autonomous_turtlebot3

ros2 launch tb3_bringup sim_world.launch.py world:=$PWD/src/tb3_bringup/worlds/office_small.world

ros2 launch tb3_bringup online_async_launch.py

ros2 launch tb3_bringup navigation_launch.py

ros2 run rviz2 rviz2 -d $PWD/src/tb3_bringup/config/rviz_tb3.rviz

ros2 launch tb3_bringup explore_frontier.launch.py
```

Video:


https://github.com/user-attachments/assets/d7d80d70-7b9e-4e84-979c-f29af3497e21

For full video, see `videos/s1_vid.mp4`

### 🤖 Agentic Semantic Reasoning

This is a simple *mock* implementation of agentic semantic reasoning. An example scene graph is precomputed and stored in `src/tb3_explorer/resources/scene_graph.json`

In different terminals, run the following:

```bash
cd autonomous_turtlebot3

ros2 launch tb3_bringup sim_world.launch.py world:=$PWD/src/tb3_bringup/worlds/office_small.world

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$PWD/maps/office_small_s1_t2.yaml

ros2 run tb3_explorer semantic_query --ros-args -p scene_json_path:=$PWD/src/tb3_explorer/resource/scene_graph.json

ros2 topic pub /semantic_query std_msgs/String "{data: 'Go to the filing cabinet'}"
```


Video:


https://github.com/user-attachments/assets/840b4485-28fc-43d7-b770-ebd414b92786

For full video, see `videos/s2_vid.mp4`.

### 🛣️ RRT* Global Planner

In different terminals, run the following:

```bash
cd autonomous_turtlebot3

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$PWD/maps/office_small_s1_t2.yaml

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

ros2 run rviz2 rviz2 -d $PWD/src/tb3_bringup/config/rviz_rrt.rviz

ros2 run tb3_explorer rrt_planner --ros-args -p planner_type:=rrt_star -p single_shot:=true
```

Video:




https://github.com/user-attachments/assets/4002304c-7736-4b35-8a97-00af5e5dbfd5

For full video, see `videos/s3_vid.mp4`.
