# Autonomous TurtleBot3

## Setup

Install dependencies:

[Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
[slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
[nav2](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
[gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection)

## Usage

### Autonomous Exploration

In different terminals, run the following:

```bash
cd autonomous_turtlebot3

ros2 launch tb3_bringup sim_world.launch.py world:=$PWD$/src/tb3_bringup/worlds/office_small.world

ros2 launch tb3_bringup online_async_launch.py

ros2 launch tb3_bringup navigation_launch.py

./run_rviz.sh

ros2 launch tb3_bringup explore_frontier.launch.py
```

Video:
[![Section 1 Video]](https://raw.githubusercontent.com/anishmrao/autonomous_turtlebot3/master/videos/s1_vid.mp4)

### Agentic Semantic Reasoning

This is a simple mock implementation of agentic semantic reasoning. An example scene graph is precomputed and stored in `src/tb3_explorer/resources/scene_graph.json`

In different terminals, run the following:

```bash
cd autonomous_turtlebot3

ros2 launch tb3_bringup sim_world.launch.py world:=$PWD$/src/tb3_bringup/worlds/office_small.world

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$PWD/maps/office_small_s1_t2.yaml

ros2 run tb3_explorer semantic_query --ros-args -p scene_json_path:=$PWD/src/tb3_explorer/resource/scene_graph.json

ros2 topic pub /semantic_query std_msgs/String "{data: 'Go to the filing cabinet'}"
```


Video:
[![Section 2 Video]](https://raw.githubusercontent.com/anishmrao/autonomous_turtlebot3/master/videos/s2_vid.mp4)

### RRT* Global Planner

In different terminals, run the following:

```bash
cd autonomous_turtlebot3

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$PWD/maps/office_small_s1_t2.yaml

ros2 run rviz2 rviz2 -d $PWD/src/tb3_bringup/config/rviz_rrt.rviz

ros2 run tb3_explorer rrt_planner --ros-args -p planner_type:=rrt_star -p single_shot:=true
```

Video:
[![Section 3 Video]](https://raw.githubusercontent.com/anishmrao/autonomous_turtlebot3/master/videos/s3_vid.mp4)



