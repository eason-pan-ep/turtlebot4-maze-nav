# TurtleBot4 Navigation with Obstacle Avoidance

This package provides enhanced navigation capabilities for TurtleBot4 robots operating in environments with dynamic or unmapped obstacles. Multiple navigation strategies are available to handle different scenarios.

## Navigation Options

We provide three different navigation approaches, each with its own strengths:

### 1. SLAM Navigation (Recommended)

This approach uses Simultaneous Localization and Mapping (SLAM) to dynamically build a map of the environment including all obstacles.

```bash
ros2 launch turtlebot4_safety slam_navigation.launch.py
```

**Benefits:**
- Builds a complete map of the environment including all obstacles
- Doesn't require a pre-existing map
- Adapts to changes in the environment
- Best for exploring new areas or environments with many unmapped obstacles

### 2. SLAM with Prior Map Visualization

This approach uses SLAM but also displays your empty room map as a reference layer in RViz. This helps you visualize what parts of the environment are fixed vs. what are dynamic obstacles.

```bash
ros2 launch turtlebot4_safety slam_with_visible_prior.launch.py
```

**Benefits:**
- Same benefits as basic SLAM navigation
- Additional visual reference of the empty room for comparison
- Useful for education or demonstrations about SLAM vs. static maps

### 3. Map-Based Navigation with Obstacle Avoidance

This approach uses a pre-existing map of the room (without obstacles) and performs real-time obstacle detection and avoidance.

```bash
ros2 launch turtlebot4_safety obstacle_navigation.launch.py
```

**Benefits:**
- Fast localization using a known map structure
- Lower computational requirements than SLAM
- Good for well-mapped environments with occasional obstacles
- Works well when obstacles are easily detectable by sensors

## Setting Navigation Goals

In all approaches, you can set navigation goals using RViz:

1. Click on the "2D Nav Goal" button in the RViz toolbar
2. Click and drag on the map to set a goal position and orientation
3. The robot will navigate to the goal while avoiding obstacles

## Safety Parameters

The safety parameters are configurable for each approach:

- `inflation_radius`: How much space to keep around obstacles (default: 0.20m)
- `obstacle_threshold`: Distance to consider something an obstacle (default: 0.5m)
- `min_approach_distance`: Minimum distance to keep from obstacles (default: 0.10m)
- `footprint_padding`: Extra buffer around the robot's physical size (default: 0.08m)

You can modify these parameters in the launch files or set them dynamically:

```bash
ros2 param set /safety_parameter_node inflation_radius 0.25
```

## Troubleshooting

### Robot doesn't avoid obstacles

- Increase the `inflation_radius` and `footprint_padding` values
- Decrease the `obstacle_threshold` value
- Make sure the LIDAR is functioning correctly

### Navigation is too cautious

- Decrease the `inflation_radius` and `footprint_padding` values
- Increase the `obstacle_threshold` value

### Robot gets stuck

- Try clearing the costmaps:
  ```bash
  ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Trigger
  ```

### SLAM map doesn't look right

- Move the robot slowly to allow for better mapping
- Ensure the environment has enough distinct features
- Try revisiting areas to improve map quality

## Implementation Details

This package includes several key components:

1. **safety_params**: Manages safety-related parameters for navigation 
2. **obstacle_avoidance_nav**: Provides enhanced obstacle detection and visualization
3. **slam_navigation.launch.py**: Launches SLAM-based navigation
4. **slam_with_room.launch.py**: SLAM with reference map visualization
5. **obstacle_navigation.launch.py**: Map-based navigation with obstacle avoidance

## Requirements

- ROS2 Humble
- Gazebo Garden/Ignition Gazebo
- TurtleBot4 packages
- Nav2
- SLAM

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/your_workspace/src
   git clone <repository_url> turtlebot4_safety
   ```

2. Build the package:
   ```bash
   cd ~/your_workspace
   colcon build --symlink-install --packages-select turtlebot4_safety
   ```

3. Source the workspace:
   ```bash
   source ~/your_workspace/install/setup.bash
   ```