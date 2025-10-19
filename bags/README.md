# Bag Files Documentation

This directory contains ROS2 bag files used for testing and demonstrating the
particle filter localization system.

## Available Bag Files

### 1. macfirst_floor_take_1

- **Location**: `bags/macfirst_floor_take_1/`
- **Map**: `maps/mac_1st_floor_final.yaml`
- **Description**: First data collection run on the MAC first floor
- **Collection Method**: Recorded using Turtlebot4 robot with the following
  topics:
  - `/scan` - Lidar scan data
  - `/odom` - Odometry information
  - `/tf` and `/tf_static` - Transform data
  - `/clock` - Timing information
- **Use Case**: Primary testing dataset for particle filter development and
  validation
- **What You Can Learn**:
  - Basic particle filter convergence behavior
  - Performance in a structured indoor environment
  - How the particle cloud concentrates around the true robot pose over time

### 2. macfirst_floor_take_2

- **Location**: `bags/macfirst_floor_take_2/`
- **Map**: `maps/mac_1st_floor_final.yaml`
- **Description**: Second data collection run on the MAC first floor
- **Collection Method**: Recorded using Turtlebot4 robot (same topics as take_1)
- **Use Case**: Secondary dataset for validating consistency and robustness
- **What You Can Learn**:
  - Particle filter consistency across different runs
  - Handling of similar environments with different trajectories
  - Robustness to slightly different initial conditions

## How to Use These Bag Files

### Running a Bag File

```bash
# Play the bag file with clock information
ros2 bag play path/to/bag/file --clock
```

### Testing Your Particle Filter

1. Start the particle filter with the corresponding map:

```bash
ros2 launch robot_localization test_pf.py map_yaml:=/path/to/mac_1st_floor_final.yaml
```

2. Open RViz with the appropriate configuration:

```bash
rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz
```

3. Set an initial pose estimate using the "2D Pose Estimate" tool in RViz

4. Play the bag file:

```bash
ros2 bag play ~/ros2_ws/src/robot_localization/bags/macfirst_floor_take_1 --clock
```

## Performance Observations

### Convergence Behavior

-MAYBE? TODO

### Computational Performance

- MAYBE TODO

### Localization Accuracy

- MAYBE TODO

## Notes

- These bag files were recorded from a physical Turtlebot4 robot
- Make sure Gazebo is not running when playing these bag files
- Ensure you're not connected to a physical robot when using bag files
