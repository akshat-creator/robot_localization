# Robot Localization - Particle Filter Implementation

**Authors**: Akshat Jain, Tabitha Davison, Owen Himsworth 
**Course**: Computational Robotics, Olin College

---

## Project Goal

This project implements a particle filter algorithm to estimate a robot’s position within a known map. 
Our system maintains a cloud of weighted particles representing possible robot locations and then uses Lidar sensor measurements to iteratively predict, 
correct and resample estimates of position and orientation until we have successfully converged on the Neato's position.

We tested our algorithm in simulation and using recorded bag files of the MAC 1st floor.

### Key Objectives:

- Implement a fully functional particle filter for robot localization
- Handle sensor noise and odometry uncertainty through probabilistic methods
- Achieve real-time performance suitable for autonomous navigation
- Validate the implementation using recorded bag files from a Turtlebot4 robot

---

## Project Demo

🎥 **Video Demo**: [https://youtu.be/1uryYJuzfIA]

![Particle Filter in Action](images/particle_filter_demo.gif)

---

## Implementation Overview


### Algorithm Approach

Our particle filter follows the following steps:

1. **Initialization** (`initialize_particle_cloud`): We generate a set of particles (neato position hypotheses) randomly distributed across the map or distributed around an initial guess we give. Each particle starts with equal weight, representing initial uncertainty and so the dsitribution sums up to one.
2. **Motion Update** (`update_particles_with_odom`): We compute change in odometry (`Δx, Δy, Δθ`) and update each particle by applying this delta plus Gaussian noise (+/- 0.02m, 0.01rad) to simulate real-world uncertainty.This predicts where the robot could be after movement.
3. **Measurement Update** (`(update_particles_with_laser)`):
We use every 20th laser beam to reduce computation. Transform laser hits into map coordinates based on the particle’s pose. We then query the occupancy_field.`get_closest_obstacle_distance()` to compare expected vs observed distances. Compute weight with a Gaussian likelihood:
`wᵢ = exp(−dᵢ² / (2σ²))`

- where `dᵢ`  is the distance to the closest obstacle and `sigma`  represents the sensor noise parameter.

4. **Pose Estimation**: Convert to quaternion and publish as `geometry_msgs/Pose`. We compute weighted average of particles:

`x = Σ wᵢ xᵢ`
`y = Σ wᵢ yᵢ`

`θ = atan2(Σ wᵢ sinθᵢ, Σ wᵢ cosθᵢ)`

5. **Resampling** (`resample_particles`): We use the helper draw_random_sample to resample particles based on their weights. Add small Gaussian noise to x, y, θ to avoid maintain diverity in the sample. This focuses computation on higher probability areas.
6. **Publishing**(`publish_particles`) We then update the estimate of the neato given the new particles, update the map to odom transform and publish particle cloud with new weighted particle set for visualization and pose estimation.
### System Architecture


```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│   /scan     │─────▶│              │      │   /odom     │
│  (Lidar)    │      │   Particle   │◀─────│ (Odometry)  │
└─────────────┘      │    Filter    │      └─────────────┘
                     │              │
┌─────────────┐      │   (pf.py)    │      ┌─────────────┐
│    /map     │─────▶│              │─────▶│/particle_   │
│  (Map srv)  │      └──────────────┘      │   cloud     │
└─────────────┘              │             └─────────────┘
         ▲                   │
         │                   ▼
┌──────────────┐      ┌──────────────┐
│ /initialpose │      │ map→odom tf  │
│  (from RViz) │      └──────────────┘
└──────────────┘
```

### Key Components

#### 1. Particle Representation (`Particle` class)

Each particle represents a hypothesis of the robot's pose:

- **Position**: (x, y) coordinates in the map frame
- **Orientation**: theta (yaw angle) in radians
- **Weight**: confidence value representing likelihood of this pose

#### 2. Motion Model (`update_particles_with_odom`)

- Computes the change in pose from consecutive odometry readings
- Applies this motion to each particle with added Gaussian noise
- Noise parameters:
  - Translation: σ = 0.02 m
  - Rotation: σ = 0.01 rad

#### 3. Sensor Model (`update_particles_with_laser`)

- Uses a **likelihood field** for efficient computation
- For each particle, projects laser scan points into the map frame
- Compares predicted obstacle distances with actual map obstacles
- Computes weight using Gaussian probability: `w = exp(-(d²)/(2σ²))`
- Performance optimization: uses every 20th laser beam (instead of all 360)

#### 4. Resampling (`resample_particles`)

- Implements **low-variance resampling** using the `draw_random_sample` helper
  function
- Focuses computational resources on high-probability regions
- Adds small random noise after resampling to prevent particle depletion

#### 5. Pose Estimation (`update_robot_pose`)

- Computes weighted mean of particle positions
- Uses circular mean for angular component to handle wraparound
- Updates the `map → odom` transform for ROS navigation stack integration

---

## Design Decisions

- Number of Particles: 500 — a balance between accuracy and computational cost.
- Noise Parameters: Tuned empirically to maintain convergence without over-spreading.
- Selective Laser Sampling (every 20th beam): Trade-off between speed and accuracy.
- Weighted mean pose estimation: Avoids instability when clusters form.

**Implementation**:

- The occupancy field is computed once at initialization using scikit-learn's
  ball tree algorithm
- Each map cell stores the distance to the nearest obstacle
- Particle weights are computed by querying this field for each laser endpoint

## Result

Initially, the particles spread across the map. As the robot moved, particles far from the correct pose were down weighted, and the cluster of high-weight particles converged near the true robot position. Over time, the filter tracked the robot accurately through the map.

## Challenges and Lessons Learned

- Debugging odometry updates — small sign errors caused a big difference.
- Visualization is key — seeing the particle cloud helps immediately diagnose logic bugs.
- Test incrementally — verifying each step (motion update, laser update, etc.) independently is crucial.
- More particles help
- Transforms are difficult to implement
### Future Improvements

If we had more time, there are several areas we would want to explore to make our particle filter more efficient and robust:

1. **Adaptive Particle Count (KLD-Sampling)**  
   Right now, the number of particles is fixed at 500, which works but isn’t always optimal.  
   We’d like to dynamically adjust the number of particles based on the robot’s pose uncertainty—using more particles when localization confidence is low and fewer when it’s high.  
   This would improve computational efficiency while maintaining accuracy.

2. **Improved Sensor Model**  
   Our current likelihood field model samples every 20th laser beam for performance reasons.  
   A more advanced version could use all beams (or weighted subsets) and possibly run the likelihood calculations in parallel using GPU acceleration.  
   We’d also be interested in fusing data from other sensors, like the IMU, to make localization more robust in feature-sparse areas.

3. **Map Update Capability (Toward SLAM)**  
   Right now, our particle filter assumes a static, known map.  
   A natural next step would be to extend this into a full SLAM system by updating the map based on mismatches between observed and expected laser data.  
   Handling dynamic obstacles or small environmental changes would make the system much more flexible.

5. **Integration with the ROS2 Navigation Stack**  
   Finally, we’d like to tie our localization system into ROS2’s Nav2 stack.  
   With localization feeding into a planner, we could test full autonomous navigation and collision avoidance.  
   This would demonstrate how our localization module fits into a complete robot autonomy pipeline.

---

## Code Structure

```
robot_localization/
├── robot_localization/
│   ├── pf.py                    # Main particle filter implementation
│   ├── occupancy_field.py       # Likelihood field for sensor model
│   ├── helper_functions.py      # Transform and utility functions
│   └── angle_helpers.py         # Quaternion/euler conversions
├── launch/
│   ├── test_pf.py              # Launch file for particle filter
│   ├── test_amcl.py            # Launch file for baseline comparison
│   └── launch_map_server.py    # Map server launch file
├── bags/
│   ├── macfirst_floor_take_1/  # Test bag file #1
│   ├── macfirst_floor_take_2/  # Test bag file #2
│   └── README.md               # Bag file documentation
├── maps/
│   └── mac_1st_floor_final.*   # Test environment map
└── rviz/
    └── turtlebot_bag_files.rviz # RViz configuration
```

---

## How to Run

### Prerequisites

```bash
# Install dependencies
pip3 install scikit-learn
sudo apt install -y ros-humble-nav2-map-server python3-pykdl
```

### Running the Particle Filter

1. **Build the workspace**:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

2. **Launch the particle filter**:

```bash
ros2 launch robot_localization test_pf.py map_yaml:=<path-to-map-yaml>
```

3. **Open RViz**:

```bash
rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz
```

4. **Play a bag file**:

```bash
ros2 bag play ~/ros2_ws/src/robot_localization/bags/macfirst_floor_take_1 --clock
```

5. **Set initial pose** in RViz using the "2D Pose Estimate" tool

6. Commands for help with docker or running the code can be found in commands.txt
---

## References

- **Particle Filter Overview**:
  [Probabilistic Robotics (Thrun, Burgard, Fox)](http://www.probabilistic-robotics.org/)
- **ROS2 Navigation**: [Nav2 Documentation](https://navigation.ros.org/)
- **Likelihood Field Model**:
  [Pieter Abbeel's Lecture Slides](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/slides/Lec6-ParticleFilters.pdf)
- **Course Resources**:
  [CompRobo at Olin College](https://comprobo24.github.io/)

---

## AI Usage (ChatGPT and Claude Sonnet via Git Co-Pilot)

- Used for Report Boiler Plate
- Spell Check and Formatting Errors
- Creation of System Architecture Diagram

---

_This project was completed as part of Computational Robotics at Olin College,
Fall 2025._
