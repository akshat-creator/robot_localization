# Robot Localization - Particle Filter Implementation

**Authors**: Akshat Jain, Tabitha Davidson, Owen Himsworth **Date**: October
2025  
**Course**: Computational Robotics, Olin College

---

## Project Goal

The goal of this project was to implement a **Monte Carlo Localization (MCL)**
system using a particle filter algorithm to enable a mobile robot to determine
its position and orientation within a known map. The particle filter uses
odometry data and laser scan measurements to probabilistically(big word I know)
estimate the robot's pose, allowing it to localize itself in real-time as it
moves through an environment.

### Key Objectives:

- Implement a fully functional particle filter for robot localization
- Handle sensor noise and odometry uncertainty through probabilistic methods
- Achieve real-time performance suitable for autonomous navigation
- Validate the implementation using recorded bag files from a Turtlebot4 robot

---

## Project Demo

🎥 **Video Demo**: [Link to demo video - upload and add link here]

![Particle Filter in Action](images/particle_filter_demo.gif)

---

## Implementation Overview

### High-Level Algorithm

Our particle filter follows the classic Monte Carlo Localization algorithm with
four main steps:

1. **Initialization**: Generate a set of particles (hypotheses) randomly
   distributed across the map
2. **Motion Update**: Update each particle's pose based on odometry data, adding
   noise to account for motion uncertainty
3. **Measurement Update**: Weight each particle based on how well its predicted
   laser scan matches the actual scan
4. **Resampling**: Generate a new set of particles by sampling from the current
   set, with probability proportional to particle weights

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
                             │
                             ▼
                     ┌──────────────┐
                     │ map→odom tf  │
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

### 1. Number of Particles: 300

**Decision**: Use 300 particles as the default particle count.

**Reasoning**:

-

**Alternatives Considered**:

-

### 2. Laser Scan Downsampling: Every 20th Beam

**Decision**: Use every 20th laser beam (18 beams total from 360) for particle
weighting.

**Reasoning**:

-

**Trade-offs**:

-

### 3. Likelihood Field vs. Ray Casting

**Decision**: Use the pre-computed likelihood field (occupancy field) approach.

**Reasoning**:

-

**Implementation**:

- The occupancy field is computed once at initialization using scikit-learn's
  ball tree algorithm
- Each map cell stores the distance to the nearest obstacle
- Particle weights are computed by querying this field for each laser endpoint

### 4. Noise Parameters

**Decision**: Motion noise σ_trans = 0.02m, σ_rot = 0.01rad; Sensor noise σ =
0.2m

**Reasoning**:

-

---

## Challenges Faced

### 1.

**Challenge**:

**Solution**:

-

### 2.

**Challenge**:

**Solution**:

-

### 3.

**Challenge**:

**Solution**:

-

### 4.

**Challenge**:

**Solution**:

-

---

## Results and Performance

### Quantitative Metrics

- **Convergence Time**:

- **Steady-State Error**:
- **Processing Rate**:
- **Success Rate**:

### Qualitative Observations

-

### Test Environment

- Map: MAC First Floor (structured indoor environment)
- Robot: Turtlebot4 with 360° Lidar
- Bag files: Two separate runs covering ~100m of travel

---

## Future Improvements

Given more time, here are enhancements we would implement:

### 1. Adaptive Particle Count (KLD-Sampling)

Dynamically adjust the number of particles based on pose uncertainty. Use fewer
particles when well-localized, more when uncertain. This could improve
efficiency while maintaining robustness.

### 2. Improved Sensor Model

- Use actual beam endpoints rather than downsampling
- Implement GPU acceleration for parallel likelihood computation
- Add multi-modal sensor fusion (incorporate IMU data)

### 3. Global Localization

Enhance the kidnapped robot problem handling with:

- Multiple hypothesis tracking
- Mixture of uniform and focused initialization
- Automatic recovery detection

### 4. Map Update Capability

Extend to SLAM (Simultaneous Localization and Mapping):

- Update map based on inconsistencies between scans and map
- Handle dynamic obstacles
- Detect and respond to map changes

### 5. Integration with Navigation Stack

- Connect to ROS2 Nav2 for autonomous navigation
- Implement path planning using localization output
- Add collision avoidance behaviors

---

## Lessons Learned

### Technical Lessons

1.

2.

3.

4.

### Teamwork Lessons

1.

2.

3.

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
