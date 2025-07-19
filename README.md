# 2-Link Robotic Arm Inverse Kinematics

A ROS package that solves inverse kinematics for a 2-link planar manipulator. The system takes target coordinates (x, y) and calculates joint angles, visualizing results in RViz.

## Features

- Real-time inverse kinematics calculation
- 3D visualization in RViz
- Interactive keyboard control
- Reachability checking
- Multiple control interfaces (rostopic, keyboard)

## Technical Specifications

- **OS**: Ubuntu 20.04 LTS
- **Framework**: ROS Noetic
- **Language**: C++17
- **Libraries**: Eigen, roscpp, geometry_msgs, sensor_msgs

## Installation

### Prerequisites

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install libeigen3-dev
```

### Build Instructions

1. Create catkin workspace (if not exists):

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

2. Clone and build the package:

```bash
cd ~/catkin_ws/src
git clone <repository-url> robotprogramming
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Basic Launch

Start the complete system with RViz visualization:

```bash
roslaunch robotprogramming robot_control.launch
```

### Keyboard Control

Launch with interactive keyboard controller:

```bash
roslaunch robotprogramming keyboard_control.launch
```

Enter target coordinates when prompted (e.g., `1.5 0.5`).

### Manual Control

Send target coordinates via rostopic:

```bash
# In another terminal
source ~/catkin_ws/devel/setup.bash
rostopic pub /target_goal geometry_msgs/Point '{x: 1.5, y: 0.5, z: 0.0}' -1
```

### Launch Parameters

Customize robot parameters:

```bash
roslaunch robotprogramming robot_control.launch link1_length:=1.5 link2_length:=1.2
```

Available parameters:

- `link1_length`: Length of first link (default: 1.0)
- `link2_length`: Length of second link (default: 1.0)
- `use_rviz`: Enable RViz visualization (default: true)

## Robot Model

2-link planar manipulator with:

- Base link at origin
- Link 1: length L1 (red visualization)
- Link 2: length L2 (blue visualization)
- End effector (green sphere)

Workspace: circular region with radius [|L1-L2|, L1+L2]

## ROS Architecture

### Nodes

- `ik_solver_node`: Main inverse kinematics solver
- `robot_state_publisher`: Publishes robot transforms
- `keyboard_controller_node`: Interactive keyboard input (optional)

### Topics

**Subscribed:**

- `/target_goal` (geometry_msgs/Point): Target coordinates

**Published:**

- `/joint_states` (sensor_msgs/JointState): Joint angles
- `/target_reachable` (std_msgs/Bool): Reachability status
- `/ik_status` (std_msgs/String): Calculation status

### Services

None

## Mathematical Background

The inverse kinematics solution uses geometric approach:

1. Calculate distance to target: d = √(x² + y²)
2. Check reachability: |L1-L2| ≤ d ≤ L1+L2
3. Apply law of cosines for joint angles
4. Select "elbow up" configuration

## Testing

### Unit Tests

```bash
# Build and run IK calculator tests
cd ~/catkin_ws
catkin_make
rosrun robotprogramming test_ik_calculator
```

### System Test

```bash
# Test reachable target
rostopic pub /target_goal geometry_msgs/Point '{x: 1.5, y: 0.5, z: 0.0}' -1

# Test unreachable target
rostopic pub /target_goal geometry_msgs/Point '{x: 3.0, y: 0.0, z: 0.0}' -1

# Test boundary case
rostopic pub /target_goal geometry_msgs/Point '{x: 2.0, y: 0.0, z: 0.0}' -1
```

Monitor output topics:

```bash
rostopic echo /joint_states
rostopic echo /target_reachable
rostopic echo /ik_status
```

## Troubleshooting

### Common Issues

**Robot not visible in RViz:**

- Expand display tree in RViz left panel
- Check that Fixed Frame is set to "base_link"
- Verify robot_state_publisher is running

**IK calculation fails:**

- Check target coordinates are within workspace
- Verify link lengths are positive
- Check parameter server values

**Build errors:**

- Ensure Eigen3 is installed: `sudo apt install libeigen3-dev`
- Check ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`

### Debug Commands

```bash
# Check running nodes
rosnode list

# Monitor topics
rostopic list
rostopic hz /joint_states

# Check parameters
rosparam list
rosparam get /link1_length

# View TF tree
rosrun tf view_frames
evince frames.pdf
```

## File Structure

```
robotprogramming/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── README.md              # This file
├── launch/                # Launch files
│   ├── robot_control.launch
│   └── keyboard_control.launch
├── urdf/                  # Robot model
│   └── robot.urdf
├── src/                   # Source files
│   ├── ik_calculator.cpp
│   ├── ik_solver_node.cpp
│   ├── keyboard_controller_node.cpp
│   └── test_ik_calculator.cpp
├── include/robotprogramming/  # Headers
│   └── ik_calculator.h
└── config/                # Configuration
    └── rviz_config.rviz
```

## Contributing

1. Follow ROS coding standards
2. Write unit tests for new features
3. Update documentation
4. Test with different robot parameters

## License

This project is part of an academic robot programming course.
