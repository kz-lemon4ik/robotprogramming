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

### Workspace Setup

This package requires a catkin workspace. Create and configure:

```bash
# Create workspace structure
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone or symlink the package
git clone <repository-url> robotprogramming
# OR if you have local copy: ln -s /path/to/robotprogramming .

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Note**: The `catkin_ws` directory contains build artifacts and should not be committed to version control. Only the package source code in `robotprogramming/` is tracked by git.

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

The keyboard controller will start in the same terminal. Enter target coordinates when prompted using space-separated format: `1.5 0.5` (not comma-separated).

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
# Method 1: Using provided script
./scripts/run_tests.sh

# Method 2: Manual execution
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

## Dependencies

### System Requirements
- Ubuntu 20.04 LTS
- ROS Noetic Ninjemys
- C++17 compiler

### Installation
```bash
# Install ROS dependencies
rosdep install --from-paths . --ignore-src -r -y

# Or manually install required packages
sudo apt install ros-noetic-roscpp ros-noetic-std-msgs ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-robot-state-publisher ros-noetic-rviz
sudo apt install libeigen3-dev
```

See `package.xml` for detailed dependency information.

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
├── config/                # Configuration
│   └── rviz_config.rviz
└── scripts/               # Utility scripts
    ├── run_tests.sh
    └── test_ik_node.sh
```

## Code Formatting

This project uses clang-format for consistent code style:

```bash
# Install clang-format (if not installed)
sudo apt install clang-format

# Format all C++ files
find src include -name "*.cpp" -o -name "*.h" | xargs clang-format -i

# Check formatting without modifying files
find src include -name "*.cpp" -o -name "*.h" | xargs clang-format --dry-run --Werror
```

## Contributing

1. Follow ROS coding standards
2. Use clang-format before committing code
3. Write unit tests for new features
4. Update documentation
5. Test with different robot parameters

## License

This project is part of an academic robot programming course.
