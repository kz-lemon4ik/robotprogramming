#!/bin/bash

echo "Testing IK Solver Node..."
echo "Make sure to run 'roslaunch robotprogramming ik_solver.launch' first"
echo ""

# Test reachable targets
echo "Testing reachable targets:"
rostopic pub /target_goal geometry_msgs/Point "x: 1.5, y: 0.5, z: 0.0" -1
sleep 2

rostopic pub /target_goal geometry_msgs/Point "x: 0.0, y: 1.8, z: 0.0" -1
sleep 2

rostopic pub /target_goal geometry_msgs/Point "x: -1.0, y: 1.0, z: 0.0" -1
sleep 2

# Test unreachable target
echo "Testing unreachable target:"
rostopic pub /target_goal geometry_msgs/Point "x: 3.0, y: 0.0, z: 0.0" -1
sleep 2

echo "Test completed. Check the topics:"
echo "rostopic echo /joint_states"
echo "rostopic echo /target_reachable"
echo "rostopic echo /ik_status"