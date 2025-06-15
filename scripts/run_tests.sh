#!/bin/bash

echo "Building IKCalculator test program..."
cd "$(dirname "$0")/.."

if command -v catkin_make &> /dev/null; then
    cd ../../..
    catkin_make --pkg robotprogramming
    cd -
    
    echo "Running IKCalculator tests..."
    ../../../devel/bin/test_ik_calculator
else
    echo "Warning: catkin_make not found. Please build manually:"
    echo "cd ~/catkin_ws && catkin_make --pkg robotprogramming"
    echo "Then run: ~/catkin_ws/devel/bin/test_ik_calculator"
fi