cmake_minimum_required(VERSION 3.0.2)
project(robotprogramming)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotprogramming
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/ik_calculator.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/ik_solver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

add_executable(test_ik_calculator src/test_ik_calculator.cpp)

target_link_libraries(test_ik_calculator
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

add_executable(keyboard_controller_node src/keyboard_controller_node.cpp)

target_link_libraries(keyboard_controller_node
  ${catkin_LIBRARIES}
)