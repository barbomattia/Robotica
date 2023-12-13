cmake_minimum_required(VERSION 3.0.2)
project(task_planner)

find_package(catkin REQUIRED COMPONENTS
  robot_msgs
  rospy
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS robot_msgs rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(task_planner_node src/task_planner_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(task_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(task_planner_node
  ${catkin_LIBRARIES}
)
