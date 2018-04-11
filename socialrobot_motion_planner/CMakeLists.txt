cmake_minimum_required(VERSION 2.8.3)
project(socialrobot_motion_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp moveit_ros_planning_interface moveit_msgs geometry_msgs trajectory_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_service_files(FILES
  ArmPlanning.srv)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs trajectory_msgs
)


catkin_package(
)

add_executable(planner_node src/planner_node.cpp)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

