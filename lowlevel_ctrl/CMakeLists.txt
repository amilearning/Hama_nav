cmake_minimum_required(VERSION 3.0.2)
project(lowlevel_ctrl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mavros_msgs    
  ackermann_msgs
  tf2
  tf2_geometry_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/LowLevelControl.cfg
)


catkin_package(
INCLUDE_DIRS
CATKIN_DEPENDS roscpp
               sensor_msgs
               std_msgs
               ackermann_msgs
               dynamic_reconfigure
)



include_directories(${catkin_INCLUDE_DIRS})

add_executable(lowlevel_ctrl src/main.cpp)
target_link_libraries(lowlevel_ctrl ${catkin_LIBRARIES})
add_dependencies(lowlevel_ctrl ${PROJECT_NAME}_gencfg)

