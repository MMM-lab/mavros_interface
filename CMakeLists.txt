cmake_minimum_required(VERSION 3.0.2)
project(mavros_interface)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fmu_commander
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(offb_node src/offb_node.cpp)
add_executable(vel_sp_test src/vel_sp_test.cpp)
add_executable(mavros_interface src/mavros_interface.cpp)

target_link_libraries(offb_node
  ${catkin_LIBRARIES}
)
target_link_libraries(vel_sp_test
  ${catkin_LIBRARIES}
)
target_link_libraries(mavros_interface
  ${catkin_LIBRARIES}
)
