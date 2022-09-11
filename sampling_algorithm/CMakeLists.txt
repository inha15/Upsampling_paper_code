cmake_minimum_required(VERSION 3.0.2)
project(sampling_algorithm)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs pcl_conversions pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sampling src/smp.cpp)
target_link_libraries(sampling ${catkin_LIBRARIES} pcl_visualization)
