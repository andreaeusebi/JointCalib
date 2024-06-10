cmake_minimum_required(VERSION 3.0.2)
project(msg_saver_pkg)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(OpenCV 4.2.0 EXACT REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(msg_saver_node src/msg_saver_node.cpp)
target_link_libraries(msg_saver_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
