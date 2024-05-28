cmake_minimum_required(VERSION 3.5)
project(lidar2camera)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Set compiler options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED filesystem system)
find_package(PCL REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${BOOST_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

add_executable(run_lidardetection src/run_lidar_detection.cpp )
target_include_directories(run_lidardetection PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(run_lidardetection ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME} libjsoncpp.a)

add_executable(calibration_target_mask src/calibration_target_mask.cpp )
target_include_directories(calibration_target_mask PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(calibration_target_mask ${PCL_LIBRARIES} pthread)

# Use Qt's MOC on the header file containing BasePointCloudViewer
QT5_WRAP_CPP(MOC_FILES include/BasePointCloudViewer.h)

add_executable(pointcloud_viewer src/pointcloud_viewer.cpp ${MOC_FILES})
target_link_libraries(pointcloud_viewer ${PCL_LIBRARIES} Qt5::Widgets pthread)
