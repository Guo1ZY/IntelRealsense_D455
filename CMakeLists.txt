cmake_minimum_required(VERSION 3.10)
project(IntelRealsenseD455)

set(CMAKE_CXX_STANDARD 17)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(include ${realsense2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(IntelRealsense_D455 main.cpp source/Realsense.cpp)

target_link_libraries(IntelRealsense_D455 ${realsense2_LIBRARY} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})