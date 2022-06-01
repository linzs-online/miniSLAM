cmake_minimum_required(VERSION 3.0)

project(test_node)

add_subdirectory(parameters)
#add_subdirectory(odometry)
add_subdirectory(feature_tracker)
add_subdirectory(ros_node)

set(CMAKE_CXX_FLAGS "-std=c++14")

# 生成debug版本
SET(CMAKE_BUILD_TYPE "debug")
if (CMAKE_BUILD_TYPE STREQUAL debug)
    add_definitions(-D_DEBUG)
endif ()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  geometry_msgs
  image_transport
)
include_directories(${catkin_INCLUDE_DIRS}
                    "/usr/include/eigen3")

# 设置可执行文件生成路径
set(EXECUTABLE_OUTPUT_PATH ../bin)

add_executable(${PROJECT_NAME} test_node.cpp)
target_link_libraries(${PROJECT_NAME} parameters ros_node feature_tracker)
