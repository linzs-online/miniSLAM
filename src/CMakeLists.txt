cmake_minimum_required(VERSION 2.8.3)
project("miniSLAM")

# 生成debug版本
SET(CMAKE_BUILD_TYPE "debug")
if (CMAKE_BUILD_TYPE STREQUAL debug)
    add_definitions(-D_DEBUG)
endif ()
# set(CMAKE_MODULE_PATH /home/lin/OpenCV_Src/opencv-4.2.0/build)
# set(Opencv_DIR /home/lin/OpenCV_Src/opencv-4.2.0/build)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  geometry_msgs
  camera_models
  image_transport
)

# include路径
include_directories(${PROJECT_SOURCE_DIR}/inc
                    ${catkin_INCLUDE_DIRS} 
                    ${CERES_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    "/usr/include/eigen3"
                    )

# 设置可执行文件生成路径
set(EXECUTABLE_OUTPUT_PATH ../bin)

set(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -std=c++11")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

add_library(miniSLAM_lib
    parameters/src/parameters.cpp
    odometry/src/publisher.cpp
    odometry/src/feature_tracker.cpp
    odometry/src/subscriber.cpp
    )
target_link_libraries(miniSLAM_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(test_node test_node.cpp)
target_link_libraries(test_node miniSLAM_lib)