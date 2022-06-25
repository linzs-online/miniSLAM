cmake_minimum_required(VERSION 3.0)
project(vio)

# add_subdirectory(parameters)
# add_subdirectory(odometry)
# add_subdirectory(feature_tracker)
# add_subdirectory(ros_node)

set(CMAKE_BUILD_TYPE "debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport)
include_directories(${catkin_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")


# # 设置可执行文件生成路径
# set(EXECUTABLE_OUTPUT_PATH ../bin)

add_library(vio_lib
            feature_tracker/src/feature_tracker.cpp
            odometry/src/estimator.cpp
            odometry/src/feature_manager.cpp
            odometry/src/initial_alignment.cpp
            odometry/src/initial_ex_rotation.cpp
            odometry/src/initial_sfm.cpp
            odometry/src/marginalization_factor.cpp
            odometry/src/pose_local_parameterization.cpp
            odometry/src/pre_integrated.cpp
            odometry/src/projectionOneFrameTwoCamFactor.cpp
            odometry/src/projectionTwoFrameOneCamFactor.cpp
            odometry/src/projectionTwoFrameTwoCamFactor.cpp
            odometry/src/solve_5pts.cpp
            odometry/src/utility.cpp
            parameters/src/parameters.cpp
            ros_node/src/publisher.cpp
            ros_node/src/subscriber.cpp)   
target_link_libraries(vio_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(vio test_node.cpp)
target_link_libraries(vio vio_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})
