cmake_minimum_required(VERSION 3.0.2)
project(feature_tracker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
)
# include路径
include_directories(${PROJECT_SOURCE_DIR}/inc
                    ${catkin_INCLUDE_DIRS}
                    "/usr/include/eigen3" )

include_directories("./ros_node/inc" 
"./parameters/src"
"./odometry/inc"
"./feature_tracker/inc")

add_library(${PROJECT_NAME} src/feature_tracker.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})