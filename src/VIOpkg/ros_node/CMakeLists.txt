cmake_minimum_required(VERSION 3.0.2)
project(ros_node)


# # include路径
# include_directories(${PROJECT_SOURCE_DIR}/inc)

find_package(OpenCV REQUIRED) 
include_directories(${OpenCV_INCLUDE_DIR})   


find_package(Eigen3) 
include_directories(${EIGEN3_INCLUDE_DIR})    

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)
# include路径
include_directories(${catkin_INCLUDE_DIRS})

include_directories("./ros_node/inc" 
                    "./parameters/src"
                    "./odometry/inc"
                    "./feature_tracker/inc")

add_library(${PROJECT_NAME} src/publisher.cpp
                            src/subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})