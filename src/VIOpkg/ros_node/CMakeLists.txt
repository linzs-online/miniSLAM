cmake_minimum_required(VERSION 3.0.2)
project(ros_node)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  geometry_msgs
  image_transport
)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")


# include路径
include_directories(${PROJECT_SOURCE_DIR}/inc
                    ${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    "/usr/include/eigen3"   # eigen 库只有头文件
                    )

add_library(${PROJECT_NAME} src/publisher.cpp src/subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})