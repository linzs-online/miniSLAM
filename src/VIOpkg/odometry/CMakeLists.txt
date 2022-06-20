cmake_minimum_required(VERSION 3.0.2)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  geometry_msgs
  image_transport
  OpenCV
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odometry
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${PROJECT_SOURCE_DIR}/inc
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)
add_executable(${PROJECT_NAME}_node src/odometry_node.cpp)
