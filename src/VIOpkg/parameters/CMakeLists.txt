cmake_minimum_required(VERSION 3.0.2)
project(parameters)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/parameters.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBS})
