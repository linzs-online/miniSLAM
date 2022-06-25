cmake_minimum_required(VERSION 3.0.2)
project(odometry)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
)
# include路径
include_directories(${PROJECT_SOURCE_DIR}/inc
                    ${catkin_INCLUDE_DIRS})
find_package(Eigen3) 
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Ceres REQUIRED) 
include_directories(${Ceres_INCLUDE_DIR})        


include_directories("./ros_node/inc" 
                    "./parameters/src"
                    "./odometry/inc"
                    "./feature_tracker/inc")

add_library(${PROJECT_NAME} 
            src/estimator.cpp
            src/initial_alignment.cpp
            src/initial_ex_rotation.cpp
            src/feature_manager.cpp
            src/pre_integrated.cpp
            src/initial_sfm.cpp
            src/marginalization_factor.cpp
            src/pose_local_parameterization.cpp
            src/projectionOneFrameTwoCamFactor.cpp
            src/projectionTwoFrameOneCamFactor.cpp
            src/projectionTwoFrameTwoCamFactor.cpp
            src/solve_5pts.cpp
            src/utility.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES})