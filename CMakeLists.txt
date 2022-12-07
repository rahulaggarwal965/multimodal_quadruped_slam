cmake_minimum_required(VERSION 3.0.2)
project(quadruped_slam)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    
    cv_bridge
    pcl_ros
    pcl_conversions

    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs

    message_generation
)

find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(teaserpp REQUIRED)
find_package(pinocchio REQUIRED)

add_message_files(
    DIRECTORY msg
    FILES
    lidar_factor.msg
    ForwardKinematicFactor.msg
    ForwardKinematicFactorStamped.msg
    RigidContactFactor.msg
    RigidContactFactorStamped.msg
    ForwardKinematicChain.msg
    ForwardKinematicChainStamped.msg
    RigidContactChain.msg
    RigidContactChainStamped.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS PCL

    CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

add_executable(state_estimation src/slam.cpp src/optimizer.cpp src/imu.cpp)
add_dependencies(state_estimation ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_estimation ${catkin_LIBRARIES} gtsam)

add_executable(lidar src/lidar.cpp)
add_dependencies(lidar ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES} teaserpp::teaser_registration gtsam)

add_executable(legged_kinematics src/legged_kinematics.cpp)
add_dependencies(legged_kinematics ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(legged_kinematics ${catkin_LIBRARIES} gtsam pinocchio::pinocchio)
