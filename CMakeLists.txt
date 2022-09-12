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

add_message_files(
    DIRECTORY msg
    FILES
    segmentation_info.msg
    plane.msg
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

add_executable(lidar_tracking src/lidar_tracking.cpp)
target_link_libraries(lidar_tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(feature_extraction src/feature_extraction.cpp)
target_link_libraries(feature_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(visualization src/visualization.cpp)
target_link_libraries(visualization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
