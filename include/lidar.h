#pragma once

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <gtsam/geometry/Pose3.h>

#include <quadruped_slam/lidar_factor.h>

#include "utils.h"

// OUSTER point type
struct OusterPointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (OusterPointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint16_t, ring, ring)
)

struct Lidar {
    
    std::string topic;

    std::string frame;
    std::string map_frame;
    std::string base_link_frame;
    std::string odom_frame;

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Subscriber state_sub;

    ros::Publisher lidar_factor_pub;
    ros::Publisher cloud_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
    ros::Time prev_cloud_timestamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    ros::Time current_cloud_timestamp;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    gtsam::Pose3 base_link_T_lidar;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_grid;
    fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;

    gtsam::Pose3 last_keyframe_pose;

    Lidar();
    void handle_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void handle_state(const geometry_msgs::PoseStampedConstPtr &state_msg);
};
