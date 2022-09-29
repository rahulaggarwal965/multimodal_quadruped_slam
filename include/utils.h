#pragma once

#include <ros/time.h>

#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Quaternion.h>

#include <gtsam/geometry/Pose3.h>

using Pose3 = gtsam::Pose3;

// Convert a gtsam Pose2 into a ros PoseStamped
inline geometry_msgs::PoseStamped generate_pose_message(Pose3 pose, std::string frame) {
    geometry_msgs::PoseStamped pose_message;

    pose_message.header.frame_id = frame;
    pose_message.header.stamp = ros::Time::now();
    pose_message.pose.position.x = pose.x();
    pose_message.pose.position.y = pose.y();
    pose_message.pose.position.z = pose.z();

    const gtsam::Quaternion q = pose.rotation().toQuaternion();

    pose_message.pose.orientation.x = q.x();
    pose_message.pose.orientation.y = q.y();
    pose_message.pose.orientation.z = q.z();
    pose_message.pose.orientation.w = q.w();

    return pose_message;
}
