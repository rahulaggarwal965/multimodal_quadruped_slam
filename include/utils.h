#pragma once

#include <ros/time.h>

#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>

// STYLE(Rahul): put this stuff into a namespace

// Convert a gtsam Pose2 into a ros PoseStamped
inline geometry_msgs::PoseStamped to_pose_stamped(const gtsam::Pose3 &pose, const std::string &frame) {

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

inline gtsam::Pose3 from_tf(const geometry_msgs::TransformStamped &transform) {
    gtsam::Pose3 pose{
        gtsam::Rot3{transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z},
        gtsam::Vector3{transform.transform.translation.x,
               transform.transform.translation.y,
               transform.transform.translation.z}
        };
    return pose;
}

inline gtsam::Pose3 from_tf_tree(tf2_ros::Buffer &buffer, const std::string &parent_frame, const std::string &child_frame) {
    // gets the transform T of the child_frame in the parent_frame
    // e.g if a point p is (0, 0, 0) in the child_frame, then the point is Tp in the parent frame
    const geometry_msgs::TransformStamped transform = buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
    return from_tf(transform);
}
