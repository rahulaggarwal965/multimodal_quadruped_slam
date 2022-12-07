#pragma once

#include <ros/time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <gtsam/geometry/Pose3.h>

// STYLE(Rahul): put this stuff into a namespace

inline geometry_msgs::Pose to_pose_message(const gtsam::Pose3 &pose) {
    geometry_msgs::Pose pose_message;

    pose_message.position.x = pose.x();
    pose_message.position.y = pose.y();
    pose_message.position.z = pose.z();

    const gtsam::Quaternion q = pose.rotation().toQuaternion();

    pose_message.orientation.x = q.x();
    pose_message.orientation.y = q.y();
    pose_message.orientation.z = q.z();
    pose_message.orientation.w = q.w();

    return pose_message;
}

inline gtsam::Pose3 from_pose_message(const geometry_msgs::Pose &pose_message) {
    gtsam::Pose3 pose{
        gtsam::Rot3{pose_message.orientation.w,
                    pose_message.orientation.x,
                    pose_message.orientation.y,
                    pose_message.orientation.z},
        gtsam::Vector3{pose_message.position.x,
                       pose_message.position.y,
                       pose_message.position.z}
        };
    return pose;
} 

inline gtsam::Pose3 from_pose_stamped_message(const geometry_msgs::PoseStamped &pose_stamped_message) {
    return from_pose_message(pose_stamped_message.pose);
}

// Convert a gtsam Pose2 into a ros PoseStamped
inline geometry_msgs::PoseStamped to_pose_stamped_message(const gtsam::Pose3 &pose, const std::string &frame) {
    geometry_msgs::PoseStamped pose_stamped_message;

    pose_stamped_message.pose = to_pose_message(pose);
    pose_stamped_message.header.frame_id = frame;
    pose_stamped_message.header.stamp = ros::Time::now();

    return pose_stamped_message;
}

inline geometry_msgs::Transform to_transform_message(const gtsam::Pose3 &pose) {

    geometry_msgs::Transform transform_message;

    transform_message.translation.x = pose.x();
    transform_message.translation.y = pose.y();
    transform_message.translation.z = pose.z();

    const gtsam::Quaternion q = pose.rotation().toQuaternion();

    transform_message.rotation.x = q.x();
    transform_message.rotation.y = q.y();
    transform_message.rotation.z = q.z();
    transform_message.rotation.w = q.w();

    return transform_message;
}

inline geometry_msgs::TransformStamped to_transform_stamped_message(const gtsam::Pose3 &pose, const std::string &parent_frame, const std::string &child_frame) {
    geometry_msgs::TransformStamped transform_stamped_message;

    transform_stamped_message.transform = to_transform_message(pose);

    transform_stamped_message.header.stamp = ros::Time::now();
    transform_stamped_message.header.frame_id = parent_frame;
    transform_stamped_message.child_frame_id = child_frame;
    
    return transform_stamped_message;

}

inline void to_tf_tree(tf2_ros::TransformBroadcaster &broadcaster, const gtsam::Pose3 &pose, const std::string &parent_frame, const std::string &child_frame) {
    broadcaster.sendTransform(to_transform_stamped_message(pose, parent_frame, child_frame));
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

inline gtsam::Pose3 from_tf_tree(tf2_ros::Buffer &buffer, const std::string &parent_frame, const std::string &child_frame, const ros::Duration timeout = ros::Duration{0}) {
    // gets the transform T of the child_frame in the parent_frame
    // e.g if a point p is (0, 0, 0) in the child_frame, then the point is Tp in the parent frame
    const geometry_msgs::TransformStamped transform = buffer.lookupTransform(parent_frame, child_frame, ros::Time(0), timeout);
    return from_tf(transform);
}

template<int s>
Eigen::Matrix<double, s, 1> vector_from_param(const ros::NodeHandle &nh, const std::string &param_name) {
    std::vector<double> v;
    nh.getParam(param_name, v);
    return Eigen::Matrix<double, s, 1>(v.data());
}


