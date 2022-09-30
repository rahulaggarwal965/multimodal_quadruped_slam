#include "optimizer.h"
#include "utils.h"

Optimizer::Optimizer() 
    : transform_listener(transform_buffer),
      imu(nh)
{
    trajectory_pub  = this->nh.advertise<nav_msgs::Path>("trajectory", 1);
}

// TODO:
// we need to optimize at some point. I am thinking
// that when we receive an exteroceptive factor (via lidar/camera)
// we take the latest imu_factor in the threaded queue 

void Optimizer::optimize(int steps) {

    this->isam.update(this->graph, this->initial_estimate);
    for (int i = 0; i < steps; i++) {
        this->isam.update();
    }

    this->current_poses = this->isam.calculateEstimate();
    
    this->graph = gtsam::NonlinearFactorGraph();
    this->initial_estimate.clear();

    this->publish_trajectory();

    // TODO(Rahul):
    // publish map_to_odom transform
    // requires listening for odom_to_base_link
}

void Optimizer::publish_trajectory() {
    nav_msgs::Path trajectory;
    trajectory.header.frame_id = "map";
    for (const auto &pose : this->current_poses) {
        trajectory.poses.push_back(to_pose_stamped_message(pose.value.cast<gtsam::Pose3>(), "map"));
    } 

    trajectory.header.stamp = ros::Time::now();
    this->trajectory_pub.publish(trajectory);
}
