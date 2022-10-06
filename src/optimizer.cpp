#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "optimizer.h"
#include "utils.h"

using gtsam::symbol_shorthand::X;

Optimizer::Optimizer() 
    : transform_listener(transform_buffer),
      imu(nh)
{
    this->nh.getParam("map_frame", this->map_frame);
    this->nh.getParam("base_link_frame", this->base_link_frame);
    this->nh.getParam("odom_frame", this->odom_frame);

    trajectory_pub  = this->nh.advertise<nav_msgs::Path>("trajectory", 1);

    std::vector<double> prior_pose_sigmas;
    this->nh.getParam("optimizer/prior_pose_sigmas", prior_pose_sigmas);

    // We always start at 0
    this->graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3{}, gtsam::Vector{prior_pose_sigmas.data()}.asDiagonal()));
    this->initial_estimate.insert(X(0), gtsam::Pose3{});
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
    trajectory.header.frame_id = this->map_frame;
    for (const auto &pose : this->current_poses) {
        trajectory.poses.push_back(to_pose_stamped_message(pose.value.cast<gtsam::Pose3>(), this->map_frame));
    } 

    trajectory.header.stamp = ros::Time::now();
    this->trajectory_pub.publish(trajectory);
}
