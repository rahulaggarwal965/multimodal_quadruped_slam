#include "optimizer.h"
#include "utils.h"

void Optimizer::optimize(int steps) {

    this->isam.update(this->graph, this->initial_estimate);
    for (int i = 0; i < steps; i++) {
        this->isam.update();
    }

    this->current_poses = this->isam.calculateEstimate();
    
    this->graph = gtsam::NonlinearFactorGraph();
    this->initial_estimate.clear();
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
