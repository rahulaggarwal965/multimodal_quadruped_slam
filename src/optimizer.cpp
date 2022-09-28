#include <ros/ros.h>
#include <ros/time.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "nav_msgs/Path.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

#include "utils.h"

using Pose3 = gtsam::Pose3;

struct Optimizer {
    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;
    gtsam::Values current_poses;

    ros::NodeHandle nh;

    ros::Subscriber imu_sub;
    ros::Publisher imu_bias_pub;

    ros::Publisher trajectory_pub;


    tf::TransformListener transform_listener;
    tf::TransformBroadcaster transform_broadcaster;

    Optimizer() {
        
        /* imu_sub = nh.subscribe("imu", 5, &Optimizer::imu_callback, this); */
        trajectory_pub  = nh.advertise<nav_msgs::Path>("trajectory", 1);
    }

    /* void imu_callback() */

    // TODO(rahul): think about when we want to actually optimize. Ideally, this should not happen very often, preferably when we get a lidar/camera scan.
    void optimize(int steps = 1);

    void publish_trajectory();

};

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
        trajectory.poses.push_back(generate_pose_message(pose.value.cast<Pose3>(), "map"));
    } 

    trajectory.header.stamp = ros::Time::now();
    this->trajectory_pub.publish(trajectory);
}
