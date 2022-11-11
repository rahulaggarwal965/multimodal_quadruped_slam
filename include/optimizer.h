#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "nav_msgs/Path.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuFactor.h>

#include "quadruped_slam/ForwardKinematicFactorStamped.h"
#include "quadruped_slam/RigidContactFactorStamped.h"
#include "imu.h"

// NOTE(Rahul):
// might fold in IMU into this class to avoid complexity
struct Optimizer {

    std::string map_frame;
    std::string base_link_frame;
    std::string odom_frame;

    int state_index = 1;
    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimates;
    gtsam::Values current_state;

    ros::NodeHandle nh;

    // TODO(Rahul):
    // we include the IMU inside this class because it exchanges significant 
    // information between global optimization and IMU preintegration
    // perhaps it would be better to encapsulate IMU information inside a ros msg
    // but it is not clear how I would do that right now
    IMU imu;

    // Legged Kinematics
    ros::Subscriber forward_kinematic_factor_pub;
    ros::Subscriber rigid_contact_factor_pub;

    ros::Publisher trajectory_pub;
    ros::Publisher pose_pub;


    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    Optimizer();

    // TODO(rahul): think about when we want to actually optimize. Ideally, this
    // should not happen very often, preferably when we get a lidar/camera scan.
    void optimize(int steps = 0);

    void publish_trajectory();

    void handle_forward_kinematic_factor(const quadruped_slam::ForwardKinematicFactorStampedConstPtr &forward_kinematic_factor);
    void handle_rigid_contact_factor(const quadruped_slam::RigidContactFactorStampedConstPtr &rigid_contact_factor);

};
