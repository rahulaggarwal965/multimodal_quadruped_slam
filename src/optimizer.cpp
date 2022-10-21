#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "optimizer.h"
#include "utils.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;

Optimizer::Optimizer() 
    : transform_listener(transform_buffer),
      imu(nh)
{
    this->nh.getParam("/map_frame", this->map_frame);
    this->nh.getParam("/base_link_frame", this->base_link_frame);
    this->nh.getParam("/odom_frame", this->odom_frame);

    trajectory_pub = this->nh.advertise<nav_msgs::Path>("trajectory", 1);

    forward_kinematic_factor_pub = this->nh.subscribe<quadruped_slam::ForwardKinematicFactorStamped>("/legged_kinematics/forward_kinematic_factor", 1, &Optimizer::handle_forward_kinematic_factor, this);
    rigid_contact_factor_pub = this->nh.subscribe<quadruped_slam::RigidContactFactorStamped>("/legged_kinematics/rigid_contact_factor", 1, &Optimizer::handle_rigid_contact_factor, this);

    // We always start at 0
    this->graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3{}, vector_from_param<6>(nh, "/optimizer/prior_pose_sigmas").asDiagonal()));
    this->initial_estimates.insert(X(0), gtsam::Pose3{});

    this->graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3{},
        vector_from_param<3>(nh, "/optimizer/prior_vel_sigmas").asDiagonal()));
    this->initial_estimates.insert(V(0), gtsam::Vector3{});

    this->graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), imu.prior_imu_bias));
}

// TODO:
// we need to optimize at some point. I am thinking
// that when we receive an exteroceptive factor (via lidar/camera)
// we take the latest imu_factor in the threaded queue 

void Optimizer::optimize(int steps) {

    this->isam.update(this->graph, this->initial_estimates);
    for (int i = 0; i < steps; i++) {
        this->isam.update();
    }

    this->current_state = this->isam.calculateEstimate();
    
    this->graph = gtsam::NonlinearFactorGraph();
    this->initial_estimates.clear();

    this->publish_trajectory();

    // TODO(Rahul):
    // publish map_to_odom transform
    // requires listening for odom_to_base_link
}

void Optimizer::publish_trajectory() {
    nav_msgs::Path trajectory;
    trajectory.header.frame_id = this->map_frame;
    for (int i = 0; i < this->state_index; i++) {
        trajectory.poses.push_back(to_pose_stamped_message(current_state.at<gtsam::Pose3>(X(i)), map_frame));
    } 

    trajectory.header.stamp = ros::Time::now();
    this->trajectory_pub.publish(trajectory);
}

void Optimizer::handle_forward_kinematic_factor(const quadruped_slam::ForwardKinematicFactorStampedConstPtr &forward_kinematic_factor) {
    const gtsam::Pose3 &contact_pose = from_pose_message(forward_kinematic_factor->forward_kinematic_factor.contact_pose);
    const Eigen::Matrix<float, 6, 6> encoder_noise_matrix{forward_kinematic_factor->forward_kinematic_factor.encoder_noise.data()};
    

    IMUFactor imu_factor = imu.create_factor(state_index - 1, state_index);
    this->graph.add(imu_factor.factor);
    this->initial_estimates.at<gtsam::Pose3>(X(state_index)) = imu_factor.pose_estimate;
    this->initial_estimates.at<gtsam::Vector3>(V(state_index)) = imu_factor.velocity_estimate;
    this->initial_estimates.at<gtsam::imuBias::ConstantBias>(B(state_index)) = imu_factor.bias_estimate;

    // TODO(Rahul): initial estimates (linearization points) are ideally seeded by lidar odometry, not IMU, because biases are so unstable
    this->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(state_index), C(state_index), contact_pose, gtsam::noiseModel::Gaussian::Covariance(encoder_noise_matrix)));
    this->initial_estimates.at<gtsam::Pose3>(C(state_index)) = imu_factor.pose_estimate.transformPoseFrom(contact_pose);

    this->optimize();

    this->imu.reset_integration(
            current_state.at<gtsam::imuBias::ConstantBias>(B(state_index)),
            current_state.at<gtsam::Pose3>(X(state_index)),
            current_state.at<gtsam::Vector3>(V(state_index)));

    state_index += 1;

}

void Optimizer::handle_rigid_contact_factor(const quadruped_slam::RigidContactFactorStampedConstPtr &rigid_contact_factor) {

    const auto &forward_kinematic_factor = rigid_contact_factor->rigid_contact_factor.forward_kinematic_factor;

    const gtsam::Pose3 &contact_pose = from_pose_message(forward_kinematic_factor.contact_pose);
    const Eigen::Matrix<float, 6, 6> encoder_noise_matrix{forward_kinematic_factor.encoder_noise.data()};
    const Eigen::Matrix<float, 6, 6> contact_pose_covariance{rigid_contact_factor->rigid_contact_factor.contact_noise.data()};
    
    this->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(state_index), C(state_index), contact_pose, gtsam::noiseModel::Gaussian::Covariance(encoder_noise_matrix)));

    this->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(C(state_index), C(state_index - 1), gtsam::Pose3::identity(), 
                gtsam::noiseModel::Gaussian::Covariance(contact_pose_covariance)));

    state_index += 1;

}
