#include <fstream>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <ios>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>

#include "optimizer.h"
#include "utils.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

using gtsam::symbol_shorthand::H;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::C;

Optimizer::Optimizer() 
    : transform_listener(transform_buffer),
      imu(nh)
{
    this->nh.getParam("/map_frame", this->map_frame);
    this->nh.getParam("/base_link_frame", this->base_link_frame);
    this->nh.getParam("/odom_frame", this->odom_frame);

    trajectory_pub = this->nh.advertise<nav_msgs::Path>("trajectory", 1);
    pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    forward_kinematic_factor_sub = this->nh.subscribe<quadruped_slam::ForwardKinematicFactorStamped>("/legged_kinematics/forward_kinematic_factor", 1, &Optimizer::handle_forward_kinematic_factor, this);
    rigid_contact_factor_sub = this->nh.subscribe<quadruped_slam::RigidContactFactorStamped>("/legged_kinematics/rigid_contact_factor", 1, &Optimizer::handle_rigid_contact_factor, this);

    fkc_sub = this->nh.subscribe<quadruped_slam::ForwardKinematicChainStamped>("/legged_kinematics/fkc", 1, &Optimizer::handle_fkc, this);
    rcc_sub = this->nh.subscribe<quadruped_slam::RigidContactChainStamped>("/legged_kinematics/rcc", 1, &Optimizer::handle_rcc, this);

    // We always start at 0
    this->graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3{}, vector_from_param<6>(nh, "/optimizer/prior_pose_sigmas").asDiagonal()));
    this->initial_estimates.insert(X(0), gtsam::Pose3{});

    this->graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3{},
        vector_from_param<3>(nh, "/optimizer/prior_vel_sigmas").asDiagonal()));
    this->initial_estimates.insert(V(0), gtsam::Vector3{});


    this->graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), imu.prev_bias));
    this->initial_estimates.insert(B(0), imu.prev_bias);
}

void Optimizer::optimize(int steps) {

    try {
        this->isam.update(this->graph, this->initial_estimates);
        for (int i = 0; i < steps; i++) {
            this->isam.update();
        }
    } catch(const gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("%s\n", e.what());
        std::ofstream f("/home/infinity/Documents/graph.gv", std::ios_base::out);
        this->graph.saveGraph(f);
        f.close();
    }

    try {
        this->current_state = this->isam.calculateEstimate();
    } catch(const gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("%s\n", e.what());
        /* this->isam.saveGraph("/home/infinity/Documents/graph.gv"); */
        const auto &graph  = this->isam.getFactorsUnsafe();

        std::ofstream f("/home/infinity/Documents/graph.gv");
        graph.saveGraph(f);
        f.close();

        printf("IMU RESET? %d\n", imu.reset);

        this->current_state.insert(this->initial_estimates);
        printf("==== FACTOR ERRORS ====\n");
        graph.printErrors(this->current_state, "NonlinearFactorGraph: ", gtsam::DefaultKeyFormatter, [](const gtsam::Factor *f, double err, size_t s) {return err > 1.0f;});
        printf("==== TOTAL ERROR ====\n");
        printf("%f\n", graph.error(this->current_state));

        exit(1);
    }
    
    this->graph = gtsam::NonlinearFactorGraph();
    this->initial_estimates.clear();

    this->publish_trajectory();

    const gtsam::Pose3 current_pose = this->current_state.at<gtsam::Pose3>(X(state_index));
    this->pose_pub.publish(to_pose_stamped_message(current_pose, this->map_frame));
    

    const gtsam::Pose3 map_to_odom = current_pose.compose(imu.current_state.pose().inverse());
    to_tf_tree(this->transform_broadcaster, map_to_odom, this->map_frame, this->odom_frame);
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
    const gtsam::Pose3 &relative_contact_pose = from_pose_message(forward_kinematic_factor->forward_kinematic_factor.contact_pose);
    const Eigen::Matrix<double, 6, 6> encoder_noise_matrix{forward_kinematic_factor->forward_kinematic_factor.encoder_noise.data()};
    
    IMUFactor imu_factor = imu.create_factor(state_index - 1, state_index);
    this->graph.add(imu_factor.factor);
    this->initial_estimates.insert(X(state_index), imu_factor.pose_estimate);
    this->initial_estimates.insert(V(state_index), imu_factor.velocity_estimate);
    /* this->initial_estimates.insert(B(state_index), imu_factor.bias_estimate); */

    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3);
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    // TODO(Rahul): initial estimates (linearization points) are ideally seeded by lidar odometry, not IMU, because biases are so unstable
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(state_index), C(state_index), relative_contact_pose, noise);
    /* this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(state_index), C(state_index), relative_contact_pose, gtsam::noiseModel::Gaussian::Covariance(encoder_noise_matrix)); */
    this->initial_estimates.insert(C(state_index), imu_factor.pose_estimate.transformPoseFrom(relative_contact_pose));

    this->optimize();

    this->imu.reset_integration(
            current_state.at<gtsam::imuBias::ConstantBias>(B(0)),
            current_state.at<gtsam::Pose3>(X(state_index)),
            current_state.at<gtsam::Vector3>(V(state_index)));

    state_index += 1;

}

void Optimizer::handle_fkc(const quadruped_slam::ForwardKinematicChainStampedConstPtr &fkc) {
    IMUFactor imu_factor = imu.create_factor(state_index - 1, state_index);
    this->graph.add(imu_factor.factor);
    this->initial_estimates.insert(X(state_index), imu_factor.pose_estimate);
    this->initial_estimates.insert(V(state_index), imu_factor.velocity_estimate);

    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3);
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    const auto base_T_thigh = from_pose_message(fkc->forward_kinematic_chain.hip);
    const auto thigh_T_calf = from_pose_message(fkc->forward_kinematic_chain.thigh);
    const auto calf_T_foot = from_pose_message(fkc->forward_kinematic_chain.calf);

    // no noise assumptions
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(state_index), H(state_index), base_T_thigh, noise);
    auto pose = imu_factor.pose_estimate.compose(base_T_thigh);
    this->initial_estimates.insert(H(state_index), pose);
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(H(state_index), T(state_index), thigh_T_calf, noise);
    pose = pose.compose(thigh_T_calf);
    this->initial_estimates.insert(T(state_index), pose);
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(T(state_index), C(state_index), calf_T_foot, noise);
    pose = pose.compose(calf_T_foot);
    this->initial_estimates.insert(C(state_index), pose);
    
    this->optimize();

    this->imu.reset_integration(
            current_state.at<gtsam::imuBias::ConstantBias>(B(0)),
            current_state.at<gtsam::Pose3>(X(state_index)),
            current_state.at<gtsam::Vector3>(V(state_index)));

    state_index += 1;
}

void Optimizer::handle_rigid_contact_factor(const quadruped_slam::RigidContactFactorStampedConstPtr &rigid_contact_factor) {
    const auto &forward_kinematic_factor = rigid_contact_factor->rigid_contact_factor.forward_kinematic_factor;

    const gtsam::Pose3 &relative_contact_pose = from_pose_message(forward_kinematic_factor.contact_pose);
    const Eigen::Matrix<double, 6, 6> encoder_noise_matrix{forward_kinematic_factor.encoder_noise.data()};
    const Eigen::Matrix<double, 6, 6> contact_pose_covariance{rigid_contact_factor->rigid_contact_factor.contact_noise.data()};
    
    IMUFactor imu_factor = imu.create_factor(state_index - 1, state_index);
    this->graph.add(imu_factor.factor);
    this->initial_estimates.insert(X(state_index), imu_factor.pose_estimate);
    this->initial_estimates.insert(V(state_index), imu_factor.velocity_estimate);
    /* this->initial_estimates.insert(B(state_index), imu_factor.bias_estimate); */

    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3);
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(state_index), C(state_index), relative_contact_pose, noise);

    // @Robustness(Rahul): is this really a correct way of doing this, where should we linearize? By the forward kinematic factor or the contact pose?
    this->initial_estimates.insert(C(state_index), imu_factor.pose_estimate.transformPoseFrom(relative_contact_pose));

    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(C(state_index), C(state_index - 1), gtsam::Pose3::identity(), noise);
    /* this->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(C(state_index), C(state_index - 1), gtsam::Pose3::identity(),  */
    /*             gtsam::noiseModel::Gaussian::Covariance(contact_pose_covariance))); */

    this->optimize();

    this->imu.reset_integration(
            current_state.at<gtsam::imuBias::ConstantBias>(B((0))),
            current_state.at<gtsam::Pose3>(X(state_index)),
            current_state.at<gtsam::Vector3>(V(state_index)));

    state_index += 1;

}

void Optimizer::handle_rcc(const quadruped_slam::RigidContactChainStampedConstPtr &rcc) {
    IMUFactor imu_factor = imu.create_factor(state_index - 1, state_index);
    this->graph.add(imu_factor.factor);
    this->initial_estimates.insert(X(state_index), imu_factor.pose_estimate);
    this->initial_estimates.insert(V(state_index), imu_factor.velocity_estimate);

    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3);
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    const auto base_T_thigh = from_pose_message(rcc->rcc.fkc.hip);
    const auto thigh_T_calf = from_pose_message(rcc->rcc.fkc.thigh);
    const auto calf_T_foot = from_pose_message(rcc->rcc.fkc.calf);

    // no noise assumptions
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(state_index), H(state_index), base_T_thigh, noise);
    auto pose = imu_factor.pose_estimate.compose(base_T_thigh);
    this->initial_estimates.insert(H(state_index), pose);
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(H(state_index), T(state_index), thigh_T_calf, noise);
    pose = pose.compose(thigh_T_calf);
    this->initial_estimates.insert(T(state_index), pose);
    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(T(state_index), C(state_index), calf_T_foot, noise);
    pose = pose.compose(calf_T_foot);
    this->initial_estimates.insert(C(state_index), pose);

    this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(C(state_index), C(state_index - 1), gtsam::Pose3::identity(), noise);
    
    this->optimize();

    this->imu.reset_integration(
            current_state.at<gtsam::imuBias::ConstantBias>(B(0)),
            current_state.at<gtsam::Pose3>(X(state_index)),
            current_state.at<gtsam::Vector3>(V(state_index)));

    state_index += 1;
}
