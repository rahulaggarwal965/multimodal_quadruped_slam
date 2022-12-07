#include <pinocchio/fwd.hpp>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <unitree_legged_msgs/LowState.h>

#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

#include <Eigen/Dense>

#include <algorithm>

// gtsam redefines boost::serialization::... as well
#define __pinocchio_serialization_eigen_matrix_hpp__
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "utils.h"
#include "quadruped_slam/ForwardKinematicFactorStamped.h"
#include "quadruped_slam/RigidContactFactorStamped.h"
#include "quadruped_slam/ForwardKinematicChainStamped.h"
#include "quadruped_slam/RigidContactChainStamped.h"

struct Leg {
    Eigen::Vector3d q = {0, 0, 0}; //joint angles
    bool in_contact = false;
    double t = 0.0;
};

struct LeggedKinematics {

    std::string base_link_frame;
    std::string urdf_file_name;

    Leg FL, FR, RL, RR;

    pinocchio::Model model;
    pinocchio::Data data;

    ros::NodeHandle nh;

    // joint state
    ros::Subscriber leg_state_sub;

    ros::Publisher forward_kinematic_factor_pub;
    ros::Publisher rigid_contact_factor_pub;
    ros::Publisher fkc_pub;
    ros::Publisher rcc_pub;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener tranform_listener;

    gtsam::Matrix33 encoder_covariance;

    LeggedKinematics();

    void handle_low_state(const unitree_legged_msgs::LowStateConstPtr &low_state);

};

LeggedKinematics::LeggedKinematics() 
    : tranform_listener(transform_buffer)
{

    this->nh.getParam("/base_link_frame", this->base_link_frame);
    this->nh.getParam("/legged_kinematics/urdf_file_name", this->urdf_file_name);

    pinocchio::urdf::buildModel(this->urdf_file_name, this->model);
    this->data = pinocchio::Data(this->model);

    ros::Duration{1.0}.sleep();

    this->encoder_covariance = vector_from_param<3>(this->nh, "/legged_kinematics/motor_encoder_variances").asDiagonal();

    // TODO(Rahul): make topics params as in the IMU PR
    leg_state_sub = nh.subscribe<unitree_legged_msgs::LowState>("/a1_gazebo/lowState/state", 1, &LeggedKinematics::handle_low_state, this, ros::TransportHints().tcpNoDelay());

    this->forward_kinematic_factor_pub = nh.advertise<quadruped_slam::ForwardKinematicFactorStamped>("/legged_kinematics/forward_kinematic_factor", 1);
    this->rigid_contact_factor_pub = nh.advertise<quadruped_slam::RigidContactFactorStamped>("/legged_kinematics/rigid_contact_factor", 1);

    this->fkc_pub = nh.advertise<quadruped_slam::ForwardKinematicChainStamped>("/legged_kinematics/fkc", 1);
    this->rcc_pub = nh.advertise<quadruped_slam::RigidContactChainStamped>("/legged_kinematics/rcc", 1);
}

// 1. if leg transitions from no contact to contact at time t
// 2. introduce forward kinematic factor at time t
//      a. Get leg jacobian from pinocchio (6x3)
//      b. Get contact pose from tf
// 3. publish forward kinematic factor with timestamp
// 4. if same leg transitions from contact to no contact at time t
// 5. Do steps 2 to 3
// 6. Also broadcast a contact pose factor (zero change, increased noise)
void LeggedKinematics::handle_low_state(const unitree_legged_msgs::LowStateConstPtr &low_state) {

    const ros::Time t = ros::Time::now();
    const double current_time = t.toSec();

    /* gtsam::Vector3 FR_q = {low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q}; */
    /* gtsam::Vector3 FL_q = {low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q}; */
    /* gtsam::Vector3 RR_q = {low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q}; */
    /* gtsam::Vector3 RL_q = {low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q}; */
    
    bool FR_in_contact = min((int) low_state->footForce[0], 1);
    /* bool FL_in_contact = min((int) low_state->footForce[1], 1); */
    /* bool RR_in_contact = min((int) low_state->footForce[2], 1); */
    /* bool RL_in_contact = min((int) low_state->footForce[3], 1); */

    if (!FR.in_contact && FR_in_contact) {
        printf("[Legged_Kinematics] no contact -> contact at {%f s}, dt: {%f s}\n", current_time, current_time - FR.t);
        if (current_time - FR.t > 1e-2) {

            quadruped_slam::ForwardKinematicChainStamped fkc_msg;
            fkc_msg.header.stamp = t;
            fkc_msg.forward_kinematic_chain.hip = to_pose_message(from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_thigh"));
            fkc_msg.forward_kinematic_chain.thigh = to_pose_message(from_tf_tree(this->transform_buffer, "FR_thigh", "FR_calf"));
            fkc_msg.forward_kinematic_chain.calf = to_pose_message(from_tf_tree(this->transform_buffer, "FR_calf", "FR_foot"));

            fkc_pub.publish(fkc_msg);

            /* const gtsam::Pose3 FR_transform = from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_foot");  */
            /**/
            /**/
            /* gtsam::Vector12 q; */
            /* q << low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q,  */
            /*      low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q, */
            /*      low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q, */
            /*      low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q; */
            /* Eigen::Matrix<double, 6, 12> J{}; */
            /* J.setZero(); */
            /* pinocchio::forwardKinematics(this->model, this->data, q); */
            /* pinocchio::updateFramePlacements(this->model, this->data); */
            /* pinocchio::computeJointJacobians(this->model, this->data, q); */
            /**/
            /* const auto fr_foot_id = this->model.getFrameId("FR_foot"); */
            /* pinocchio::getFrameJacobian(this->model, this->data, fr_foot_id, pinocchio::LOCAL, J); */
            /* Eigen::Matrix<double, 6, 3> joint_J = J.block<6, 3>(0, 3); */
            /* gtsam::Matrix66 cov = joint_J * this->encoder_covariance * joint_J.transpose(); */
            /**/
            /* quadruped_slam::ForwardKinematicFactorStamped fk_msg; */
            /* fk_msg.header.stamp = t; */
            /* fk_msg.forward_kinematic_factor.id = 1; // FR */
            /* fk_msg.forward_kinematic_factor.contact_pose = to_pose_message(FR_transform); */
            /* for (int i = 0; i < 36; i++) { */
            /*     // NOTE(Rahul): column major (for now) */
            /*     if (i % 6 == 0) { */
            /*         fk_msg.forward_kinematic_factor.encoder_noise[i] = fmax(cov(i), 1e-3); */
            /*     } else { */
            /*         fk_msg.forward_kinematic_factor.encoder_noise[i] = cov(i); */
            /*     } */
            /* } */
            /* this->forward_kinematic_factor_pub.publish(fk_msg); */
            FR.in_contact = FR_in_contact;
            FR.t = current_time;
        }
    }

    // @ArbitraryParameter 0.3 sec
    if ((FR.in_contact && !FR_in_contact) || (FR.t != 0.0 && current_time - FR.t >= 0.3)) {
        printf("[Legged_Kinematics] contact -> no contact at {%f s}, dt: {%f s}\n", current_time, current_time - FR.t);
        if (current_time - FR.t > 1e-2) {

            quadruped_slam::RigidContactChainStamped rcc_msg;
            rcc_msg.header.stamp = t;
            rcc_msg.rcc.fkc.hip = to_pose_message(from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_thigh"));
            rcc_msg.rcc.fkc.thigh = to_pose_message(from_tf_tree(this->transform_buffer, "FR_thigh", "FR_calf"));
            rcc_msg.rcc.fkc.calf = to_pose_message(from_tf_tree(this->transform_buffer, "FR_calf", "FR_foot"));

            rcc_pub.publish(rcc_msg);
            /* const gtsam::Pose3 FR_transform = from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_foot");  */
            /* gtsam::Vector12 q; */
            /* q << low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q,  */
            /*      low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q, */
            /*      low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q, */
            /*      low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q; */
            /* Eigen::Matrix<double, 6, 12> J{}; */
            /* J.setZero(); */
            /* pinocchio::framesForwardKinematics(this->model, this->data, q); */
            /* pinocchio::computeJointJacobians(this->model, this->data, q); */
            /**/
            /* auto fr_foot_id = this->model.getFrameId("FR_foot"); */
            /* pinocchio::getFrameJacobian(this->model, this->data, fr_foot_id, pinocchio::LOCAL, J); */
            /* Eigen::Matrix<double, 6, 3> joint_J = J.block<6, 3>(0, 3); */
            /* gtsam::Matrix66 cov = joint_J * this->encoder_covariance * joint_J.transpose(); */
            /**/
            /* quadruped_slam::RigidContactFactorStamped rc_msg; */
            /* rc_msg.header.stamp = t; */
            /**/
            /* auto &rc = rc_msg.rigid_contact_factor; */
            /**/
            /* rc.forward_kinematic_factor.id = 1; // FR */
            /* rc.forward_kinematic_factor.contact_pose = to_pose_message(FR_transform); */
            /* for (int i = 0; i < 36; i++) { */
            /*     rc.forward_kinematic_factor.encoder_noise[i] = cov(i); // NOTE(Rahul): column major (for now) */
            /* } */
            /**/
            /* // TODO(rahul): refactor */
            /* gtsam::Vector3 contact_angular_velocity_variances = {1e-3, 1e-3, 1e-3}; */
            /* gtsam::Vector3 contact_linear_velocity_variances = {1e-6, 1e-6, 1e-6}; */
            /**/
            /* rc.contact_noise = {0}; */
            /* for (int i = 0; i < 3; i++) { */
            /*     rc.contact_noise[6 * i + i] = contact_angular_velocity_variances[i] * (FR.t - current_time); */
            /* } */
            /* for (int i = 0; i < 3; i++) { */
            /*     rc.contact_noise[6 * (i + 3) + i] = contact_linear_velocity_variances[i] * (FR.t - current_time); */
            /* } */
            /**/
            /* this->rigid_contact_factor_pub.publish(rc_msg); */
            FR.in_contact = FR_in_contact;
            FR.t = current_time;
        }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "legged_kinematics");

    LeggedKinematics lk;

    ros::spin();
    return 0;
}
