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

#define __pinocchio_serialization_eigen_matrix_hpp__
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "utils.h"
#include "quadruped_slam/ForwardKinematicFactorStamped.h"
#include "quadruped_slam/RigidContactFactorStamped.h"


// needs to be 6x3
/* gtsam::Matrix33 compute_leg_jacobian(const gtsam::Vector3 &q, const gtsam::Vector4 &l) { */
/*  */
/*     // q = joint angles */
/*     // l = limb lengths */
/*  */
/*     // TODO(rahul): what do these represent */
/*     // l1 = abadLinkLength; */
/*     // l2 = hipLinkLength; */
/*     // l3 = kneeLinkLength; */
/*     // l4 = kneeLinkY_offset; */
/*  */
/*     const auto s1 = std::sin( q[0]); */
/*     const auto s2 = std::sin(-q[1]); */
/*     const auto s3 = std::sin(-q[2]); */
/*  */
/*     const auto c1 = std::cos( q[0]); */
/*     const auto c2 = std::cos(-q[1]); */
/*     const auto c3 = std::cos(-q[2]); */
/*  */
/*     const auto c23 = c2 * c3 - s2 * s3; */
/*     const auto s23 = s2 * c3 + c2 * s3; */
/*  */
/*     gtsam::Matrix33 J { */
/*         {0, -l[2] * c23 - l[1] * c2,  -l[2] * c23}, */
/*         {}, */
/*         {}, */
/*     } */
/*  */
/*     if (J) { */
/*         J->operator()(0, 0) = 0; */
/*         J->operator()(0, 1) = -l3 * c23 - l2 * c2; */
/*         J->operator()(0, 2) = -l3 * c23; */
/*         J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1; */
/*         J->operator()(1, 1) = l3 * s1 * s23 + l2 * s1 * s2; */
/*         J->operator()(1, 2) = l3 * s1 * s23; */
/*         J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1; */
/*         J->operator()(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2; */
/*         J->operator()(2, 2) = -l3 * c1 * s23; */
/*     } */
/* } */

struct Leg {
    Eigen::Vector3d q = {0, 0, 0}; //joint angles
    bool in_contact = false;
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
    leg_state_sub = nh.subscribe<unitree_legged_msgs::LowState>("/a1_gazebo/lowState/state", 1, &LeggedKinematics::handle_low_state, this);

    this->forward_kinematic_factor_pub = nh.advertise<quadruped_slam::ForwardKinematicFactorStamped>("/legged_kinematics/forward_kinematic_factor", 1);
    this->rigid_contact_factor_pub = nh.advertise<quadruped_slam::RigidContactFactorStamped>("/legged_kinematics/rigid_contact_factor", 1);
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

    /* gtsam::Vector3 FR_q = {low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q}; */
    /* gtsam::Vector3 FL_q = {low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q}; */
    /* gtsam::Vector3 RR_q = {low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q}; */
    /* gtsam::Vector3 RL_q = {low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q}; */
    
    bool FR_in_contact = min((int) low_state->footForce[0], 1);
    /* bool FL_in_contact = min((int) low_state->footForce[1], 1); */
    /* bool RR_in_contact = min((int) low_state->footForce[2], 1); */
    /* bool RL_in_contact = min((int) low_state->footForce[3], 1); */

    if (!FR.in_contact && FR_in_contact) {
        std::cout << "FR contact" << '\n';
        gtsam::Pose3 FR_transform = from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_foot"); 
        gtsam::Vector12 q;
        q << low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q, 
             low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q,
             low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q,
             low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q;
        Eigen::Matrix<double, 6, 12> J{};
        J.setZero();
        pinocchio::computeJointJacobian(this->model, this->data, q, 6, J);
        Eigen::Matrix<double, 6, 3> joint_J = J.block<6, 3>(0, 3);
        gtsam::Matrix66 cov = joint_J * this->encoder_covariance * joint_J.transpose();

        quadruped_slam::ForwardKinematicFactorStamped fk_msg;
        fk_msg.header.stamp = ros::Time::now();
        fk_msg.forward_kinematic_factor.id = 1; // FR
        fk_msg.forward_kinematic_factor.contact_pose = to_pose_message(FR_transform);
        for (int i = 0; i < 36; i++) {
            fk_msg.forward_kinematic_factor.encoder_noise[i] = cov(i); // NOTE(Rahul): column major (for now)
        }
        this->forward_kinematic_factor_pub.publish(fk_msg);

        FR.in_contact = FR_in_contact;
    }

    if (FR.in_contact && !FR_in_contact) {
        std::cout << "Lost FR contact" << '\n';
        gtsam::Pose3 FR_transform = from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_foot"); 
        gtsam::Vector12 q;
        q << low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q, 
             low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q,
             low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q,
             low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q;
        Eigen::Matrix<double, 6, 12> J{};
        J.setZero();
        pinocchio::computeJointJacobian(this->model, this->data, q, 6, J);
        Eigen::Matrix<double, 6, 3> joint_J = J.block<6, 3>(0, 3);
        gtsam::Matrix66 cov = joint_J * this->encoder_covariance * joint_J.transpose();

        quadruped_slam::RigidContactFactorStamped rc_msg;
        rc_msg.header.stamp = ros::Time::now();

        auto &rc = rc_msg.rigid_contact_factor;

        rc.forward_kinematic_factor.id = 1; // FR
        rc.forward_kinematic_factor.contact_pose = to_pose_message(FR_transform);
        for (int i = 0; i < 36; i++) {
            rc.forward_kinematic_factor.encoder_noise[i] = cov(i); // NOTE(Rahul): column major (for now)
        }

        for (int i = 0; i < 36; i++) {
            // TODO(rahul): keep track of time and param multiply
            rc.contact_noise[i] = 0;
        }

        this->rigid_contact_factor_pub.publish(rc_msg);

        FR.in_contact = FR_in_contact;

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "legged_kinematics");

    LeggedKinematics lk;

    ros::spin();
    return 0;
}
