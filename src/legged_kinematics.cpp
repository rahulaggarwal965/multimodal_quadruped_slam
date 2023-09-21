#include <pinocchio/fwd.hpp>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <unitree_legged_msgs/LowState.h>
#include <geometry_msgs/WrenchStamped.h>

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

#include "quadruped_slam/ForwardKinematicFactor.h"
#include "quadruped_slam/ForwardKinematicFactorStamped.h"
#include "quadruped_slam/RigidContactFactor.h"

using quadruped_slam::ForwardKinematicFactorStamped;
using quadruped_slam::ForwardKinematicFactor;
using quadruped_slam::RigidContactFactor;

enum LegID {
    FL = 0,
    FR,
    RL,
    RR,
    NUM_LEGS,
    NONE,
};

std::string leg_names[] = {"FL", "FR", "RL", "RR"};
std::string topic_names[] = {"/visual/FL_foot_contact/the_force", "/visual/FR_foot_contact/the_force", "/visual/RL_foot_contact/the_force", "/visual/RR_foot_contact/the_force"};
std::string frame_names[] = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};

struct Leg {
    /* Eigen::Vector3d q = {0, 0, 0}; //joint angles */
    bool in_contact = false;
    double t = 0.0;
};

struct LeggedKinematics {

    std::string base_link_frame;
    std::string urdf_file_name;

    Leg legs[NUM_LEGS];
    LegID current_leg_id = NONE;
    gtsam::Pose3 ci_T_cj; // initialized as identity
    gtsam::Matrix33 current_rc_covariance;

    pinocchio::Model model;
    pinocchio::Data data;

    ros::NodeHandle nh;

    // joint state
    ros::Subscriber leg_state_sub;

    ros::Subscriber leg_state_subs[NUM_LEGS];
    /* ros::Subscriber FL_sub; */
    /* ros::Subscriber FR_sub; */
    /* ros::Subscriber RL_sub; */
    /* ros::Subscriber RR_sub; */

    ros::Publisher fk_pub;
    ros::Publisher rc_pub;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener tranform_listener;

    gtsam::Matrix33 fk_covariance;
    gtsam::Matrix33 rc_covariance;

    LeggedKinematics();

    void handle_low_state(const unitree_legged_msgs::LowStateConstPtr &low_state);
    void handle_leg_state(const geometry_msgs::WrenchStamped& msg, const LegID leg_id);

};

LeggedKinematics::LeggedKinematics() 
    : tranform_listener(transform_buffer)
{

    this->nh.getParam("/base_link_frame", this->base_link_frame);
    this->nh.getParam("/legged_kinematics/urdf_file_name", this->urdf_file_name);

    /* pinocchio::urdf::buildModel(this->urdf_file_name, this->model); */
    /* this->data = pinocchio::Data(this->model); */

    ros::Duration{1.0}.sleep();

    this->fk_covariance = vector_from_param<3>(this->nh, "/legged_kinematics/fk_variances").asDiagonal();

    // TODO(Rahul): make topics params as in the IMU PR
    leg_state_sub = nh.subscribe<unitree_legged_msgs::LowState>("/a1_gazebo/lowState/state", 1, &LeggedKinematics::handle_low_state, this, ros::TransportHints().tcpNoDelay());

    for (int i = 0; i < NUM_LEGS; i++) {
        leg_state_subs[i] = nh.subscribe<geometry_msgs::WrenchStamped>(topic_names[i], 1, boost::bind(&LeggedKinematics::handle_leg_state, this, _1, (LegID) i));
    }

    // Foward Kinematics
    this->fk_pub = nh.advertise<quadruped_slam::ForwardKinematicFactorStamped>("/legged_kinematics/forward_kinematic_factor", 1);

    // "Rigid Contact"
    this->rc_pub = nh.advertise<quadruped_slam::RigidContactFactor>("/legged_kinematics/rigid_contact_factor", 1);
}

void LeggedKinematics::handle_leg_state(const geometry_msgs::WrenchStamped &wrench_stamped_msg, const LegID leg_id) {
    const ros::Time t = ros::Time::now();
    const double current_time = t.toSec();

    bool leg_contact = min(wrench_stamped_msg.wrench.force.z, 1.0) >= 0;

    if (current_leg_id == NONE) {
        if (leg_contact) {

            this->current_rc_covariance = this->rc_covariance;

            this->current_leg_id = (LegID) leg_id;

            printf("[Legged Kinematics] initialized with Leg {%s} at {%f s}\n", leg_names[this->current_leg_id].c_str(), current_time);

            legs[this->current_leg_id] = {leg_contact, current_time};

            ForwardKinematicFactorStamped fk_stamped_msg;
            fk_stamped_msg.forward_kinematic_factor.contact_pose = to_pose_message(from_tf_tree(transform_buffer, base_link_frame, frame_names[current_leg_id]));
            Eigen::Map<Eigen::Matrix3d>(fk_stamped_msg.forward_kinematic_factor.covariance.data()) = fk_covariance;

            return;
        }
    }

    const Leg &current_leg = this->legs[this->current_leg_id]; // explicit copy
    /* legs[this->current_leg_id] = {leg_contact, current_time}; */

    const auto dt = current_time - current_leg.t;

    // out of contact event (must switch or publish)
    if (leg_id == current_leg_id && current_leg.in_contact && !leg_contact) {
        printf("[Legged Kinematics] Leg {%s} contact -> no contact at {%f s}, dt: {%f s}\n", leg_names[current_leg_id].c_str(), current_time, dt);


        if (dt >= 0.3) {
            const auto fk_pose = from_tf_tree(this->transform_buffer, this->base_link_frame, frame_names[current_leg_id]);

            ForwardKinematicFactor fk_msg;
            fk_msg.contact_pose = to_pose_message(fk_pose);
            Eigen::Map<Eigen::Matrix3d>(fk_msg.covariance.data()) = fk_covariance;

            RigidContactFactor rc_msg;
            rc_msg.contact_a_stamp = ros::Time(current_leg.t);
            rc_msg.contact_b_stamp = t;
            rc_msg.forward_kinematic_factor = fk_msg;
            rc_msg.pose = to_pose_message(ci_T_cj);
            Eigen::Map<Eigen::Matrix3d>(rc_msg.covariance.data()) = current_rc_covariance;

            rc_pub.publish(rc_msg);
        } else {
            legs[this->current_leg_id] = {leg_contact, current_time};
            // for now this will be greedy. Perhaps we should make this smarter (shift diagonally?)
            for (int i = 0; i < NUM_LEGS; i++) {
                if (legs[i].in_contact) {
                    // switch to new leg
                    // this new pose is the pose of the switched to leg in the pose of the previous leg
                    this->ci_T_cj = ci_T_cj.transformPoseFrom(from_tf_tree(transform_buffer, frame_names[current_leg_id], frame_names[i]));
                    this->current_rc_covariance += this->rc_covariance;
                    // should increase noise by another isotropic gaussian
                    this->current_leg_id = (LegID) i;
                    break;   
                }
            }
        }
    }


}

void LeggedKinematics::handle_low_state(const unitree_legged_msgs::LowStateConstPtr &low_state) {
    const ros::Time t = ros::Time::now();
    const double current_time = t.toSec();

    bool new_contact_status[NUM_LEGS] = {
        (bool) min((int) low_state->footForce[0], 1),
        (bool) min((int) low_state->footForce[1], 1),
        (bool) min((int) low_state->footForce[2], 1),
        (bool) min((int) low_state->footForce[3], 1)
    };

    // new contact event
    if (current_leg_id == NONE) {
        // for now this will be greedy. Perhaps we should make this smarter (shift diagonally?)
        for (int i = 0; i < NUM_LEGS; i++) {
            if (new_contact_status[i]) {
                // this new pose is the pose of the switched to leg in the pose of the previous leg
                this->current_rc_covariance = rc_covariance;
                // should increase noise by another isotropic gaussian
                this->current_leg_id = (LegID) i;
                break;   
            }
        }

        legs[current_leg_id] = {
            true, // in_contact
            current_time // first_contact_time
        };


        ForwardKinematicFactorStamped fk_stamped_msg;
        fk_stamped_msg.forward_kinematic_factor.contact_pose = to_pose_message(from_tf_tree(transform_buffer, base_link_frame, frame_names[current_leg_id]));
        Eigen::Map<Eigen::Matrix3d>(fk_stamped_msg.forward_kinematic_factor.covariance.data()) = fk_covariance;

    }

    const Leg &current_leg = this->legs[this->current_leg_id];
    const auto dt = current_time - current_leg.t;


    // out of contact event (must switch or publish)
    if (current_leg.in_contact && !new_contact_status[this->current_leg_id]) {
        printf("[Legged Kinematics] Leg {%s} contact -> no contact at {%f s}, dt: {%f s}\n", leg_names[current_leg_id].c_str(), current_time, dt);

        if (dt >= 0.3) {
            const auto fk_pose = from_tf_tree(this->transform_buffer, this->base_link_frame, frame_names[current_leg_id]);

            ForwardKinematicFactor fk_msg;
            fk_msg.contact_pose = to_pose_message(fk_pose);
            Eigen::Map<Eigen::Matrix3d>(fk_msg.covariance.data()) = fk_covariance;

            RigidContactFactor rc_msg;
            rc_msg.contact_a_stamp = ros::Time(current_leg.t);
            rc_msg.contact_b_stamp = t;
            rc_msg.forward_kinematic_factor = fk_msg;
            rc_msg.pose = to_pose_message(ci_T_cj);
            Eigen::Map<Eigen::Matrix3d>(rc_msg.covariance.data()) = current_rc_covariance;

            rc_pub.publish(rc_msg);
            // publish
        } else {
            // for now this will be greedy. Perhaps we should make this smarter (shift diagonally?)
            for (int i = 0; i < NUM_LEGS; i++) {
                if (new_contact_status[i]) {
                    // switch to new leg
                    // this new pose is the pose of the switched to leg in the pose of the previous leg
                    this->ci_T_cj = ci_T_cj.transformPoseFrom(from_tf_tree(transform_buffer, frame_names[current_leg_id], frame_names[i]));
                    this->current_rc_covariance += this->rc_covariance;
                    // should increase noise by another isotropic gaussian
                    this->current_leg_id = (LegID) i;
                    break;   
                }
            }

            legs[current_leg_id] = {
                true, // in_contact
                current_time // first_contact_time
            };
        }

    } else if (current_leg.in_contact && new_contact_status[current_leg_id]) {
        printf("[Legged Kinematics] Leg {%s} continued contact at {%f s}, dt: {%f s}\n", leg_names[current_leg_id].c_str(), current_time, dt);

        if (dt >= 0.3) {
            const auto fk_pose = from_tf_tree(transform_buffer, this->base_link_frame, frame_names[current_leg_id]);

            /* quadrfk_msg =  */
            /* fkc_pub.publish(to_pose_message()) */
        }

    }

    // new contact event
    if (!current_leg.in_contact && new_contact_status[this->current_leg_id]) {
        printf("[Legged Kinematics] Leg {%s} no contact -> contact at {%f s}, dt: {%f s}\n", leg_names[current_leg_id].c_str(), current_time, current_time - current_leg.t);



    }

}

// 1. if leg transitions from no contact to contact at time t
// 2. introduce forward kinematic factor at time t
//      a. Get leg jacobian from pinocchio (6x3)
//      b. Get contact pose from tf
// 3. publish forward kinematic factor with timestamp
// 4. if same leg transitions from contact to no contact at time t
// 5. Do steps 2 to 3
// 6. Also broadcast a contact pose factor (zero change, increased noise)
/* void LeggedKinematics::handle_low_state(const unitree_legged_msgs::LowStateConstPtr &low_state) { */
/**/
/*     const ros::Time t = ros::Time::now(); */
/*     const double current_time = t.toSec(); */

    /* gtsam::Vector3 FR_q = {low_state->motorState[0].q, low_state->motorState[1].q, low_state->motorState[2].q}; */
    /* gtsam::Vector3 FL_q = {low_state->motorState[3].q, low_state->motorState[4].q, low_state->motorState[5].q}; */
    /* gtsam::Vector3 RR_q = {low_state->motorState[6].q, low_state->motorState[7].q, low_state->motorState[8].q}; */
    /* gtsam::Vector3 RL_q = {low_state->motorState[9].q, low_state->motorState[10].q, low_state->motorState[11].q}; */
    
    /* bool FR_in_contact = min((int) low_state->footForce[0], 1); */
    /* bool FL_in_contact = min((int) low_state->footForce[1], 1); */
    /* bool RR_in_contact = min((int) low_state->footForce[2], 1); */
    /* bool RL_in_contact = min((int) low_state->footForce[3], 1); */

    /* if (!FR.in_contact && FR_in_contact) { */
    /*     printf("[Legged_Kinematics] no contact -> contact at {%f s}, dt: {%f s}\n", current_time, current_time - FR.t); */
    /*     if (current_time - FR.t > 1e-2) { */
    /**/
    /*         quadruped_slam::ForwardKinematicChainStamped fkc_msg; */
    /*         fkc_msg.header.stamp = t; */
    /*         fkc_msg.forward_kinematic_chain.hip = to_pose_message(from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_thigh")); */
    /*         fkc_msg.forward_kinematic_chain.thigh = to_pose_message(from_tf_tree(this->transform_buffer, "FR_thigh", "FR_calf")); */
    /*         fkc_msg.forward_kinematic_chain.calf = to_pose_message(from_tf_tree(this->transform_buffer, "FR_calf", "FR_foot")); */
    /**/
    /*         fkc_pub.publish(fkc_msg); */

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
    /*         FR.in_contact = FR_in_contact; */
    /*         FR.t = current_time; */
    /*     } */
    /* } */

    /* // @ArbitraryParameter 0.3 sec */
    /* if ((FR.in_contact && !FR_in_contact) || (FR.t != 0.0 && current_time - FR.t >= 0.3)) { */
    /*     printf("[Legged_Kinematics] contact -> no contact at {%f s}, dt: {%f s}\n", current_time, current_time - FR.t); */
    /*     if (current_time - FR.t > 1e-2) { */
    /**/
    /*         quadruped_slam::RigidContactChainStamped rcc_msg; */
    /*         rcc_msg.header.stamp = t; */
    /*         rcc_msg.rcc.fkc.hip = to_pose_message(from_tf_tree(this->transform_buffer, this->base_link_frame, "FR_thigh")); */
    /*         rcc_msg.rcc.fkc.thigh = to_pose_message(from_tf_tree(this->transform_buffer, "FR_thigh", "FR_calf")); */
    /*         rcc_msg.rcc.fkc.calf = to_pose_message(from_tf_tree(this->transform_buffer, "FR_calf", "FR_foot")); */
    /**/
    /*         rcc_pub.publish(rcc_msg); */
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
/*             FR.in_contact = FR_in_contact; */
/*             FR.t = current_time; */
/*         } */
/**/
/*     } */
/* } */

int main(int argc, char **argv) {
    ros::init(argc, argv, "legged_kinematics");

    LeggedKinematics lk;

    ros::spin();
    return 0;
}
