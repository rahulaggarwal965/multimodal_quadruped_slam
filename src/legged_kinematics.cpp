#include <ros/ros.h>
#include <ros/subscriber.h>

#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

#include <unitree_legged_msgs/LowState.h>

gtsam::Matrix33 compute_leg_jacobian(const gtsam::Vector3 &q, const gtsam::Vector4 &l) {

    // q = joint angles
    // l = limb lengths

    // TODO(rahul): what to these represent
    // l1 = abadLinkLength;
    // l2 = hipLinkLength;
    // l3 = kneeLinkLength;
    // l4 = kneeLinkY_offset;

    const auto s1 = std::sin( q[0]);
    const auto s2 = std::sin(-q[1]);
    const auto s3 = std::sin(-q[2]);

    const auto c1 = std::cos( q[0]);
    const auto c2 = std::cos(-q[1]);
    const auto c3 = std::cos(-q[2]);

    const auto c23 = c2 * c3 - s2 * s3;
    const auto s23 = s2 * c3 + c2 * s3;

    gtsam::Matrix33 J {
        {0, -l[2] * c23 - l[1] * c2,  -l[2] * c23},
        {},
        {},
    }

    if (J) {
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = -l3 * c23 - l2 * c2;
        J->operator()(0, 2) = -l3 * c23;
        J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
        J->operator()(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
        J->operator()(1, 2) = l3 * s1 * s23;
        J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
        J->operator()(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
        J->operator()(2, 2) = -l3 * c1 * s23;
    }
}

struct Leg {
    const int sideSign;
    Eigen::Vector3d q; //joint angles

};

struct LeggedKinematics {

    const int side_sign[4] = {-1, 1, -1, 1};

    ros::NodeHandle nh;

    // joint state
    ros::Subscriber leg_state_sub;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener tranform_listener;

    LeggedKinematics();

    void handle_low_state(const unitree_legged_msgs::LowStateConstPtr &low_state);

};

LeggedKinematics::LeggedKinematics() 
    : tranform_listener(transform_buffer)
{
    // TODO(Rahul): make topics params
    leg_state_sub = nh.subscribe<unitree_legged_msgs::LowState>("/a1_gazebo/lowState/state", 1, &LeggedKinematics::handle_low_state, this);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Legged Kinematics");

    LeggedKinematics lk;

    ros::spin();
    return 0;
}
