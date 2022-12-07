#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ImuFactor.h>

struct IMUFactor;

struct IMU {
    gtsam::PreintegratedImuMeasurements imu_integrator;

    std::string imu_topic;
    std::string imu_frame;
    std::string high_frequency_state_topic;

    std::string base_link_frame;
    std::string odom_frame;
    
    bool reset = false;

    // Transform that takes a point in the IMU coordinate frame 
    // to a point in the base_link coordinate frame
    gtsam::Pose3 base_link_T_imu;

    ros::NodeHandle &nh;

    ros::Subscriber imu_sub;
    ros::Publisher high_frequency_pose_pub;

    double last_time = 0;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    // contains last optimized state
    gtsam::NavState prev_state;
    gtsam::NavState current_state;
    gtsam::imuBias::ConstantBias prev_bias;

    IMU(ros::NodeHandle &nh);

    void handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data);
    void reset_integration(const gtsam::imuBias::ConstantBias &b, const gtsam::Pose3 &p, const gtsam::Vector3 &v);

    IMUFactor create_factor(int from, int to);
};

struct IMUFactor {
    gtsam::ImuFactor factor;
    gtsam::Pose3 pose_estimate;
    gtsam::Vector3 velocity_estimate;
    gtsam::imuBias::ConstantBias bias_estimate;
};
