#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ImuFactor.h>

struct IMU {
    gtsam::PreintegratedImuMeasurements imu_integrator;

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
    gtsam::imuBias::ConstantBias prev_bias;

    gtsam::ImuFactor current_imu_factor;

    int state_index = 0;

    IMU(ros::NodeHandle &nh);

    void handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data);
    void reset_integration(const gtsam::Vector3 &bias_acc, const gtsam::Vector3 &bias_gyro);
};
