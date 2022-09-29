#include <gtsam/navigation/NavState.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ImuFactor.h>

#include "utils.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

using gtsam::Pose3;
using gtsam::Vector3;

struct IMU {
    gtsam::PreintegratedImuMeasurements imu_integrator;

    // Transform that takes a point in the IMU coordinate frame 
    // to a point in the base_link coordinate frame
    Pose3 base_link_T_imu;

    ros::NodeHandle nh;

    ros::Subscriber imu_sub;

    double last_time = 0;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    // for high frequency updates
    gtsam::NavState prev_state;
    gtsam::imuBias::ConstantBias prev_bias;

    IMU() :
        transform_listener(transform_buffer)
    {
        imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1, &IMU::handle_imu, this);

        // TODO(rahul): should this really be init time. Might want to do an initialization 
        // when we get the first IMU measurement
        last_time = ros::Time::now().toSec();

        // TODO(rahul): not sure if we actually have a base_link frame.
        // make this a configurable parameter
        this->base_link_T_imu = from_tf_tree(this->transform_buffer, "base_link", "imu");


        // gravity points down (-z)
        auto preintegration_params = gtsam::PreintegrationParams::MakeSharedU();
        // TODO(Rahul): set params
        preintegration_params->setAccelerometerCovariance(gtsam::Matrix::Identity(3, 3));
        preintegration_params->setGyroscopeCovariance(gtsam::Matrix::Identity(3, 3));
        preintegration_params->setIntegrationCovariance(gtsam::Matrix::Identity(3, 3));
        preintegration_params->setBodyPSensor(this->base_link_T_imu);

        gtsam::imuBias::ConstantBias prior_imu_bias{
            Vector3{0, 0, 0}, // b_a (linear acceleration bias)
            Vector3{0, 0, 0} // b_g (angular velocity bias)
        };

        this->imu_integrator = gtsam::PreintegratedImuMeasurements(preintegration_params, prior_imu_bias);

    }

    void handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data);
    void handle_biases();
};

// NOTE(Rahul):
// we want to reset the preintegration when we get a new bias from the optimizer
// this means whenever we send a factor to the optimizer, there will be a period
// of time where we are integrating measurements on the non-reset integator
// this implies we will a small queue to store IMU messages as they come in, 
// so that we can re-integrate

void IMU::handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data) {

    if (last_time == 0) {
        last_time = imu_data->header.stamp.toSec();
        return;
    }

    double dt = imu_data->header.stamp.toSec() - this->last_time;

    this->imu_integrator.integrateMeasurement(
            Vector3{imu_data->linear_acceleration.x,
                    imu_data->linear_acceleration.y,
                    imu_data->linear_acceleration.z},
            Vector3{imu_data->angular_velocity.x,
                    imu_data->angular_velocity.y,
                    imu_data->angular_velocity.z},
            dt
            );

    last_time = imu_data->header.stamp.toSec();

    const auto current_state = imu_integrator.predict(this->prev_state, this->prev_bias);

    // TODO(Rahul): again "base_link" might not exist
    to_tf_tree(this->transform_broadcaster, current_state.pose(), "odom", "base_link");
}
