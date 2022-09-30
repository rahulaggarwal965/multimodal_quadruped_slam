#include "imu.h"
#include "utils.h"

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

using gtsam::Vector3;

IMU::IMU(ros::NodeHandle &nh) 
    : nh(nh),
      transform_listener(transform_buffer)
{
    imu_sub = this->nh.subscribe<sensor_msgs::Imu>("imu", 1, &IMU::handle_imu, this);

    high_frequency_pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("high_frequency/pose", 1);

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

    current_imu_factor = gtsam::ImuFactor(X(state_index), V(state_index),
                                          X(state_index + 1), V(state_index + 1), B(state_index),
                                          this->imu_integrator);

    // TODO(rahul):
    // should we introduce a factor between the biases B(x - 1) and B(x) to say that bias remains constant?
    // would introduce a slowly changing bias constraint that might be useful


    // TODO(Rahul): again "base_link" might not exist
    to_tf_tree(this->transform_broadcaster, current_state.pose(), "odom", "base_link");

    geometry_msgs::PoseStamped pose_stamped_message = to_pose_stamped_message(current_state.pose(), "odom");
    this->high_frequency_pose_pub.publish(pose_stamped_message);
}

// Because this function is called by the optimizer, which can happen arbitrarily far after 
// current_imu_factor is actually created, if this is multithreaded, we have to take extra 
// precaution to not lose any measurements
void IMU::reset_integration(const gtsam::Vector3 &bias_acc, const gtsam::Vector3 &bias_gyro) {
    this->prev_bias = {bias_acc, bias_gyro};
    this->imu_integrator.resetIntegrationAndSetBias(this->prev_bias);
    this->state_index += 1;
}
