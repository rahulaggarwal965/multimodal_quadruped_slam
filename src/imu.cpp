#include "imu.h"
#include "utils.h"
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

using gtsam::Vector3;

IMU::IMU(ros::NodeHandle &nh) 
    : nh(nh),
      transform_listener(transform_buffer)
{

    this->nh.getParam("/imu/imu_topic", this->imu_topic);
    this->nh.getParam("/imu/imu_frame", this->imu_frame);
    this->nh.getParam("/imu/high_frequency_state_topic", this->high_frequency_state_topic);

    this->nh.getParam("/base_link_frame", this->base_link_frame);
    this->nh.getParam("/odom_frame", this->odom_frame);

    imu_sub = this->nh.subscribe<sensor_msgs::Imu>(this->imu_topic, 1, &IMU::handle_imu, this, ros::TransportHints().tcpNoDelay());

    high_frequency_pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>(this->high_frequency_state_topic, 1);

    // TODO(rahul): should this really be init time. Might want to do an initialization 
    // when we get the first IMU measurement
    last_time = ros::Time::now().toSec();

    this->base_link_T_imu = from_tf_tree(this->transform_buffer, this->base_link_frame, this->imu_frame, ros::Duration{1.0});

    // gravity points down (-z)
    auto preintegration_params = gtsam::PreintegrationParams::MakeSharedU();
    // TODO(Rahul): set params

    preintegration_params->setAccelerometerCovariance(vector_from_param<3>(nh, "/imu/accelerometer_variances").asDiagonal());
    preintegration_params->setGyroscopeCovariance(vector_from_param<3>(nh, "/imu/gyroscope_variances").asDiagonal());
    preintegration_params->setIntegrationCovariance(vector_from_param<3>(nh, "/imu/integration_variances").asDiagonal());
    preintegration_params->setBodyPSensor(this->base_link_T_imu);

     this->prev_bias = {
        vector_from_param<3>(nh, "/imu/prior_accelerometer_bias"),
        vector_from_param<3>(nh, "/imu/prior_gyroscope_bias")
    };

    this->imu_integrator = gtsam::PreintegratedImuMeasurements(preintegration_params, this->prev_bias);
}

// NOTE(Rahul):
// we want to reset the preintegration when we get a new bias from the optimizer
// this means whenever we send a factor to the optimizer, there will be a period
// of time where we are integrating measurements on the non-reset integator
// this implies we will a small queue to store IMU messages as they come in, 
// so that we can re-integrate

IMUFactor IMU::create_factor(int from, int to) {
    IMUFactor factor;
    factor.factor = gtsam::ImuFactor(X(from), V(from), X(to), V(to), B(0), this->imu_integrator);
    factor.pose_estimate = this->current_state.pose();
    factor.velocity_estimate = this->current_state.v();
    factor.bias_estimate = this->prev_bias;
    // TODO(rahul): figure out noise model for this factor
    /* graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(from), B(to), gtsam::imuBias::ConstantBias(), )) */

    return factor;
};

void IMU::handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data) {

    reset = false;

    const double current_time = imu_data->header.stamp.toSec();

    if (last_time == 0) {
        last_time = current_time;
        return;
    }

    const double dt = current_time - this->last_time;

    printf("[IMU] imu_received at {%f s}, dt: {%f s}\n", current_time, dt);


    this->imu_integrator.integrateMeasurement(
            Vector3{imu_data->linear_acceleration.x,
                    imu_data->linear_acceleration.y,
                    imu_data->linear_acceleration.z},
            Vector3{imu_data->angular_velocity.x,
                    imu_data->angular_velocity.y,
                    imu_data->angular_velocity.z},
            dt
            );

    this->last_time = current_time;

    this->current_state = imu_integrator.predict(this->prev_state, this->prev_bias);

    // TODO(rahul):
    // should we introduce a factor between the biases B(x - 1) and B(x) to say that bias remains constant?
    // would introduce a slowly changing bias constraint that might be useful

    to_tf_tree(this->transform_broadcaster, current_state.pose(), this->odom_frame, this->base_link_frame);

    const geometry_msgs::PoseStamped pose_stamped_message = to_pose_stamped_message(current_state.pose(), this->odom_frame);
    this->high_frequency_pose_pub.publish(pose_stamped_message);
}

// Because this function is called by the optimizer, which can happen arbitrarily far after 
// current_imu_factor is actually created, if this is multithreaded, we have to take extra 
// precaution to not lose any measurements
void IMU::reset_integration(const gtsam::imuBias::ConstantBias &b, const gtsam::Pose3 &p, const gtsam::Vector3 &v) {
    this->imu_integrator.resetIntegrationAndSetBias(b);
    this->prev_bias = b;
    this->prev_state = {p, v};
    this->reset = true;
}
