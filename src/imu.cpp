#include <ros/ros.h>
#include <ros/time.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_listener.h>

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

    IMU() :
        transform_listener(transform_buffer)
    {
        imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1, &IMU::handle_imu, this);

        // TODO(rahul): should this really be init time. Might want to do an initialization 
        // when we get the first IMU measurement
        last_time = ros::Time::now().toSec();

        geometry_msgs::TransformStamped base_link_to_imu = this->transform_buffer.lookupTransform("base_link", "imu", ros::Time::now());

        // TODO(rahul): not sure if we actually have a base_link frame.
        // make this a configurable parameter
        this->base_link_T_imu = from_tf_tree(this->transform_buffer, "base_link", "imu");
    }

    void handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data);
};

void IMU::handle_imu(const sensor_msgs::Imu::ConstPtr &imu_data) {
    this->imu_integrator.integrateMeasurement(
            gtsam::Vector3()
            )
}
