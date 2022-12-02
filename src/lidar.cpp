#include <ros/ros.h>
#include <ros/subscriber.h>

#include <pcl_conversions/pcl_conversions.h>

#include "lidar.h"
#include <teaser/registration.h>

struct Lidar {
    
    std::string topic;
    std::string frame;

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr prev_cloud;
    ros::Time prev_cloud_timestamp;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr current_cloud;
    ros::Time current_cloud_timestamp;

    Lidar()
    : prev_cloud(new pcl::PointCloud<OusterPointXYZIRT>()),
      current_cloud(new pcl::PointCloud<OusterPointXYZIRT>()) {

        this->nh.getParam("/lidar/topic", this->topic);
        cloud_sub = nh.subscribe(topic, 1, &Lidar::handle_cloud, this);
    }

    void handle_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

void Lidar::handle_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // TODO(Rahul): need to change to body frame
    if (prev_cloud_timestamp.isZero()) {
        // initialize
        this->current_cloud_timestamp = cloud_msg->header.stamp;
        pcl::fromROSMsg(*cloud_msg, *this->current_cloud);
        return;
    }
    
    this->prev_cloud_timestamp = this->current_cloud_timestamp;
    this->prev_cloud = this->current_cloud;

    this->current_cloud_timestamp = cloud_msg->header.stamp;
    pcl::fromROSMsg(*cloud_msg, *this->current_cloud);

    // icp
}

void icp(const pcl::PointCloud<OusterPointXYZIRT>::Ptr &src, const pcl::PointCloud<OusterPointXYZIRT>::Ptr &dst) {
    Eigen::Matrix<double, 3, Eigen::Dynamic> src_points, dst_points;

    for (const auto &poi)
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar");

    Lidar l;
    
    ros::spin();
    return 0;
}

