#include <ros/ros.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <gtsam/geometry/Pose3.h>

#include "lidar.h"
#include "utils.h"
#include <teaser/registration.h>

#include <quadruped_slam/lidar_factor.h>

struct Lidar {
    
    std::string topic;

    std::string frame;
    std::string base_link_frame;

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;

    ros::Publisher lidar_factor_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
    ros::Time prev_cloud_timestamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    ros::Time current_cloud_timestamp;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;
    tf2_ros::TransformBroadcaster transform_broadcaster;

    gtsam::Pose3 base_link_T_lidar;

    Lidar()
    : prev_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      current_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      transform_listener(transform_buffer) {

        this->nh.getParam("/lidar/sensor_topic", this->topic);

        this->nh.getParam("/base_link_frame", this->base_link_frame);
        this->nh.getParam("/lidar/frame", this->frame);
        this->base_link_T_lidar = from_tf_tree(this->transform_buffer, this->base_link_frame, this->frame, ros::Duration{1.0});

        cloud_sub = nh.subscribe(topic, 1, &Lidar::handle_cloud, this);

        std::string factor_topic;
        this->nh.getParam("/lidar/factor_topic", factor_topic);
        lidar_factor_pub = nh.advertise<quadruped_slam::lidar_factor>(factor_topic, 1, false);
    }

    void handle_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
};

teaser::RegistrationSolution icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &dst) {

    const auto num_src_points = src->size();
    const auto num_dst_points = dst->size();

    Eigen::Matrix<double, 3, Eigen::Dynamic> src_points{3, num_src_points}, dst_points{3, num_dst_points};

    for (auto i = 0; i < num_src_points; i++) {
        const auto pt = src->points[i];
        src_points(0, i) = pt.x;
        src_points(1, i) = pt.y;
        src_points(2, i) = pt.z;
    }

    for (auto i = 0; i < num_dst_points; i++) {
        const auto pt = dst->points[i];
        dst_points(0, i) = pt.x;
        dst_points(1, i) = pt.y;
        dst_points(2, i) = pt.z;
    }

    teaser::RobustRegistrationSolver::Params params;
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src_points, dst_points); // assuming src & dst are 3-by-N Eigen matrices
    return solver.getSolution();
}

void Lidar::handle_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // TODO(Rahul): need to change to body frame
    pcl::PointCloud<OusterPointXYZIRT> ouster_cloud;
    pcl::fromROSMsg(*cloud_msg, ouster_cloud);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // get rid of points below a certain distance threshold
    for (const auto &point : ouster_cloud.points) {
        const float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        if (dist < 0.2) {
            continue; 
        }

        cloud.emplace_back(point.x, point.y, point.z);
    }


    if (prev_cloud_timestamp.isZero()) {
        // initialize
        this->current_cloud_timestamp = cloud_msg->header.stamp;
        pcl::transformPointCloud(cloud, *this->current_cloud, this->base_link_T_lidar.matrix());
        return;
    }
    
    this->prev_cloud_timestamp = this->current_cloud_timestamp;
    this->prev_cloud = this->current_cloud;

    this->current_cloud_timestamp = cloud_msg->header.stamp;
    pcl::transformPointCloud(cloud, *this->current_cloud, this->base_link_T_lidar.matrix());

    const auto solution = icp(this->prev_cloud, this->current_cloud);
    
    if (!solution.valid) {
        return;
    }

    gtsam::Pose3 a_T_b = gtsam::Pose3{gtsam::Rot3{solution.rotation}, solution.translation};

    quadruped_slam::lidar_factor factor;
    factor.header.stamp = ros::Time::now();
    factor.pose = to_pose_message(a_T_b);
    factor.cloud_a_stamp = prev_cloud_timestamp;
    factor.cloud_b_stamp = current_cloud_timestamp;

    this->lidar_factor_pub.publish(factor);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar");

    Lidar l;
    
    ros::spin();
    return 0;
}

