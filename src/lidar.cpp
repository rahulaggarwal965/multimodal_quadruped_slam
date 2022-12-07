#include "lidar.h"
#include "pcl_conversions/pcl_conversions.h"

Lidar::Lidar()
    : prev_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      current_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      transform_listener(transform_buffer) {

        this->nh.getParam("/lidar/sensor_topic", this->topic);

        this->nh.getParam("/map_frame", this->map_frame);
        this->nh.getParam("/base_link_frame", this->base_link_frame);
        this->nh.getParam("/odom/frame", this->odom_frame);

        this->nh.getParam("/lidar/frame", this->frame);
        this->base_link_T_lidar = from_tf_tree(this->transform_buffer, this->base_link_frame, this->frame, ros::Duration{1.0});

        cloud_sub = nh.subscribe(topic, 1, &Lidar::handle_cloud, this);
        state_sub = nh.subscribe(get_param<std::string>(nh, "/optimizer/state_topic"), 1, &Lidar::handle_state, this);

        lidar_factor_pub = nh.advertise<quadruped_slam::lidar_factor>(get_param<std::string>(nh, "/lidar/factor_topic"), 1, false);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(get_param<std::string>(nh, "/lidar/cloud_topic"), 5, false);

        this->voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    }

void Lidar::handle_state(const geometry_msgs::PoseStampedConstPtr &state_msg) {
    const auto map_T_base = from_pose_stamped_message(*state_msg);
    this->last_keyframe_pose = map_T_base;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::transformPointCloud(*current_cloud, cloud, map_T_base.matrix());

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = current_cloud_timestamp;
    cloud_msg.header.frame_id = map_frame;

    this->cloud_pub.publish(cloud_msg);
}

void Lidar::handle_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    // TODO(Rahul): need to change to body frame
    pcl::PointCloud<OusterPointXYZIR> ouster_cloud;
    pcl::fromROSMsg(*cloud_msg, ouster_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>{}};

    // get rid of points below a certain distance threshold
    for (const auto &point : ouster_cloud.points) {
        const float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        if (dist < 0.2) {
            continue; 
        }

        cloud->emplace_back(point.x, point.y, point.z);
    }

    pcl::transformPointCloud(*cloud, *cloud, this->base_link_T_lidar.matrix());


    if (current_cloud_timestamp.isZero()) {
        // initialize
        this->last_keyframe_pose = from_tf_tree(this->transform_buffer, this->odom_frame, this->base_link_frame);
        this->current_cloud_timestamp = cloud_msg->header.stamp;
        this->current_cloud = cloud;

        voxel_grid.setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target{new pcl::PointCloud<pcl::PointXYZ>{}};
        voxel_grid.filter(*target);
        gicp.setInputTarget(target);

        return;
    }

    gtsam::Pose3 approximate_state = from_tf_tree(this->transform_buffer, this->odom_frame, this->base_link_frame);
    
    this->prev_cloud_timestamp = this->current_cloud_timestamp;
    this->prev_cloud = this->current_cloud;

    this->current_cloud_timestamp = cloud_msg->header.stamp;
    this->current_cloud = cloud;

    printf("Running ICP between cloud [%f] and cloud [%f]\n", prev_cloud_timestamp.toSec(), current_cloud_timestamp.toSec());

    voxel_grid.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source{new pcl::PointCloud<pcl::PointXYZ>{}};
    voxel_grid.filter(*source);
    gicp.setInputSource(source);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*aligned);
    gicp.swapSourceAndTarget();

    const auto pose = gicp.getFinalTransformation();

    gtsam::Pose3 a_T_b = gtsam::Pose3{pose.cast<double>()};

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

