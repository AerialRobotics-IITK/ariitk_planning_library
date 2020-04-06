#include <obstacle_detector/obstacle_detector.hpp>

#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

namespace ariitk::obstacle_detector {

void ObstacleDetector::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    freespace_available_ = false;
    freespace_center_ = Eigen::Vector3d::Zero();
    
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("voxel_size", voxel_size_);
    nh_private.getParam("visible_horizon", visible_horizon_);
    nh_private.getParam("slice_level", slice_level_);
    nh_private.getParam("height_range", height_range_);
    nh_private.getParam("quad_frame_id", quad_frame_id_);
    nh_private.getParam("world_frame_id", world_frame_id_);

    point_cloud_sub_ = nh.subscribe("cloud", 1, &ObstacleDetector::pointCloudCallback, this);
    
    freespace_center_pub_ = nh_private.advertise<geometry_msgs::PointStamped>("freespace_center", 1);
    obstacle_cloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>("obs_cloud", 1);
    boundary_pub_ = nh_private.advertise<geometry_msgs::PoseArray>("boundaries", 1);

    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    freespace_center_.z() = 0;
}

void ObstacleDetector::run() {
    // if(!freespace_available_) { return; }
    
    geometry_msgs::PointStamped center_msg;
    center_msg.header.stamp = ros::Time::now();
    center_msg.header.frame_id = quad_frame_id_;
    center_msg.point.x = freespace_center_.x();
    center_msg.point.y = freespace_center_.y();
    center_msg.point.z = freespace_center_.z();
    freespace_center_pub_.publish(center_msg);

    pcl::PCLPointCloud2Ptr obs_cloud(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(obstacle_cloud_, *obs_cloud);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl_conversions::fromPCL(*obs_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = quad_frame_id_;
    obstacle_cloud_pub_.publish(cloud_msg);

    geometry_msgs::PoseArray bound_msg;
    bound_msg.header.frame_id = quad_frame_id_;
    bound_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < 4; i++) {
        geometry_msgs::Pose pose;
        pose.position.x = freespace_boundary_[i].x();
        pose.position.y = freespace_boundary_[i].y();
        pose.position.z = 0;
        bound_msg.poses.push_back(pose);
    }
    boundary_pub_.publish(bound_msg);
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfer_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2Ptr quad_cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2Ptr voxel_cloud(new pcl::PCLPointCloud2());

    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(quad_frame_id_, msg->header.frame_id, ros::Time(0), transform);
    } catch(tf::TransformException e) {
        ROS_ERROR("%s", e.what());
    }
    pcl_conversions::toPCL(*msg, *voxel_cloud);
    pcl::fromPCLPointCloud2(*voxel_cloud, *transfer_cloud);
    pcl_ros::transformPointCloud(*transfer_cloud, *input_cloud, transform);
    pcl::toPCLPointCloud2(*input_cloud, *quad_cloud);

    voxel_filter_.setInputCloud(quad_cloud);
    voxel_filter_.filter(*voxel_cloud);

    detectObstacles(voxel_cloud);
}

void ObstacleDetector::detectObstacles(const pcl::PCLPointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(*cloud, *voxel_cloud);
    uint num_occluded_points = 0;

    obstacle_cloud_.points.clear();
    for(int i = 0; i < 4; i++) { freespace_boundary_[i] = Eigen::Vector3d(voxel_cloud->points[0].x, voxel_cloud->points[0].y, 0); }

    for(auto& point : voxel_cloud->points) {
        if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) { continue; }
        if(fabs(point.z) >= height_range_) { continue; }
        if(pow(point.x, 2) + pow(point.y, 2) >= pow(visible_horizon_, 2)) { continue; }
        
        num_occluded_points += 1;

        obstacle_cloud_.points.push_back(point);

        if(point.x <= freespace_boundary_[0].x()) { freespace_boundary_[0] = Eigen::Vector3d(point.x, point.y, 0); }
        if(point.x >= freespace_boundary_[1].x()) { freespace_boundary_[1] = Eigen::Vector3d(point.x, point.y, 0); }
        
        if(point.y <= freespace_boundary_[2].y()) { freespace_boundary_[2] = Eigen::Vector3d(point.x, point.y, 0); }
        if(point.y >= freespace_boundary_[3].y()) { freespace_boundary_[3] = Eigen::Vector3d(point.x, point.y, 0); }
    }

    // for(int i = 0; i < 4; i++) { freespace_center_ += freespace_boundary_[i]; }
    // freespace_center_ /= 4.0;
    freespace_center_ = (freespace_boundary_[0] + freespace_boundary_[2])/2.0;
}

} // namespace ariitk::obstacle_detector
