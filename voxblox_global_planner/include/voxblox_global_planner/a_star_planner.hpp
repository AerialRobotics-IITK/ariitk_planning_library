#pragma once

#include<memory>

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <Eigen/Eigen>
#include <mav_planning_msgs/PlannerService.h>
#include <std_srvs/Empty.h>
#include <pcl/pcl_macro.h>

namespace ariitk::global_planner {

struct GraphNode {
    uint id;
    Eigen::Vector3d position;
    typedef std::shared_ptr<struct GraphNode> Ptr;
    std::vector<Ptr > neighbours;
};

class AStarPlanner {
    public:
        AStarPlanner(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);


    private:
        bool plannerServiceCallback(
            mav_planning_msgs::PlannerServiceRequest& request,
            mav_planning_msgs::PlannerServiceResponse& response);
        bool publishPathCallback(
            std_srvs::EmptyRequest& request,
            std_srvs::EmptyResponse& response);
        void generateGraph();
        void findPath(const Eigen::Vector3d& start_pose, const Eigen::Vector3d& goal_pose);
        void esdfSliceCallback(pcl::PointCloud<pcl::PointXYZI> pointcloud);

        voxblox::EsdfServer voxblox_server_;
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher esdf_slice_pub_;
        ros::Subscriber esdf_slice_sub_;
        std::vector<GraphNode::Ptr> graph_;
        std::vector<Eigen::Vector3d > path_;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_;
        double robot_radius_;
};
}