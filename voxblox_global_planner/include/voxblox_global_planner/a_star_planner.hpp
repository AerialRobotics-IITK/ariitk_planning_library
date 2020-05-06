#pragma once

#include<memory>

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <Eigen/Eigen>
// #include <mav_planning_msgs/PlannerService.h>
// #include <std_srvs/Empty.h>
#include <pcl/pcl_macros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace ariitk::global_planner {

typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point;
typedef std::pair<Point, unsigned> Value;
typedef boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> RTree;

struct GraphNode {

    uint id_;
    Eigen::Vector3d position_;
    typedef std::shared_ptr<struct GraphNode> Ptr;
    typedef GraphNode::Ptr Node;
    std::vector<Ptr > neighbours_;

    GraphNode(const pcl::PointXYZI& point,uint id);
    void addNeighbour(const Node& node) { neighbours_.push_back(node); }
};

typedef std::shared_ptr<struct GraphNode> Ptr;
typedef GraphNode::Ptr Node;

class AStarPlanner {
    public:
        AStarPlanner(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);
        void esdfSliceCallback(pcl::PointCloud<pcl::PointXYZI> pointcloud);
        ros::Publisher esdf_slice_pub_;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_;

    private:
        // bool plannerServiceCallback(
        //     mav_planning_msgs::PlannerServiceRequest& request,
        //     mav_planning_msgs::PlannerServiceResponse& response);
        // bool publishPathCallback(
        //     std_srvs::EmptyRequest& request,
        //     std_srvs::EmptyResponse& response);
        void generateGraph();
        void findPath(const Eigen::Vector3d& start_pose, const Eigen::Vector3d& goal_pose);

        voxblox::EsdfServer voxblox_server_;
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber esdf_slice_sub_;
        std::vector<Node> graph_;
        std::vector<Eigen::Vector3d > path_;
    
        double robot_radius_;
        RTree tree_;
};


}