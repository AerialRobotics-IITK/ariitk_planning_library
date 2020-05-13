#pragma once

#include <ros/ros.h>
#include <ariitk_planning_msgs/Frontier.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <rviz_visualizer/visualizer.hpp>

namespace ariitk::frontier_explorer {

typedef ariitk::rviz_visualizer::Visualizer Visualizer;

typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point;
typedef std::pair<Point, unsigned> Value;
typedef boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> RTree;

typedef struct GraphNode {
    uint id;
    Eigen::Vector3d pos;
    typedef std::shared_ptr<GraphNode> Ptr;
    std::vector<GraphNode::Ptr> neighbours;

    GraphNode(const pcl::PointXYZI& point,uint id) 
        : id(id)
        , pos(point.x, point.y, point.z) {}
    void addNeighbour(const GraphNode::Ptr& node) { neighbours.push_back(node); }
} Node;

typedef std::vector<Node::Ptr> Graph;
typedef std::vector<Eigen::Vector3d> Path;

class FrontierComparator {
    public:
        FrontierComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        bool operator()(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2);
    
    private:
        void sliceCallback(const sensor_msgs::PointCloud2& msg);
        void odometryCallback(const nav_msgs::Odometry& odom);
        
        static inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z), 2));
        }

        inline std::string getHash(const geometry_msgs::Point& point) {
            return std::to_string(int(point.x / voxel_size_)) + "," + std::to_string(int(point.y / voxel_size_));
        };

        Path findPath(const geometry_msgs::Point& start, const geometry_msgs::Point& end);
        double getPathLength(const Path& path);
        double getDistance(const geometry_msgs::Point& point);
        void generateGraph(const pcl::PointCloud<pcl::PointXYZI>& slice);
        void insertNode(const geometry_msgs::Point& point);
        ariitk::rviz_visualizer::Graph convertGraph(const Graph& graph);
        pcl::PointXYZI convertToVoxel(const geometry_msgs::Point& point);

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber odom_sub_;
        ros::Subscriber esdf_slice_sub_;

        bool visualize_;
        double clear_radius_;
        double slice_level_;
        double min_size_;
        double voxel_size_;
        double robot_radius_;

        geometry_msgs::Pose curr_pose_;

        RTree esdf_tree_;
        Graph esdf_graph_;

        Visualizer visualizer_;

        std::unordered_map<std::string, double> distance_map_;
};

} // namespace ariitk::frontier_explorer
