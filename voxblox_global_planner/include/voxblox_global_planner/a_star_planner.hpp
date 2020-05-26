
#pragma once
#include <geometry_msgs/PoseArray.h>
#include<memory>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_trajectory_generation/timing.h>
#include <mav_planning_common/utils.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_visualization/helpers.h>
#include <pcl/pcl_macros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox_planning_common/path_shortening.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton_planner.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <voxblox_skeleton/sparse_graph_planner.h>

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
typedef std::vector<Node> Graph;
typedef std::vector<Eigen::Vector3d> Path;
typedef std::vector<Path> Paths;
class Color : public std_msgs::ColorRGBA {
    public:
        Color() : std_msgs::ColorRGBA() {}
        Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
        Color(double red, double green, double blue, double alpha) : Color() {
            r = red;
            g = green;
            b = blue;
            a = alpha;
        }

    static const Color White() { return Color(1.0, 1.0, 1.0); }
    static const Color Black() { return Color(0.0, 0.0, 0.0); }
    static const Color Gray() { return Color(0.5, 0.5, 0.5); }
    static const Color Red() { return Color(1.0, 0.0, 0.0); }
    static const Color Green() { return Color(0.0, 1.0, 0.0); }
    static const Color Blue() { return Color(0.0, 0.0, 1.0); }
    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
    static const Color Orange() { return Color(1.0, 0.5, 0.0); }
    static const Color Purple() { return Color(0.5, 0.0, 1.0); }
    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
    static const Color Teal() { return Color(0.0, 1.0, 1.0); }
    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

class PathVisualizer {
    public:
        void init(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        void createPublisher(const std::string& topic_name);
        enum class ColorType{WHITE, BLACK, GRAY, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE, CHARTREUSE, TEAL, PINK};
        void visualizePath(const std::string& topic_name, const std::vector<Eigen::Vector3d>& path, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::PINK, const double& size_factor = 0.5);
        void visualizePaths(const std::string& topic_name, const std::vector<std::vector<Eigen::Vector3d>>& paths, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::PINK, const double& size_factor = 0.5);
        void visualizeGraph(const std::string& topic_name, const Graph& graph, const std::string& frame_id = "world",
                        const ColorType& vertex_color = ColorType::ORANGE, const ColorType& edge_color = ColorType::BLUE, const double& size_factor = 0.5);
        void visualizePoint(const std::string& topic_name, const Eigen::Vector3d& point, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::RED, const double& size_factor = 1.0);
        void visualizePoints(const std::string& topic_name, const std::vector<Eigen::Vector3d>& point, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::RED, const double& size_factor = 1.0);
        void visualizeTrajectory(const std::string& topic_name, const mav_msgs::EigenTrajectoryPointVector& trajectory, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::BLACK, const double& size_factor = 1.0);
                       
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_; 
        bool visualize_;
        double voxel_size_; // Cache the size of the voxels used by the map.
        std::unordered_map<ColorType, Color> color_map_;
        std::unordered_map<std::string, ros::Publisher> publisher_map_;
};

class AStarPlanner {
    public:
        AStarPlanner(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private);
         bool plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& request,
                                    mav_planning_msgs::PlannerServiceResponse& response);
        bool publishPathCallback(std_srvs::EmptyRequest& request,
                                std_srvs::EmptyResponse& response);
        double getMapDistance(const Eigen::Vector3d& position);
        void generateSparseGraph();
        void skeletonize(voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer);


    private:
        void createGraph(const Eigen::Vector3d& start_pose, const Eigen::Vector3d& goal_pose);
        void findPath(const Eigen::Vector3d& start_pose, const Eigen::Vector3d& goal_pose);
        void searchPath(const uint& start_index, const uint& end_index);
        void esdfSliceCallback(pcl::PointCloud<pcl::PointXYZI> pointcloud);
        // void shortenPath();
        // void findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map);
        // bool isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

        voxblox::EsdfServer voxblox_server_;
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber esdf_slice_sub_;
        ros::Publisher esdf_slice_pub_;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_;
        std::vector<Node> graph_;
        Path path_;
        ros::Publisher path_marker_pub_;
        ros::Publisher sparse_graph_pub_;
        ros::Publisher waypoint_list_pub_;
        ros::ServiceServer planner_srv_;
        ros::ServiceServer path_pub_srv_;

        std::string frame_id_;
        bool verbose_;

        // Settings for physical constriants.
        mav_planning::PhysicalConstraints constraints_;
        voxblox::SkeletonGenerator skeleton_generator_;

        double robot_radius_;
        double voxel_size_;
        RTree tree_;
        PathVisualizer visualizer_;
        bool visualize_;

        voxblox::SkeletonAStar skeleton_planner_;
        mav_planning::EsdfPathShortener path_shortener_;

        mav_msgs::EigenTrajectoryPointVector last_waypoints_;
};

}