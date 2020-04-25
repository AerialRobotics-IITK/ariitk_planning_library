#pragma once

#include <random>
#include <ros/ros.h>
#include <Eigen/Core>
#include <voxblox_ros/esdf_server.h>
#include <voxblox/core/tsdf_map.h>

#include <voxblox_local_planner/ray_caster.hpp>
#include <voxblox_local_planner/path_visualizer.hpp>
#include <voxblox_local_planner/graph_def.hpp>

namespace ariitk::local_planner {

typedef std::vector<Eigen::Vector3d> Path;
typedef std::vector<Path> Paths;

class PointSampler {
    public:
        PointSampler() {
            engine_ = std::default_random_engine(rd_()) ;
            dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        }
        void init(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        Eigen::Vector3d getSample();
    
    private:
        std::random_device rd_;
        std::default_random_engine engine_;
        std::uniform_real_distribution<double> dist_;
        
        Eigen::Matrix3d rotation_;
        Eigen::Vector3d translation_;
        Eigen::Vector3d region_;
};

class PathFinder {
    public:
        PathFinder(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void setRobotRadius(const double& robot_radius) { robot_radius_ = robot_radius; };
        void findPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
        Path getPath() { return path_; };
        double getMapDistance(const Eigen::Vector3d& point);
       
        // void visualizePaths();
        // void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private); // need easier init
        // void setEsdfMapPtr(const voxblox::EsdfMap::Ptr& map_ptr) { esdf_map_ptr_ = map_ptr; };
        // void setTsdfMapPtr(const voxblox::EsdfMap::Ptr& map_ptr) { tsdf_map_ptr_ = map_ptr; };
        // void setOrigin(const Eigen::Vector3d& origin) { origin_ = origin; };

    private:
        void createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void searchPath(const uint& start_index, const uint& end_index);
        void shortenPath();
        void findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map);
        bool isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        double getPathLength(const Path& path);
    
        // void createGraph();
        // Path evaluatePaths(const Paths& paths);
        // void pruneGraph(Graph& graph);
        // Paths traverseGraph(const Graph& graph);
        // Nodes findVisibleGuards(const Graph& graph, const Eigen::Vector3d& point);
        // bool checkConnection(const Node& start, const Node& end, const Eigen::Vector3d& point);
        // bool hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& threshold = 0.0);
        // bool hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Eigen::Vector3d& point, const double& threshold = 0.0);
        // bool checkPathSimilarity(const Path& path1, const Path& path2, const double& threshold = 0.0);
        // Path discretizePath(const Path& path, const uint& num_points);
        // Path linearizePath(const Path& path);
        // Path linearize(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        // int getIndex(const Eigen::Vector3d& point);

        // void trim(Paths& paths);
        // void trimPath(Path& path, const uint& iterations = 1);
        // Paths removeDuplicates(Paths& paths);

        // bool getMapGradient(const Eigen::Vector3d& point, Eigen::Vector3d& gradient);

        Path short_path_;
        Path raw_path_;
        Path path_;

        Graph graph_;
        RTree tree_;

        voxblox::EsdfServer server_;

        std::vector<Eigen::Vector3d> neighbor_voxels_;
        
        double robot_radius_;
        double voxel_size_;

        bool visualize_;

        PointSampler sampler_;
        PathVisualizer visualizer_;
        
        // RayCaster caster_;
        // Eigen::Vector3d origin_;
};

} // namespace ariitk::local_planner
