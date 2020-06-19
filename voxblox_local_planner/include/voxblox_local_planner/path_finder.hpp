#pragma once

#include <Eigen/Core>
#include <random>
#include <ros/ros.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>

#include <rviz_visualizer/visualizer.hpp>
#include <voxblox_local_planner/graph_def.hpp>

namespace ariitk::local_planner {

typedef std::vector<Eigen::Vector3d> Path;
typedef std::vector<Path> Paths;
typedef ariitk::rviz_visualizer::Visualizer Visualizer;

class PointSampler {
    public:
        PointSampler() {
            engine_ = std::default_random_engine(rd_());
            dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
        }

        void init(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void expandRegion(const double& size);
        Eigen::Vector3d getSample();
        double getWidth() { return region_(1); }

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

        inline bool getMapDistance(const Eigen::Vector3d& point, double& distance) { return server_.getEsdfMapPtr()->getDistanceAtPosition(point, &distance); }
        double getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient);

        void expandSamplingRegion(const double& size);
        void increaseSamplingDensity(const double& factor);
        void inflateRadius(const double& factor);

        double getPathLength(const Path& path);

    private:
        void createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void searchPath(const uint& start_index, const uint& end_index);
        void shortenPath();

        void findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map);
        bool isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

        ariitk::rviz_visualizer::Graph convertGraph(const Graph& graph);

        Path short_path_;
        Path raw_path_;
        Path path_;

        Graph graph_;
        RTree tree_;

        voxblox::EsdfServer server_;

        std::vector<Eigen::Vector3d> neighbor_voxels_;

        uint p_sample_;
        int num_neighbours_;

        bool visualize_;
        bool inc_density_;
        bool inflate_radius_;
        bool expand_region_;

        double robot_radius_;
        double voxel_size_;
        double expand_size_;
        double density_factor_;
        double inflate_factor_;

        PointSampler sampler_;
        Visualizer visualizer_;
};

}  // namespace ariitk::local_planner
