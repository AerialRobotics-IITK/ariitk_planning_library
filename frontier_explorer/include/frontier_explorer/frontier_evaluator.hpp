#pragma once

#include <Eigen/Core>
#include <mav_planning_common/physical_constraints.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>

#include <ariitk_planning_msgs/Frontiers.h>
#include <rviz_visualizer/visualizer.hpp>

namespace ariitk::frontier_explorer {

typedef ariitk::rviz_visualizer::Visualizer Visualizer;

enum class VoxelState { OCCUPIED, FREE, UNKNOWN };

class FrontierEvaluator {
    public:
        FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

        void run();
        void findFrontiers();

        VoxelState getVoxelState(const Eigen::Vector3d& point);
        bool getVoxelDistance(const Eigen::Vector3d& point, double& distance);
        bool getVoxelWeight(const Eigen::Vector3d& point, double& weight);
        ariitk_planning_msgs::Frontiers getFrontiers() const { return frontiers_msg_; };

        void visualizeVoxelStates();
        void visualizeFrontierPoints();
        void visualizeFrontierCenters();

    protected:
        struct Frontier {
            std::vector<Eigen::Vector3d> points;
            Eigen::Vector3d center;
        };

        inline std::string getHash(const Eigen::Vector3d& coord) {
            return std::to_string(int(coord.x() / voxel_size_)) + "," + std::to_string(int(coord.y() / voxel_size_));
        }
        inline bool inHeightRange(const double& height) { return ((slice_level_ - height) < lower_range_) && ((height - slice_level_) < upper_range_); }

        bool isFrontierVoxel(const Eigen::Vector3d& voxel);
        void findNeighbours(const std::string& key, Frontier& frontier);
        void clusterFrontiers();

        void convertFrontierToMsg(const Frontier& frontier, ariitk_planning_msgs::Frontier& msg);

        std::vector<Frontier> frontiers_;

        ariitk_planning_msgs::Frontiers frontiers_msg_;

        std::vector<Eigen::Vector3d> neighbor_voxels_;
        std::vector<Eigen::Vector3d> planar_neighbor_voxels_;

        mav_planning::PhysicalConstraints constraints_;

        voxblox::EsdfServer esdf_server_;

        Visualizer visualizer_;

        std::unordered_map<std::string, Eigen::Vector3d> hash_map_;

        double voxel_size_;
        double block_size_;
        double checking_dist_;
        double occupancy_distance_;
        double slice_level_;
        double upper_range_;
        double lower_range_;
        double min_frontier_size_;

        bool accurate_frontiers_;
        bool visualize_;

        std::string frame_id_;
};

}  // namespace ariitk::frontier_explorer
