#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox/core/tsdf_map.h>
#include <mav_planning_common/physical_constraints.h>
#include <Eigen/Core>

#include <rviz_visualizer/visualizer.hpp>
#include <ariitk_planning_msgs/Frontiers.h>

namespace ariitk::frontier_explorer {

enum class VoxelState{OCCUPIED, FREE, UNKNOWN};
typedef ariitk::rviz_visualizer::Visualizer Visualizer;

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
        struct Frontier{
            std::vector<Eigen::Vector3d> points;
            Eigen::Vector3d center;
        };
        bool isFrontierVoxel(const Eigen::Vector3d& voxel);
        inline std::string getHash(const Eigen::Vector3d& coord) { 
            return std::to_string(int(coord.x() / voxel_size_)) + "," + std::to_string(int(coord.y() / voxel_size_));
        }
        void findNeighbours(const std::string& key, Frontier& frontier);
        void clusterFrontiers();
        void convertFrontierToMsg(const Frontier& frontier, ariitk_planning_msgs::Frontier& msg);

        std::vector<Frontier> frontiers_;
        std::vector<Eigen::Vector3d> neighbor_voxels_;
        std::vector<Eigen::Vector3d> planar_neighbor_voxels_;

        mav_planning::PhysicalConstraints constraints_;

        voxblox::EsdfServer esdf_server_;

        std::unordered_map<std::string, Eigen::Vector3d> hash_map_; 

        double voxel_size_;
        double block_size_;
        double checking_dist_;
        double surface_distance_threshold_factor_;
        double slice_level_;
        double upper_range_;
        double lower_range_;
        double min_frontier_size_;

        bool accurate_frontiers_;
        bool visualize_;    

        std::string frame_id_;

        Visualizer visualizer_;
        
        ariitk_planning_msgs::Frontiers frontiers_msg_;
};

} // namespace ariitk::frontier_explorer
