#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox/core/tsdf_map.h>
#include <mav_planning_common/physical_constraints.h>
#include <Eigen/Core>

namespace ariitk::global_planner {

enum VoxelState{OCCUPIED, FREE, UNKNOWN};

class FrontierEvaluator {
    public:
        FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    
        void createMarkerFromFrontiers(visualization_msgs::MarkerArray* markers);
        void visualizeFrontiers();

        void createMarkerFromVoxelStates(visualization_msgs::MarkerArray* markers);
        void visualizeVoxelStates();

        unsigned char getVoxelState(const Eigen::Vector3d& point);
        bool getVoxelDistance(const Eigen::Vector3d& point, double& distance);
        bool getVoxelWeight(const Eigen::Vector3d& point, double& weight);

    protected:
        bool isFrontierVoxel(const Eigen::Vector3d& voxel);
        
        mav_planning::PhysicalConstraints constraints_; 
        
        voxblox::EsdfServer esdf_server_;

        double voxel_size_;
        double block_size_;
        double max_weight_;
        double checking_dist_;

        std::vector<Eigen::Vector3d> neighbor_voxels_;

        bool accurate_frontiers_;
        bool visualize_;    

        std::string frame_id_;

        ros::Publisher frontier_pub_;
        ros::Publisher voxel_pub_;
};

} // namespace ariitk::global_planner