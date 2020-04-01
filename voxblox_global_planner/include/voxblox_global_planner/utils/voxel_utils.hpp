#pragma once

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox/core/tsdf_map.h>
#include <mav_planning_common/physical_constraints.h>

#include <Eigen/Core>

namespace ariitk::global_planner {

enum VoxelState{FREE, OCCUPIED, UNKNOWN};

class VoxelUtilities {
    public:

        VoxelUtilities(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        
        unsigned char getVoxelState(const Eigen::Vector3d& point);

        bool isTraversable(const Eigen::Vector3d& position);
        bool isObserved(const Eigen::Vector3d& point);
        bool getVoxelCenter(Eigen::Vector3d& center, const Eigen::Vector3d& point);
        
        double getVoxelDistance(const Eigen::Vector3d& point);
        double getVoxelWeight(const Eigen::Vector3d& point);
        double getVoxelSize() { return voxel_size_; }
        double getMaximumWeight() { return max_weight_; }

        voxblox::Layer<voxblox::TsdfVoxel>* getTsdfMapLayerPtr();

    protected:
        mav_planning::PhysicalConstraints constraints_; 
        voxblox::EsdfServer esdf_server_;

        double voxel_size_;
        double block_size_;
        double max_weight_;
};   

} // namespace ariitk::global_planner