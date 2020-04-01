#include <voxblox_global_planner/utils/voxel_utils.hpp>

namespace ariitk::global_planner {
    
VoxelUtilities::VoxelUtilities(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    :  esdf_server_(nh, nh_private) {
    constraints_.setParametersFromRos(nh_private);
    esdf_server_.setTraversabilityRadius(constraints_.robot_radius);

    CHECK(esdf_server_.getEsdfMapPtr());

    voxel_size_ = esdf_server_.getEsdfMapPtr()->voxel_size();
    block_size_ = esdf_server_.getEsdfMapPtr()->block_size();
    max_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private).max_weight;
}

bool VoxelUtilities::isTraversable(const Eigen::Vector3d& position) {
    double distance = 0.0;
    if(esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return (distance > constraints_.robot_radius);
    }  
    return false;
}

bool VoxelUtilities::isObserved(const Eigen::Vector3d& point) {
    return esdf_server_.getEsdfMapPtr()->isObserved(point);
}

unsigned char VoxelUtilities::getVoxelState(const Eigen::Vector3d& point) {
    double distance = 0.0;
    if(esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(point, &distance)){
        if(distance < voxel_size_) {
            return VoxelState::OCCUPIED;
        } else {
            return VoxelState::FREE;
        } 
    } else {
        return VoxelState::UNKNOWN;
    }
}

bool VoxelUtilities::getVoxelCenter(Eigen::Vector3d& center, const Eigen::Vector3d& point) {
    voxblox::BlockIndex block_id = esdf_server_.getEsdfMapPtr()->getEsdfLayerPtr()->
    computeBlockIndexFromCoordinates(point.cast<voxblox::FloatingPoint>());
    center = voxblox::getOriginPointFromGridIndex(block_id, block_size_).cast<double>();
    voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
        (point - center).cast<voxblox::FloatingPoint>(), 1.0/voxel_size_);
    center += voxblox::getCenterPointFromGridIndex(voxel_id, voxel_size_).cast<double>();
    return true;
}

double VoxelUtilities::getVoxelDistance(const Eigen::Vector3d& point) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->
    getBlockPtrByCoordinates(voxblox_point);
    if(block_ptr) {
        voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
        if(tsdf_voxel_ptr) { return tsdf_voxel_ptr->distance; } 
    }
    return 0.0;
}

double VoxelUtilities::getVoxelWeight(const Eigen::Vector3d& point) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->
    getBlockPtrByCoordinates(voxblox_point);
    if(block_ptr) {
        voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
        if(tsdf_voxel_ptr) { return tsdf_voxel_ptr->weight; } 
    }
    return 0.0;
}

voxblox::Layer<voxblox::TsdfVoxel>* VoxelUtilities::getTsdfMapLayerPtr() {
    return esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
}

} // namespace ariitk::global_planner