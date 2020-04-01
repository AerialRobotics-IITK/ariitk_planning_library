#include <voxblox_global_planner/frontier_evaluator/frontier_evaluator.hpp>

namespace ariitk::global_planner {

FrontierEvaluator::FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    :  esdf_server_(nh, nh_private) {
    nh_private.getParam("accurate_frontiers", accurate_frontiers_);
    nh_private.getParam("checking_distance", checking_dist_);
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("frame_id", frame_id_);

    CHECK(esdf_server_.getEsdfMapPtr());

    constraints_.setParametersFromRos(nh_private);
    esdf_server_.setTraversabilityRadius(constraints_.robot_radius);

    voxel_size_ = esdf_server_.getEsdfMapPtr()->voxel_size();
    block_size_ = esdf_server_.getEsdfMapPtr()->block_size();
    max_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private).max_weight;

    frontier_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("frontiers", 1);
    voxel_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("voxel_states", 1);

    auto vs = voxel_size_ * checking_dist_;
    ROS_INFO("%f", vs);
    if(!accurate_frontiers_) {
        neighbor_voxels_.reserve(6);
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, 0, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 0, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, -vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, 0, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, 0, -vs));
    } else {
        neighbor_voxels_.reserve(26);
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, 0, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, -vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, 0, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, vs, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, -vs, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, 0, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, vs, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, -vs, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, -vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, 0, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, vs, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, -vs, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, 0, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, vs, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, -vs, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 0, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, -vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 0, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, vs, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, -vs, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 0, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, vs, -vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, -vs, -vs));
    }
    ROS_INFO("%d", neighbor_voxels_.size());
}

bool FrontierEvaluator::isFrontierVoxel(const Eigen::Vector3d &voxel) {
    if(getVoxelState(voxel) != VoxelState::FREE) { return false; }
    unsigned char voxel_state;
    for(auto& neighbour : neighbor_voxels_) {
        voxel_state = getVoxelState(voxel + neighbour);
        if(voxel_state == VoxelState::UNKNOWN) { return true; }
    }
    return false;
}

unsigned char FrontierEvaluator::getVoxelState(const Eigen::Vector3d& point) {
    double distance = 0.0;
    if(!esdf_server_.getEsdfMapPtr()->isObserved(point)) {
        return VoxelState::UNKNOWN;
    } else if(getVoxelDistance(point, distance)){
        if(distance <= voxel_size_) {
            return VoxelState::OCCUPIED;
        } else {
            return VoxelState::FREE;
        } 
    } else {
        return VoxelState::UNKNOWN;
    }
}

bool FrontierEvaluator::getVoxelDistance(const Eigen::Vector3d& point, double& distance) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->
    getBlockPtrByCoordinates(voxblox_point);
    if(block_ptr) {
        voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
        if(tsdf_voxel_ptr) { 
            distance = tsdf_voxel_ptr->distance; 
            return true;
        } 
    }
    return false;
}

bool FrontierEvaluator::getVoxelWeight(const Eigen::Vector3d& point, double& weight) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->
    getBlockPtrByCoordinates(voxblox_point);
    if(block_ptr) {
        voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
        if(tsdf_voxel_ptr) { 
            weight = tsdf_voxel_ptr->weight; 
            return true;
        } 
    }
    return false;
}

void FrontierEvaluator::createMarkerFromFrontiers(visualization_msgs::MarkerArray* markers) {
    CHECK_NOTNULL(markers);
    
    size_t vps = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "frontier_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_/2.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    voxblox::BlockIndexList blocks;
    esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    for(const auto& index : blocks) {
        const voxblox::Block<voxblox::TsdfVoxel>& block = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);

        for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d coord = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
            if(isFrontierVoxel(coord)) {
                geometry_msgs::Point cube_center;
                cube_center.x = coord.x();
                cube_center.y = coord.y();
                cube_center.z = coord.z();
                marker.points.push_back(cube_center);
            }
        }
    }   
    markers->markers.push_back(marker);
}

void FrontierEvaluator::createMarkerFromVoxelStates(visualization_msgs::MarkerArray* markers) {
    CHECK_NOTNULL(markers);

    size_t vps = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    visualization_msgs::Marker free_marker, occupied_marker, unknown_marker;

    free_marker.header.frame_id = frame_id_;
    free_marker.id = 0;
    free_marker.ns = "free_voxels";
    free_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    free_marker.scale.x = free_marker.scale.y = free_marker.scale.y = voxel_size_/2.0;
    free_marker.action = visualization_msgs::Marker::ADD;
    free_marker.color.r = 0.5; free_marker.color.g = 0.0; free_marker.color.b = 1.0; free_marker.color.a = 1.0;

    occupied_marker.header.frame_id = frame_id_;
    occupied_marker.id = 0;
    occupied_marker.ns = "occupied_voxels";
    occupied_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    occupied_marker.scale.x = occupied_marker.scale.y = occupied_marker.scale.y = voxel_size_/2.0;
    occupied_marker.action = visualization_msgs::Marker::ADD;
    occupied_marker.color.r = 1.0; occupied_marker.color.g = 0.0; occupied_marker.color.b = 0.5; occupied_marker.color.a = 1.0;

    unknown_marker.header.frame_id = frame_id_;
    unknown_marker.id = 0; 
    unknown_marker.ns = "unknown_voxels";
    unknown_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    unknown_marker.scale.x = unknown_marker.scale.y = unknown_marker.scale.y = voxel_size_/2.0;
    unknown_marker.action = visualization_msgs::Marker::ADD;
    unknown_marker.color.r = 0.0; unknown_marker.color.g = 1.0; unknown_marker.color.b = 1.0; unknown_marker.color.a = 1.0;
    
    voxblox::BlockIndexList blocks;
    double weight = 0.0;
    esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    for(const auto& index : blocks) {
        const voxblox::Block<voxblox::TsdfVoxel>& block = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);

        for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d coord = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
            bool result = getVoxelWeight(coord, weight);
            if(weight > 1e-3) {
                geometry_msgs::Point cube_center;
                cube_center.x = coord.x();
                cube_center.y = coord.y();
                cube_center.z = coord.z();

                switch(getVoxelState(coord)) {
                    case VoxelState::OCCUPIED : occupied_marker.points.push_back(cube_center); break; 
                    case VoxelState::FREE : 
                        free_marker.points.push_back(cube_center); 
                        for(auto& neighbour : neighbor_voxels_){
                            if(getVoxelState(coord + neighbour) == VoxelState::UNKNOWN) {
                                cube_center.x += neighbour.x();
                                cube_center.y += neighbour.y();
                                cube_center.z += neighbour.z();
                                unknown_marker.points.push_back(cube_center);
                            } 
                        }
                        break;
                    case VoxelState::UNKNOWN : unknown_marker.points.push_back(cube_center); break;
                }
            }
        }
    }

    markers->markers.push_back(free_marker);
    markers->markers.push_back(occupied_marker);
    markers->markers.push_back(unknown_marker);
}

void FrontierEvaluator::visualizeFrontiers() {
    if(!visualize_) { return; }
    visualization_msgs::MarkerArray frontier_marker;
    createMarkerFromFrontiers(&frontier_marker);
    frontier_pub_.publish(frontier_marker);
}

void FrontierEvaluator::visualizeVoxelStates() {
    if(!visualize_) { return; }
    visualization_msgs::MarkerArray voxel_marker;
    createMarkerFromVoxelStates(&voxel_marker);
    voxel_pub_.publish(voxel_marker);
}

} // namespace ariitk::global_planner