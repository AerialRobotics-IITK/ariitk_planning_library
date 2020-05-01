#include <frontier_explorer/frontier_evaluator.hpp>

namespace ariitk::frontier_explorer {

FrontierEvaluator::FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    :  esdf_server_(nh, nh_private) {
    nh_private.getParam("accurate_frontiers", accurate_frontiers_);
    nh_private.getParam("checking_distance", checking_dist_);
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("frame_id", frame_id_);
    nh_private.getParam("surface_distance_threshold_factor", surface_distance_threshold_factor_);
    nh_private.getParam("slice_level", slice_level_);
    nh_private.getParam("upper_range", upper_range_);
    nh_private.getParam("lower_range", lower_range_);
    nh_private.getParam("min_frontier_size", min_frontier_size_);
    CHECK(esdf_server_.getEsdfMapPtr());

    constraints_.setParametersFromRos(nh_private);
    esdf_server_.setTraversabilityRadius(constraints_.robot_radius);

    voxel_size_ = esdf_server_.getEsdfMapPtr()->voxel_size();
    block_size_ = esdf_server_.getEsdfMapPtr()->block_size();

    frontier_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("frontiers", 1);
    voxel_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("voxel_states", 1);

    auto vs = voxel_size_ * checking_dist_;
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

    planar_neighbor_voxels_.push_back(Eigen::Vector3d(vs, 0, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 0, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-2*vs, 0, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(2*vs, 0, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(0, vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(0, -vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(0, -2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(0, 2*vs, -0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(vs, vs, -0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(vs, -vs, -0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-vs, -vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-vs, vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(2*vs, 2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-2*vs, -2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-2*vs, 2*vs, -0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(2*vs, -2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(2*vs, vs, -0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-2*vs, vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(2*vs, -vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-2*vs, -vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(-vs, -2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(vs, 2*vs, 0));
    planar_neighbor_voxels_.push_back(Eigen::Vector3d(vs, -2*vs, -0));

    ROS_INFO("%lf", min_frontier_size_);
}

void FrontierEvaluator::run() {
    frontiers_.clear();
    frontiers_msg_.frontiers.clear();
    hash_map_.clear();
    findFrontiers();
    clusterFrontiers();
    if (visualize_) {
        visualization_msgs::MarkerArray frontier_marker;
        createMarkerFromFrontiers(&frontier_marker);
        frontier_pub_.publish(frontier_marker);
        visualizeVoxelStates();
    }
}

void FrontierEvaluator::findFrontiers() {
    size_t vps = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    for (const auto &index : blocks) {
        const voxblox::Block<voxblox::TsdfVoxel> &block = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);
        for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d coord = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
            if (isFrontierVoxel(coord)){
                std::string hash = std::to_string(int(coord.x() / voxel_size_)) + "," + std::to_string(int(coord.y()/voxel_size_));
                coord(2,0) = slice_level_;
                hash_map_[hash] = coord;
            }
        }
    }
}

bool FrontierEvaluator::isFrontierVoxel(const Eigen::Vector3d &voxel) {
    if(getVoxelState(voxel) != VoxelState::FREE) { return false; }
    if((slice_level_ - voxel(2,0)) >= lower_range_ || (voxel(2,0) - slice_level_) >= upper_range_) { return false; }
    VoxelState voxel_state;
    for(auto& neighbour : neighbor_voxels_) {
        voxel_state = getVoxelState(voxel + neighbour);
        if(voxel_state == VoxelState::UNKNOWN) { 
            return true; 
        }
    }
    return false;
}

VoxelState FrontierEvaluator::getVoxelState(const Eigen::Vector3d& point) {
    double distance = 0.0, weight = 0.0;
    if(getVoxelDistance(point, distance) && getVoxelWeight(point, weight)) {
        if(std::abs(distance) < voxel_size_ * surface_distance_threshold_factor_ && weight > 1e-3) {
            return VoxelState::OCCUPIED;
        } else if(distance >= voxel_size_) {
            return VoxelState::FREE;
        } else {
            return VoxelState::UNKNOWN;
        }
    } else {
        return VoxelState::OCCUPIED;
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

void FrontierEvaluator::clusterFrontiers() {
    while(!hash_map_.empty()) {
        Frontier frontier;
        neighbour_coords(hash_map_.begin()->first, frontier);
        if(frontier.points.size() < min_frontier_size_) { continue; }
        frontiers_.push_back(frontier);

        ariitk_planning_msgs::Frontier frontier_msg;
        convertFrontierToMsg(frontier, frontier_msg);
        frontiers_msg_.frontiers.push_back(frontier_msg);
    }
}

void FrontierEvaluator::convertFrontierToMsg(const Frontier& frontier, ariitk_planning_msgs::Frontier& msg) {
    msg.center.x = frontier.center.x();
    msg.center.y = frontier.center.y();
    msg.center.z = frontier.center.z();
    msg.num_points = frontier.points.size();
}

void FrontierEvaluator::createMarkerFromFrontiers(visualization_msgs::MarkerArray* markers) {
    CHECK_NOTNULL(markers);
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "frontier_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_/2.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    visualization_msgs::Marker center = marker;
    center.ns = "center";
    center.color.r = 0.0;
    center.scale.x = center.scale.y = center.scale.y = voxel_size_ * 2.0;

    for (const auto& frontier : frontiers_) {
        for (const auto& point : frontier.points) {
            geometry_msgs::Point cube_center;
            cube_center.x = point.x();
            cube_center.y = point.y();
            cube_center.z = point.z();
            marker.points.push_back(cube_center);
        }
        geometry_msgs::Point cube_center;
        cube_center.x = frontier.center.x();
        cube_center.y = frontier.center.y();
        cube_center.z = frontier.center.z();
        center.points.push_back(cube_center);
    }
    markers->markers.push_back(marker);
    markers->markers.push_back(center);
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

void FrontierEvaluator::visualizeVoxelStates() {
    if(!visualize_) { return; }
    visualization_msgs::MarkerArray voxel_marker;
    createMarkerFromVoxelStates(&voxel_marker);
    voxel_pub_.publish(voxel_marker);
}

} // namespace ariitk::frontier_explorer
