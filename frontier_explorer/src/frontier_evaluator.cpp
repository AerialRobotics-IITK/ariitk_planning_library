#include <frontier_explorer/frontier_evaluator.hpp>

namespace ariitk::frontier_explorer {

FrontierEvaluator::FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    :  esdf_server_(nh, nh_private)
    ,  visualizer_(nh, nh_private) {
    
    nh_private.getParam("accurate_frontiers", accurate_frontiers_);
    nh_private.getParam("checking_distance", checking_dist_);
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("frame_id", frame_id_);
    nh_private.getParam("surface_distance_threshold_factor", surface_distance_threshold_factor_);
    nh_private.getParam("frontier_length_factor", frontier_length_factor_);
    nh_private.getParam("slice_level", slice_level_);
    nh_private.getParam("upper_range", upper_range_);
    nh_private.getParam("lower_range", lower_range_);
    nh_private.getParam("min_frontier_size", min_frontier_size_);
    
    CHECK(esdf_server_.getTsdfMapPtr());

    visualizer_.setTsdfLayerPtr(esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr());

    constraints_.setParametersFromRos(nh_private);
    esdf_server_.setTraversabilityRadius(constraints_.robot_radius);

    voxel_size_ = esdf_server_.getTsdfMapPtr()->voxel_size();
    block_size_ = esdf_server_.getTsdfMapPtr()->block_size();

    auto vs = voxel_size_ * checking_dist_;
    if(!accurate_frontiers_) {
        neighbor_voxels_.push_back(Eigen::Vector3d(vs, 0, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(-vs, 0, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, -vs, 0));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, 0, vs));
        neighbor_voxels_.push_back(Eigen::Vector3d(0, 0, -vs));
    } else {
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

    if(visualize_) {
        visualizer_.createPublisher("frontiers");
        visualizer_.createPublisher("frontier_centers");
        visualizer_.createPublisher("free_voxels");
        visualizer_.createPublisher("occupied_voxels");
        visualizer_.createPublisher("unknown_voxels");
    }
}

void FrontierEvaluator::run() {
    frontiers_.clear();
    frontiers_msg_.frontiers.clear();
    hash_map_.clear();
    findFrontiers();
    clusterFrontiers();
    if (visualize_) {
        visualizeFrontierPoints();
        visualizeFrontierCenters();
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
        if(voxel_state == VoxelState::UNKNOWN) { return true; }
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
    } else { return VoxelState::OCCUPIED; }
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
        frontier_msg.center.x = frontier.center.x();
        frontier_msg.center.y = frontier.center.y();
        frontier_msg.center.z = frontier.center.z();
        frontier_msg.num_points = frontier.points.size();
        frontiers_msg_.frontiers.push_back(frontier_msg);
    }
}

void FrontierEvaluator::visualizeVoxelStates() {
    if(!visualize_) { return; }
    // need to shorten this too

    size_t vps = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    std::vector<Eigen::Vector3d> free_voxels, occupied_voxels, unknown_voxels;

    voxblox::BlockIndexList blocks;
    double weight = 0.0;
    esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getAllAllocatedBlocks(&blocks);
    for(const auto& index : blocks) {
        const voxblox::Block<voxblox::TsdfVoxel>& block = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);
        for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d coord = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
            if(getVoxelWeight(coord, weight) && weight > 1e-3) {
                switch(getVoxelState(coord)) {
                    case VoxelState::OCCUPIED : occupied_voxels.push_back(coord); break; 
                    case VoxelState::FREE : free_voxels.push_back(coord); break;
                    case VoxelState::UNKNOWN : unknown_voxels.push_back(coord); break;
                }
            }
        }
    }

    visualizer_.visualizeFromPoints("free_voxels", free_voxels, frame_id_, FrontierVisualizer::ColorType::BLUE);
    visualizer_.visualizeFromPoints("unknown_voxels", unknown_voxels, frame_id_, FrontierVisualizer::ColorType::TEAL);
    visualizer_.visualizeFromPoints("occupied_voxels", occupied_voxels, frame_id_, FrontierVisualizer::ColorType::PURPLE);
}

void FrontierEvaluator::visualizeFrontierPoints() {
    if(!visualize_) { return; }
    std::vector<Eigen::Vector3d> points;
    for(auto& frontier : frontiers_) {
        for(auto& point : frontier.points) { points.push_back(point); }
    }
    visualizer_.visualizeFromPoints("frontiers", points, frame_id_, FrontierVisualizer::ColorType::WHITE);
}

void FrontierEvaluator::visualizeFrontierCenters() {
    if(!visualize_) { return; }
    std::vector<Eigen::Vector3d> centers;
    for(auto& frontier : frontiers_) { centers.push_back(frontier.center); }
    visualizer_.visualizeFromPoints("frontier_centers", centers, frame_id_, FrontierVisualizer::ColorType::RED, 2.0);
}

} // namespace ariitk::frontier_explorer
