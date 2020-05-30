#include <frontier_explorer/frontier_evaluator.hpp>

namespace ariitk::frontier_explorer {

FrontierEvaluator::FrontierEvaluator(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    :  esdf_server_(nh, nh_private) {

    nh_private.getParam("accurate_frontiers", accurate_frontiers_);
    nh_private.getParam("checking_distance",  checking_dist_);
    nh_private.getParam("visualize",          visualize_);
    nh_private.getParam("frame_id",           frame_id_);
    nh_private.getParam("occupancy_distance", occupancy_distance_);
    nh_private.getParam("slice_level",        slice_level_);
    nh_private.getParam("upper_range",        upper_range_);
    nh_private.getParam("lower_range",        lower_range_);
    nh_private.getParam("min_frontier_size",  min_frontier_size_);

    CHECK(esdf_server_.getTsdfMapPtr());

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
        for(int i = -1; i <= 1; i++) {
            for(int j = -1; j <= 1; j++) {
                for(int k = -1; k <= 1; k++) {
                    if((abs(i) + abs(j) + abs(k)) == 0) { continue; }
                    neighbor_voxels_.push_back(Eigen::Vector3d(i*vs, j*vs, k*vs));
                }
            }
        }
    }

    for(int i = -2; i <=2; i++) {
        for(int j = -2; j <= 2; j++) {
            if((abs(i) + abs(j)) == 0) { continue; }
            planar_neighbor_voxels_.push_back(Eigen::Vector3d(i*vs, j*vs, 0));
        }
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

    for(const auto &index : blocks) {
        const voxblox::Block<voxblox::TsdfVoxel> &block = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);

        for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d coord = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();

            if(isFrontierVoxel(coord)){
                coord(2,0) = slice_level_;
                hash_map_[getHash(coord)] = coord;
            }
        }
    }
}

void FrontierEvaluator::findNeighbours(const std::string& key, Frontier& frontier) {
    auto point = hash_map_.at(key);
    hash_map_.erase(key);

    frontier.center = (frontier.center * frontier.points.size() + point) / (frontier.points.size() + 1);
    frontier.points.push_back(point);

    for(auto& next_point : planar_neighbor_voxels_) {
        auto neighbour_hash = getHash(point + next_point);
        if(hash_map_.find(neighbour_hash) != hash_map_.end()) { findNeighbours(neighbour_hash, frontier); }
    }
}

bool FrontierEvaluator::isFrontierVoxel(const Eigen::Vector3d &voxel) {
    if(getVoxelState(voxel) != VoxelState::FREE) { return false; }
    if(!inHeightRange(voxel(2,0))) { return false; }

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
        if(std::abs(distance) < voxel_size_ * occupancy_distance_ && weight > 1e-3) {
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
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);

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
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);

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
        findNeighbours(hash_map_.begin()->first, frontier);

        if(frontier.points.size() < min_frontier_size_) { continue; }
        frontiers_.push_back(frontier);

        ariitk_planning_msgs::Frontier frontier_msg;
        convertFrontierToMsg(frontier, frontier_msg);
        frontiers_msg_.frontiers.push_back(frontier_msg);
    }
}

void FrontierEvaluator::convertFrontierToMsg(const FrontierEvaluator::Frontier& frontier, ariitk_planning_msgs::Frontier& msg) {
    msg.center.x = frontier.center.x();
    msg.center.y = frontier.center.y();
    msg.center.z = frontier.center.z();
    msg.num_points = frontier.points.size();
}

void FrontierEvaluator::visualizeVoxelStates() {
    if(!visualize_) { return; }
    // TODO: expensive to do this, try to simplify

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

    visualizer_.visualizePoints("free_voxels", free_voxels, frame_id_, Visualizer::ColorType::BLUE);
    visualizer_.visualizePoints("unknown_voxels", unknown_voxels, frame_id_, Visualizer::ColorType::TEAL);
    visualizer_.visualizePoints("occupied_voxels", occupied_voxels, frame_id_, Visualizer::ColorType::PURPLE);
}

void FrontierEvaluator::visualizeFrontierPoints() {
    if(!visualize_) { return; }
    std::vector<Eigen::Vector3d> points;

    for(auto& frontier : frontiers_) {
        for(auto& point : frontier.points) { points.push_back(point); }
    }
    visualizer_.visualizePoints("frontiers", points, frame_id_, Visualizer::ColorType::WHITE);
}

void FrontierEvaluator::visualizeFrontierCenters() {
    if(!visualize_) { return; }
    std::vector<Eigen::Vector3d> centers;
    for(auto& frontier : frontiers_) { centers.push_back(frontier.center); }
    visualizer_.visualizePoints("frontier_centers", centers, frame_id_, Visualizer::ColorType::RED, 2.0);
}

} // namespace ariitk::frontier_explorer
