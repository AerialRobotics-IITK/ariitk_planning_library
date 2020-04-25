#include <voxblox_local_planner/path_finder.hpp>

namespace ariitk::local_planner {

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    region_ = Eigen::Vector3d(0.5, 1.0, 0.5);
    region_(0) += 0.5 * (end - start).norm();
    translation_ = 0.5 * (start + end);

    rotation_.col(0) = (end - translation_).normalized();
    rotation_.col(1) = rotation_.col(0).cross(Eigen::Vector3d(0,0,-1)).normalized();
    rotation_.col(2) = rotation_.col(0).cross(rotation_.col(1));
}

Eigen::Vector3d PointSampler::getSample() {
    Eigen::Vector3d point;
    point(0) = dist_(engine_) * region_(0);
    point(1) = dist_(engine_) * region_(1);
    point(2) = dist_(engine_) * region_(2);

    return (rotation_ * point + translation_);
}

PathFinder::PathFinder(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : sampler_()
    , server_(nh, nh_private) {
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("visualize", visualize_);
    voxel_size_ = double(server_.getEsdfMapPtr()->voxel_size());
    
    visualizer_.init(nh, nh_private);

    if(visualize_) {
        visualizer_.createPublisher("raw_path");
        visualizer_.createPublisher("graph");
        visualizer_.createPublisher("short_path");
    }

    auto vs = voxel_size_;
    for(int i = -1; i <= 1; i++) {
        for(int j = -1; j <= 1; j++) {
            for(int k = -1; k <= 1; k++) {
                if((abs(i) + abs(j) + abs(k)) == 0) { continue; }
                neighbor_voxels_.push_back(Eigen::Vector3d(i*vs, j*vs, k*vs));
            }
        }
    }
}

void PathFinder::findPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    createGraph(start_pt, end_pt);
    visualizer_.visualizeGraph("graph", graph_);

    searchPath(0, 1);
    if(raw_path_.empty()) { 
        ROS_WARN("Plan failed!");
        return;
    }

    visualizer_.visualizePath("raw_path", raw_path_, "world", PathVisualizer::ColorType::TEAL, 0.05);

    shortenPath();
    if(!short_path_.empty()){
        visualizer_.visualizePath("short_path", short_path_, "world", PathVisualizer::ColorType::GREEN, 0.5);
        path_ = short_path_;
    }
    else { path_ = raw_path_; }
}

void PathFinder::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    graph_.clear();
    sampler_.init(start, end);

    uint density = 50; // parametrize
    uint max_samples = density * (start - end).norm();
    uint num_sample = 0;

    graph_.push_back(Node(new GraphNode(start, 0)));
    graph_.push_back(Node(new GraphNode(end, 1)));

    uint node_id = 2;
    while(num_sample++ < max_samples) {
        Eigen::Vector3d sample = sampler_.getSample();
        double distance = getMapDistance(sample);
        if(distance >= robot_radius_) {
            graph_.push_back(Node(new GraphNode(sample, node_id++)));
        }
    }
    
    unsigned k = 4; // parametrize
    tree_.clear();

    for(auto& node : graph_) {
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.insert(std::make_pair(pt, node->getID()));
    }

    for(auto& node : graph_) {
        std::vector<Value> neighbours;
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.query(boost::geometry::index::nearest(pt, k+1), std::back_inserter(neighbours));
        for(auto& neighbour : neighbours) {
            if(neighbour.second != node->getID()) {
                node->addNeighbour(graph_[neighbour.second]);
            }
        }
    }
}

void PathFinder::searchPath(const uint& start_index, const uint& end_index) {
    if(graph_.empty()) { return; }
    raw_path_.clear();

    Eigen::Vector3d end_pos = graph_[end_index]->getPosition();
    typedef std::pair<double, uint> f_score_map;

    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(graph_.size(), DBL_MAX);
    std::vector<uint> parent(graph_.size(), INT_MAX);

    open_set.push(std::make_pair((end_pos - graph_[start_index]->getPosition()).norm(), start_index));
    g_score[start_index] = 0.0;

    while(!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = graph_[curr_index]->getPosition();
        open_set.pop();

        if(curr_index == end_index) {
            Path curr_path;
            while(parent[curr_index] != INT_MAX) {
                curr_path.push_back(graph_[curr_index]->getPosition());
                curr_index = parent[curr_index];
            }
            curr_path.push_back(graph_[start_index]->getPosition());
            std::reverse(curr_path.begin(), curr_path.end());
            raw_path_ = curr_path;
            return;
        }   

        for(auto& neigh : graph_[curr_index]->getNeighbours()) {
            uint neigh_index = neigh->getID();
            Eigen::Vector3d neigh_pos = neigh->getPosition();

            double score = g_score[curr_index] + (neigh_pos - curr_pos).norm();
            if(score < g_score[neigh_index]) {
                g_score[neigh_index] = score;
                parent[neigh_index] = curr_index;
                open_set.push(std::make_pair(score + (end_pos - neigh_pos).norm(), neigh_index));
            }
        }
    }
}

void PathFinder::shortenPath() {
    if(raw_path_.empty()) { return; }
    short_path_.clear();
    std::vector<bool> retain(raw_path_.size(), false);
    ROS_WARN_STREAM(raw_path_.size());
    findMaximalIndices(0, raw_path_.size()-1, &retain);
    for(uint i = 0; i < raw_path_.size(); i++) {
        if(retain[i]) short_path_.push_back(raw_path_[i]);
    }
}

void PathFinder::findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map) {
    if(start >= end) { return; }

    if(!isLineInCollision(raw_path_[start], raw_path_[end])) {
        (*map)[start] = (*map)[end] = true;
        return;
    } else {
        uint centre = (start + end)/2;
        if(centre == start || centre == end) { return; }
        findMaximalIndices(start, centre, map);
        findMaximalIndices(centre, end, map);
    }
}

bool PathFinder::isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    double distance = (end - start).norm();
    if(distance < voxel_size_) { return false; }
    Eigen::Vector3d direction = (end - start).normalized();

    Eigen::Vector3d curr_pos = start;
    double cum_dist = 0.0;

    while(cum_dist <= distance) {
        voxblox::EsdfVoxel* esdf_voxel_ptr = server_.getEsdfMapPtr()->getEsdfLayerPtr()
                ->getVoxelPtrByCoordinates(curr_pos.cast<voxblox::FloatingPoint>());
        if(esdf_voxel_ptr == nullptr) { return true; }
        if(esdf_voxel_ptr->distance < robot_radius_) { return true; }

        double step_size = std::max(voxel_size_, esdf_voxel_ptr->distance - robot_radius_);

        curr_pos += direction * step_size;
        cum_dist += step_size;
    }

    return false;
}

double PathFinder::getPathLength(const Path& path) {
    double length = 0.0;
    for(int i = 0; i < path.size() - 1; i++) { length += (path[i+1] - path[i]).norm(); }
    return length;
}

double PathFinder::getMapDistance(const Eigen::Vector3d& point) {
    double distance = 0.0;
    if (!server_.getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) { return 0.0; }
    return distance;
}

// Path PathFinder::evaluatePaths(const Paths& paths) {
//     double min_length = DBL_MAX;
//     Path best_path;
    
//     for(auto& path : paths){
//         double path_length = getPathLength(path);
//         if(path_length < min_length) {
//             best_path = path;
//             min_length = path_length; 
//         }
//     }

//     return best_path;
// }

// void PathFinder::createGraph() {
//     uint node_id = 0;
//     size_t vps = server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxels_per_side();
//     size_t num_voxels_per_block = vps * vps * vps;

//     voxblox::BlockIndexList blocks;
//     server_.getTsdfMapPtr()->getTsdfLayerPtr()->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
//     for(const auto& index : blocks) {
//         voxblox::Block<voxblox::TsdfVoxel>& block = server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);
//         for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
//             Eigen::Vector3d point = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
//             voxblox::TsdfVoxel* voxel_ptr = block.getVoxelPtrByCoordinates(point.cast<voxblox::FloatingPoint>());
//             double point_dis = getMapDistance(point);
            
//             if(voxel_ptr && point_dis > robot_radius_){
//                 Node node = Node(new GraphNode(point, node_id++));
//                 graph_.push_back(node);
                
//                 for(auto& neighbor : neighbor_voxels_) {
//                     Eigen::Vector3d pos = point + neighbor;
//                     double distance = getMapDistance(pos);
//                     voxblox::Point voxblox_point(pos.x(), pos.y(), pos.z());
//                     voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
                    
//                     if(block_ptr) {
//                         voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
//                         if(tsdf_voxel_ptr && distance > robot_radius_) {
//                             Node connector = Node(new GraphNode(pos, node_id++));
//                             bool present = false;
//                             for(auto& point : graph_) {
//                                 if((point->getPosition() - connector->getPosition()).norm() < voxel_size_) {
//                                     present = true;
//                                     connector = point;
//                                     node_id--;
//                                     break;
//                                 }
//                             }
                            
//                             node->addNeighbour(connector);
//                             connector->addNeighbour(node);
//                             if(present) continue;   
//                             graph_.push_back(connector);
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

// int PathFinder::getIndex(const Eigen::Vector3d& point) {
//     const double angle_step = 0.1;
//     const size_t max_iterations = 10;
//     double distance = 0.0;

//     for(size_t step = 1; step <= max_iterations; step++) {
//         for(double angle = -M_PI; angle < M_PI; angle += angle_step) {
//             Eigen::Vector3d final_pos = point + Eigen::Vector3d(cos(angle), sin(angle), 0) * step * voxel_size_;
//             if(getMapDistance(final_pos) >= robot_radius_) {
//                 for(auto& node : graph_) {
//                     if((node->getPosition() - final_pos).norm() < voxel_size_) {
//                         return node->getID();
//                     }
//                 }
                
//                 uint node_id = graph_.size();
//                 Node new_node = Node(new GraphNode(final_pos, node_id++));
//                 graph_.push_back(new_node);

//                 for(auto& neighbor : neighbor_voxels_) {
//                     Eigen::Vector3d pos = final_pos + neighbor;
//                     double distance = getMapDistance(pos);
//                     voxblox::Point voxblox_point(pos.x(), pos.y(), pos.z());
//                     voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
                            
//                     if(block_ptr) {
//                         voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
//                         if(tsdf_voxel_ptr && distance > robot_radius_) {
//                             Node connector = Node(new GraphNode(pos, node_id++));
//                             bool present = false;
//                             for(auto& point : graph_) {
//                                 if((point->getPosition() - connector->getPosition()).norm() < voxel_size_) {
//                                     present = true;
//                                     connector = point;
//                                     break;
//                                 }
//                             }
                            
//                             new_node->addNeighbour(connector);
//                             connector->addNeighbour(new_node);
//                             if(present) continue;   
//                             graph_.push_back(connector);
//                         }
//                     }
//                 }
                
//                 return new_node->getID();
//             }
//         }
//     }
    
//     return -1;
// }

// Paths PathFinder::traverseGraph(const Graph& graph) {
//     Paths raw_paths;

//     for(auto& node : graph) {
//         std::vector<bool> visited(graph.size(), false);
//         std::map<int, Node> parent;
//         std::stack<Node> travel_stack;
        
//         travel_stack.push(node);
//         while(!travel_stack.empty()) {
//             Node curr = travel_stack.top();
//             travel_stack.pop();

//             ROS_WARN_STREAM(curr->getID());
//             if(curr->getID() == graph.back()->getID()) {
//                 Path curr_path;
//                 while(parent.count(curr->getID())) {
//                     curr_path.push_back(curr->getPosition());
//                     uint eraser = curr->getID();
//                     curr = parent[eraser];
//                 }
//                 std::reverse(curr_path.begin(), curr_path.end());
//                 raw_paths.push_back(curr_path);
//             }

//             if(!visited[curr->getID()]) visited[curr->getID()] = true;

//             for(auto& neigh : curr->getNeighbours()) {
//                 if(!visited[neigh->getID()]) {
//                     travel_stack.push(neigh);
//                     parent.insert(std::make_pair(neigh->getID(), curr));
//                 }
//             }
//         }
//     }
//     return raw_paths;
// }

// void PathFinder::trim(Paths& paths) {
//     for(auto& path : paths) { trimPath(path, 1); }
// }

// void PathFinder::trimPath(Path& path, const uint& iterations) {
//     Path short_path = path;
//     Path last_path;

//     for(int k = 0; k < iterations; k++) {
//         last_path = short_path;
//         Path dis_path = linearizePath(short_path);
//         if(dis_path.size() < 2) {
//             path = dis_path;
//             return;
//         }

//         short_path.clear();
//         short_path.push_back(dis_path.front());

//         for(int i = 1; i < dis_path.size(); i++) {
//             Eigen::Vector3d collide_pt, gradient;

//             if(hasLineOfSight(short_path.back(), dis_path[i], collide_pt, voxel_size_)) { continue; }
//             if(getMapGradient(collide_pt, gradient)) {
//                 if(gradient.norm() > 1e-3) {
//                     gradient.normalize();
//                     Eigen::Vector3d direction = (dis_path[i] - short_path.back()).normalized();
//                     Eigen::Vector3d push_direction = (gradient - gradient.dot(direction) * direction).normalized();
//                     collide_pt += voxel_size_ * push_direction;
//                 }
//             }
//             short_path.push_back(collide_pt); // inside or outside??
//         }
//         short_path.push_back(dis_path.back());

//         if(getPathLength(short_path) > getPathLength(last_path)) {
//             short_path = last_path;
//             break;
//         }
//     }
    
//     path = short_path;
//     return;
// }

// Paths PathFinder::removeDuplicates(Paths& paths) {
//     if(paths.size() < 1) return paths;
    
//     Paths pruned_paths;
//     std::vector<int> exist_ids;
//     exist_ids.push_back(0);

//     for(int i = 1; i < paths.size(); i++) {
//         bool new_path = true;
//         for(auto& id : exist_ids) {
//             if(checkPathSimilarity(paths[i], paths[id])) {
//                 new_path = false;
//                 break;
//             }
//         }
//         if(new_path) { exist_ids.push_back(i); }
//     }

//     for(auto& id : exist_ids) { pruned_paths.push_back(paths[id]); }
//     return pruned_paths;
// }

// bool PathFinder::getMapGradient(const Eigen::Vector3d& point, Eigen::Vector3d& gradient) {
//     double distance;
//     if (!server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(point, &distance, &gradient)) { return false; }
//     return true;
// }

// void PathFinder::pruneGraph(Graph & graph) {
//     for(auto it = graph.begin(); it != graph.end(); it++) {
//         if(graph.size() <= 2) { return; }
//         if((*it)->getNumNeighbours() > 1) { continue; }
//         for(auto& node : graph) { node->deleteNeighbour((*it)->getID()); }
//         graph.erase(it);
//         it = graph.begin();
//     }
// }

// Nodes PathFinder::findVisibleGuards(const Graph& graph, const Eigen::Vector3d& point) {
//     Nodes guards;
//     int num_visible = 0;

//     for(auto& node : graph) {
//         if(node->getType() == GraphNode::NodeType::CONNECTOR) continue;
//         if(hasLineOfSight(point, node->getPosition())) {
//             guards.push_back(node);
//             if(++num_visible > 2) { break; }
//         }
//     }
//     return guards;
// }

// bool PathFinder::hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& threshold) {
//     Eigen::Vector3d ray_pt;
//     return hasLineOfSight(start, end, ray_pt, threshold);
// }


// bool PathFinder::hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Eigen::Vector3d& point, const double& threshold) {
//     if(!caster_.setInput(start / voxel_size_, end / voxel_size_)){ 
//         return true; 
//     } // check logic

//     Path cast;
//     while(caster_.step(point)) {
//         cast.push_back(point);
//         if(getMapDistance(point) <= threshold) { return false; }
//     }
//     ROS_WARN_STREAM(cast.size());
//     visualizer_.visualizePath("cast", cast, "world", PathVisualizer::ColorType::PINK, 0.1);
//     return true;
// }


// bool PathFinder::checkConnection(const Node& start, const Node& end, const Eigen::Vector3d& point) {
//     Path through_path = { start->getPosition(), point, end->getPosition() };
//     Path direct_path(3);
//     direct_path[0] = start->getPosition(); direct_path[2] = end->getPosition();

//     for(auto& s_n : start->getNeighbours()) {
//         for(auto& e_n : end->getNeighbours()) {
//             if(s_n->getID() == e_n->getID()) {
//                 direct_path[1] = s_n->getPosition();
//                 bool same_path = checkPathSimilarity(through_path, direct_path);
//                 if(same_path) {
//                     if(getPathLength(through_path) < getPathLength(direct_path)) { s_n->setPosition(point); }
//                     return true;
//                 }
//             }
//         }
//     }

//     return false;
// }

// bool PathFinder::checkPathSimilarity(const Path& path1, const Path& path2, const double& threshold) {
//     double len1 = getPathLength(path1);
//     double len2 = getPathLength(path2);

//     uint num_points = std::ceil(std::max(len1, len2) / voxel_size_);
//     Path pts1 = discretizePath(path1, num_points);
//     Path pts2 = discretizePath(path2, num_points);

//     for(int i = 0; i < num_points; i++) {
//         if(!hasLineOfSight(pts1[i], pts2[i], threshold)) { return false; }
//     }

//     return true;
// }

// Path PathFinder::discretizePath(const Path& path, const uint& num_points) {
//     Path dis_path;
//     std::vector<double> cum_length;
//     cum_length.push_back(0.0);

//     for(int i = 0; i < path.size() - 1; i++) {
//         double inc_length = (path[i+1] - path[i]).norm();
//         cum_length.push_back(inc_length);
//     }
//     double avg_length = cum_length.back() / double(num_points - 1);

//     for(int i = 0; i < num_points; i++) {
//         double curr_length = double(i) * avg_length;
//         uint index = -1;
//         for(int j = 0; j < cum_length.size(); j++) {
//             if(curr_length >= cum_length[j] - 1e-4 && curr_length <= cum_length[j+1] + 1e-4) {
//                 index = j; break;
//             }
//         }
//         double lambda = (curr_length - cum_length[index]) / (cum_length[index + 1] - cum_length[index]);
//         Eigen::Vector3d interp_pt = (1 - lambda) * path[index] + lambda * path[index + 1];
//         dis_path.push_back(interp_pt);
//     }

//     return dis_path;
// }

// Path PathFinder::linearizePath(const Path& path) {
//     if(path.size() < 2){ return path; }
    
//     Path lin_path;
//     for(int i = 0; i < path.size()-1; i++) {
//         Path segment = linearize(path[i], path[i+1]);
//         if(segment.size() < 1){ continue; }
//         lin_path.insert(lin_path.end(), segment.begin(), segment.end());
//         if(i != path.size() - 2){ lin_path.pop_back(); }
//     }
    
//     return lin_path;
// }

// Path PathFinder::linearize(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
//     Eigen::Vector3d direction = (end - start);
//     double length = direction.norm();
//     uint num_segments = std::ceil(length / voxel_size_);
//     Path line_points;
//     if(num_segments == 0){ return line_points; }
//     for(int i = 0; i <= num_segments; i++) { line_points.push_back(start + direction * (double(i) / double(num_segments))); }
//     return line_points;
// }

// void PathFinder::visualizePaths() {
//     if(!visualize_) { return; }
//     // visualizer_.visualizePaths("raw_paths", raw_paths_, "world", PathVisualizer::ColorType::TEAL);
//     visualizer_.visualizePath("best_path", best_candidate_path_, "world", PathVisualizer::ColorType::RED, 1);
//     visualizer_.visualizeGraph("graph", graph_);
// }

} // namespace ariitk::local_planner
