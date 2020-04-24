#include <voxblox_local_planner/path_finder.hpp>

namespace ariitk::local_planner {

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    region_ = Eigen::Vector3d(0.5, 1.0, 0.5); // parametrize
    region_(0) += 0.5 * (end - start).norm();
    translation_ = 0.5 * (start + end);
    ROS_WARN("CHEC");
    rotation_.col(0) = (end - translation_).normalized();
    rotation_.col(1) = rotation_.col(0).cross(Eigen::Vector3d(0,0,-1)).normalized();
    rotation_.col(2) = rotation_.col(0).cross(rotation_.col(1));
    ROS_WARN("CHECi");
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
        visualizer_.createPublisher("raw_paths");
        visualizer_.createPublisher("graph");
        visualizer_.createPublisher("best_path");
        visualizer_.createPublisher("unique_paths");
        visualizer_.createPublisher("start");
        visualizer_.createPublisher("samples");
        visualizer_.createPublisher("cast");
        visualizer_.createPublisher("end");
    }

    auto vs = voxel_size_;
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

// void PathFinder::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
//     CHECK_NOTNULL(esdf_map_ptr_);

//     // Eigen::Vector3d origin = esdf_map_ptr_->getOrigin();
//     // Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - origin_ / voxel_size_; // why?
//     // caster_.setOffset(offset);
// }

void PathFinder::findBestPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    // if(!hasLineOfSight(start_pt, end_pt)) createGraph(start_pt, end_pt);
    createGraph(start_pt, end_pt);
    visualizer_.visualizeGraph("graph", graph_);
    // ROS_WARN_STREAM("CHECK2 " << graph_.size());

    // int start = getIndex(start_pt);
    // int end = getIndex(end_pt);

    // ROS_WARN_STREAM("CJECL" << start << " " << end);
    // visualizer_.visualizePoint("start", graph_[start]->getPosition());
    // visualizer_.visualizePoint("end", graph_[end]->getPosition());

    searchPaths(0, 1);
    ROS_WARN_STREAM("CHECK3 " << raw_paths_.size());
    if(raw_paths_.size()) visualizer_.visualizePaths("raw_paths", raw_paths_, "world", PathVisualizer::ColorType::TEAL);
    // trim(raw_paths);
    // Paths unique_paths = removeDuplicates(raw_paths);
    best_candidate_path_ = evaluatePaths(raw_paths_);
    ROS_WARN("CHEBL3C");
    
    // trimPath(best_candidate_path_);
    // best_candidate_path_ = raw_paths_[0];
    if(best_candidate_path_.size()) visualizer_.visualizePath("best_path", best_candidate_path_, "world", PathVisualizer::ColorType::GREEN);
}

void PathFinder::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    graph_.clear();
    sampler_.init(start, end);
    uint max_samples = 1000;
    uint num_sample = 0;
    ROS_WARN("CHE3C");

    graph_.push_back(Node(new GraphNode(start, 0)));
    graph_.push_back(Node(new GraphNode(end, 1)));

    uint node_id = 2;
    ROS_WARN("CH32E3C");
    std::vector<Eigen::Vector3d> samples;
    while(num_sample++ < max_samples) {
        Eigen::Vector3d sample = sampler_.getSample();
        double distance = getMapDistance(sample);
        if(distance >= robot_radius_) {
            samples.push_back(sample);
            graph_.push_back(Node(new GraphNode(sample, node_id++)));
        }
    }
    ROS_WARN("CHE123C");

    visualizer_.visualizePoints("samples", samples, "world", PathVisualizer::ColorType::RED, 0.1);
    
    unsigned k = 4;
    tree_.clear();

    ROS_WARN_STREAM("CISJD");
    for(auto& node : graph_) {
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.insert(std::make_pair(pt, node->getID()));
    }

    ROS_WARN_STREAM("BEUR");
    for(auto& node : graph_) {
        std::vector<Value> neighbours;
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.query(boost::geometry::index::nearest(pt, k+1), std::back_inserter(neighbours));
        // ROS_WARN_STREAM(node->getID());
        for(auto& neighbour : neighbours) {
            if(neighbour.second != node->getID()) {
                node->addNeighbour(graph_[neighbour.second]);
            }
        }
    }
    ROS_WARN_STREAM("BEeUR");

}

void PathFinder::createGraph() {
    // CHECK_NOTNULL(esdf_map_ptr_);
    // CHECK_NOTNULL(tsdf_map_ptr_);

    uint node_id = 0;
    size_t vps = server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    voxblox::BlockIndexList blocks;
    server_.getTsdfMapPtr()->getTsdfLayerPtr()->getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
    for(const auto& index : blocks) {
        voxblox::Block<voxblox::TsdfVoxel>& block = server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockByIndex(index);
        for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d point = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
            voxblox::TsdfVoxel* voxel_ptr = block.getVoxelPtrByCoordinates(point.cast<voxblox::FloatingPoint>());
            double point_dis = getMapDistance(point);
            
            if(voxel_ptr && point_dis > robot_radius_){
                Node node = Node(new GraphNode(point, node_id++));
                graph_.push_back(node);
                
                for(auto& neighbor : neighbor_voxels_) {
                    Eigen::Vector3d pos = point + neighbor;
                    double distance = getMapDistance(pos);
                    voxblox::Point voxblox_point(pos.x(), pos.y(), pos.z());
                    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
                    
                    if(block_ptr) {
                        voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
                        if(tsdf_voxel_ptr && distance > robot_radius_) {
                            Node connector = Node(new GraphNode(pos, node_id++));
                            bool present = false;
                            for(auto& point : graph_) {
                                if((point->getPosition() - connector->getPosition()).norm() < voxel_size_) {
                                    present = true;
                                    connector = point;
                                    node_id--;
                                    break;
                                }
                            }
                            
                            node->addNeighbour(connector);
                            connector->addNeighbour(node);
                            if(present) continue;   
                            graph_.push_back(connector);
                        }
                    }
                }
            }
        }
    }
    // ROS_WARN_STREAM("CHECKp " << graph_.size());

    // pruneGraph(graph_);
    // sampler_.init(start, end);

    // uint num_sample = 0;
    // double sample_time = 0.0;

    // double max_sample_time = 10.0; // privatize
    // uint max_samples = 1000; // privatize
    
    // ros::Time last_sample_time = ros::Time::now();
    // while(sample_time < max_sample_time && num_sample++ < max_samples) {
    //     Eigen::Vector3d sample = sampler_.getSample();

    //     double distance = getMapDistance(sample);
    //     // ROS_WARN_STREAM(distance);
        
    //     if(distance <= robot_radius_) {
    //         sample_time += (ros::Time::now() - last_sample_time).toSec();
    //         last_sample_time = ros::Time::now();
    //         continue;
    //     }
    //     ROS_WARN_STREAM("sample: " << sample.x() << " " << sample.y() << " " << sample.z());

    //     Nodes visible_guards = findVisibleGuards(graph, sample);
    //     if(visible_guards.size() == 0) {
    //         Node guard = Node(new GraphNode(sample, GraphNode::NodeType::GUARD, ++node_id));
    //         graph.push_back(guard);
    //     } else if(visible_guards.size() == 2) {
    //         bool success = checkConnection(visible_guards[0], visible_guards[1], sample); // TODO
    //         if(success) {
    //             sample_time += (ros::Time::now() - last_sample_time).toSec();
    //             last_sample_time = ros::Time::now();
    //             continue;
    //         } else {
    //             Node connector = Node(new GraphNode(sample, GraphNode::NodeType::CONNECTOR, ++node_id));
    //             graph.push_back(connector);
    //             visible_guards[0]->addNeighbour(connector);
    //             visible_guards[1]->addNeighbour(connector);
    //             connector->addNeighbour(visible_guards[0]);
    //             connector->addNeighbour(visible_guards[1]);
    //         }
    //     } // else ?

    //     sample_time += (ros::Time::now() - last_sample_time).toSec();
    //     last_sample_time = ros::Time::now();

    // }

    // return graph;
}

int PathFinder::getIndex(const Eigen::Vector3d& point) {
    const double angle_step = 0.1;
    const size_t max_iterations = 10;
    double distance = 0.0;

    for(size_t step = 1; step <= max_iterations; step++) {
        for(double angle = -M_PI; angle < M_PI; angle += angle_step) {
            Eigen::Vector3d final_pos = point + Eigen::Vector3d(cos(angle), sin(angle), 0) * step * voxel_size_;
            if(getMapDistance(final_pos) >= robot_radius_) {
                // ROS_WARN_STREAM("BRU" << final_pos.x() << " " << final_pos.y() << " " << final_pos.z());
                // ROS_WARN_STREAM("INFO");
                for(auto& node : graph_) {
                    if((node->getPosition() - final_pos).norm() < voxel_size_) {
                        return node->getID();
                    }
                }
                // ROS_WARN_STREAM("INFO1");
                
                uint node_id = graph_.size();
                Node new_node = Node(new GraphNode(final_pos, node_id++));
                graph_.push_back(new_node);
                
                // ROS_WARN_STREAM("INFO2");

                for(auto& neighbor : neighbor_voxels_) {
                    Eigen::Vector3d pos = final_pos + neighbor;
                    double distance = getMapDistance(pos);
                    voxblox::Point voxblox_point(pos.x(), pos.y(), pos.z());
                    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = server_.getTsdfMapPtr()->getTsdfLayerPtr()->getBlockPtrByCoordinates(voxblox_point);
                // ROS_WARN_STREAM("INFO4");
                            
                    if(block_ptr) {
                        voxblox::TsdfVoxel* tsdf_voxel_ptr = block_ptr->getVoxelPtrByCoordinates(voxblox_point);
                        if(tsdf_voxel_ptr && distance > robot_radius_) {
                            Node connector = Node(new GraphNode(pos, node_id++));
                            bool present = false;
                            for(auto& point : graph_) {
                                if((point->getPosition() - connector->getPosition()).norm() < voxel_size_) {
                                    present = true;
                                    connector = point;
                                    break;
                                }
                            }
                            // ROS_WARN_STREAM("INFO3");
                            
                            new_node->addNeighbour(connector);
                            connector->addNeighbour(new_node);
                            if(present) continue;   
                            graph_.push_back(connector);
                        }
                    }
                }
                // ROS_WARN_STREAM("INF5");
                
                return new_node->getID();
            }
        }
    }
  return -1;
}

void PathFinder::searchPaths(const uint& start_index, const uint& end_index) {
    if(graph_.empty()) { return; }
    raw_paths_.clear();

    // std::vector<bool> visited(graph_.size(), false);
    // std::stack<uint> travel_stack;
    Eigen::Vector3d start_pos = graph_[start_index]->getPosition();
    typedef std::pair<double, uint> f_score_map;

    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(graph_.size(), DBL_MAX);
    std::vector<uint> parent(graph_.size(), INT_MAX);

    open_set.push(std::make_pair(0.0, start_index));
    g_score[start_index] = 0.0;

    // travel_stack.push(start_index);
    Path dfs_path;
    while(!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = graph_[curr_index]->getPosition();
        open_set.pop();
        // closed_set.insert(std::make_pair(curr_index, True));

        // ROS_WARN_STREAM(curr_index);
        dfs_path.push_back(graph_[curr_index]->getPosition());
        if(curr_index == end_index) {
            Path curr_path;
            while(parent[curr_index] != INT_MAX) {
                curr_path.push_back(graph_[curr_index]->getPosition());
        //         // visited[curr_index] = false;
                curr_index = parent[curr_index];
        //         // parent.erase(eraser);
        //         // ROS_WARN_STREAM("TRAV" << curr_index);
            }
            std::reverse(curr_path.begin(), curr_path.end());
        //     // ROS_WARN_STREAM("CHECKE" << curr_path.size());
            raw_paths_.push_back(curr_path);
            return;
        //     // visited[end_index] = false;
        }

        // if(!visited[curr_index]) visited[curr_index] = true;    

        for(auto& neigh : graph_[curr_index]->getNeighbours()) {
            uint neigh_index = neigh->getID();
            Eigen::Vector3d neigh_pos = neigh->getPosition();

            double score = g_score[curr_index] + (neigh_pos - curr_pos).norm();
            if(score < g_score[neigh_index]) {
            // if(!visited[neigh->getID()]) {
                g_score[neigh_index] = score;
                // if(parent.count(neigh->getID())) parent.erase(neigh->getID());
                // parent_map.insert(std::make_pair(neigh_index, curr_index));
                parent[neigh_index] = curr_index;
                open_set.push(std::make_pair(score + (neigh_pos - start_pos).norm(), neigh_index));
            }
        }
    }

    // Path curr_path;
    // uint curr_index = end_index;
    // while(parent[curr_index] != INT_MAX) {
    //     curr_path.push_back(graph_[curr_index]->getPosition());
    //     //         // visited[curr_index] = false;
    //     curr_index = parent[curr_index];
    //     //         // parent.erase(eraser);
    //     // ROS_WARN_STREAM("TRAV" << curr_index);
    // }
    // std::reverse(curr_path.begin(), curr_path.end());
    // ROS_WARN_STREAM("CHECKE" << curr_path.size());
    // if(curr_path.size()) raw_paths_.push_back(curr_path);
        //     // return;
        //     // visited[end_index] = false;

    // int min_size = INT_MAX, max_size = 0;
    // int max_raw_paths = 300; // parametrize
    // std::vector<std::vector<int>> index_map(max_raw_paths);
    // for(int i = 0; i < raw_paths_.size(); i++) {
    //     if(int(raw_paths_[i].size()) > max_size) { max_size = raw_paths_[i].size(); }
    //     if(int(raw_paths_[i].size()) < min_size) { min_size = raw_paths_[i].size(); }
    //     index_map[int(raw_paths_[i].size())].push_back(i);
    // }

    // Paths filter_paths;
    // int max_filter_paths = 25; // parametrize
    // for(int i = min_size; i <= max_size; i++) {
    //     bool reach_max = false;
    //     for(int j = 0; j < index_map[i].size(); j++) {
    //         filter_paths.push_back(raw_paths_[index_map[i][j]]);
    //         if(filter_paths.size() >= max_filter_paths) {
    //             reach_max = true;
    //             break;
    //         }
    //     }
    //     if(reach_max) break;
    // }

    // raw_paths_ = filter_paths;
    // raw_paths_.push_back(dfs_path);
    // ROS_WARN_STREAM("CHECK S " << raw_paths_.size());
    // return raw_paths_;
}

Paths PathFinder::traverseGraph(const Graph& graph) {
    Paths raw_paths;

    for(auto& node : graph) {
        std::vector<bool> visited(graph.size(), false);
        std::map<int, Node> parent;
        std::stack<Node> travel_stack;
        
        travel_stack.push(node);
        while(!travel_stack.empty()) {
            Node curr = travel_stack.top();
            travel_stack.pop();

            ROS_WARN_STREAM(curr->getID());
            if(curr->getID() == graph.back()->getID()) {
                Path curr_path;
                while(parent.count(curr->getID())) {
                    curr_path.push_back(curr->getPosition());
                    uint eraser = curr->getID();
                    curr = parent[eraser];
                    // parent.erase(eraser);
                    ROS_WARN_STREAM("TRAV" << curr->getID());
                }
                std::reverse(curr_path.begin(), curr_path.end());
                ROS_WARN_STREAM("CHECKE" << curr_path.size());
                raw_paths.push_back(curr_path);
                // visited[curr->getID()] = false;
            }

            if(!visited[curr->getID()]) visited[curr->getID()] = true;

            for(auto& neigh : curr->getNeighbours()) {
                if(!visited[neigh->getID()]) {
                    travel_stack.push(neigh);
                    // if(parent.count(neigh->getID())) parent.erase(neigh->getID());
                    parent.insert(std::make_pair(neigh->getID(), curr));
                }
            }
        }
    }
    ROS_WARN_STREAM("CHECKIN " << raw_paths.size());
    return raw_paths;
    //     Node curr = visited.back();
    // for(auto& node : curr->getNeighbours()) {
    //     if(node->getID() == 1) {
    //         Path path;
    //         for(auto& vis : visited) { path.push_back(vis->getPosition()); }
    //         path.push_back(node->getPosition());
    //         raw_paths_.push_back(path);
    //          if (raw_paths_.size() >= 100) return;
    //         // add memory limit max_num_paths
    //         ROS_WARN("CHECK") << raw_paths_.size());
    //         break;
    //     }
    // }

    //         ROS_WARN("CHECK2");
    // for(auto& node : curr->getNeighbours()) {
    //     ROS_WARN("CHECK3");
        
    //     if(node->getID() == 1) continue;


    //     bool revisit = false;
    //     for(auto& vis : visited) {
    //         if(node->getID() == vis->getID()) {
    //             revisit = true;
    //         ROS_WARN("CHECKe");

    //             break;
    //         }
    //     }
    //     if(revisit) continue;

    //     visited.push_back(node);
    //     traverseGraph(visited); 
    //     if (raw_paths_.size() >= 100) return;
    //     visited.pop_back();
    // }
}

void PathFinder::trim(Paths& paths) {
    for(auto& path : paths) { trimPath(path, 1); }
}

void PathFinder::trimPath(Path& path, const uint& iterations) {
    Path short_path = path;
    Path last_path;

    for(int k = 0; k < iterations; k++) {
        last_path = short_path;
        Path dis_path = linearizePath(short_path);
        if(dis_path.size() < 2) {
            path = dis_path;
            return;
        }

        short_path.clear();
        short_path.push_back(dis_path.front());

        for(int i = 1; i < dis_path.size(); i++) {
            Eigen::Vector3d collide_pt, gradient;

            if(hasLineOfSight(short_path.back(), dis_path[i], collide_pt, voxel_size_)) { continue; }
            if(getMapGradient(collide_pt, gradient)) {
                if(gradient.norm() > 1e-3) {
                    gradient.normalize();
                    Eigen::Vector3d direction = (dis_path[i] - short_path.back()).normalized();
                    Eigen::Vector3d push_direction = (gradient - gradient.dot(direction) * direction).normalized();
                    collide_pt += voxel_size_ * push_direction;
                }
            }
            short_path.push_back(collide_pt); // inside or outside??
        }
        short_path.push_back(dis_path.back());

        if(getPathLength(short_path) > getPathLength(last_path)) {
            short_path = last_path;
            break;
        }
    }
    
    path = short_path;
    return;
}

Paths PathFinder::removeDuplicates(Paths& paths) {
    if(paths.size() < 1) return paths;
    
    Paths pruned_paths;
    std::vector<int> exist_ids;
    exist_ids.push_back(0);

    for(int i = 1; i < paths.size(); i++) {
        bool new_path = true;
        for(auto& id : exist_ids) {
            if(checkPathSimilarity(paths[i], paths[id])) {
                new_path = false;
                break;
            }
        }
        if(new_path) { exist_ids.push_back(i); }
    }

    for(auto& id : exist_ids) { pruned_paths.push_back(paths[id]); }
    return pruned_paths;
}

Path PathFinder::evaluatePaths(const Paths& paths) {
    double min_length = DBL_MAX;
    Path best_path;
    
    for(auto& path : paths){
        double path_length = getPathLength(path);
        if(path_length < min_length) {
            best_path = path;
            min_length = path_length; 
        }
    }

    // trimPath(best_path, 5);
    return best_path;
}

double PathFinder::getMapDistance(const Eigen::Vector3d& point) {
    // CHECK(esdf_map_ptr_);
    double distance = 0.0;
    if (!server_.getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) { return 0.0; }
    return distance;
}

bool PathFinder::getMapGradient(const Eigen::Vector3d& point, Eigen::Vector3d& gradient) {
    // CHECK(esdf_map_ptr_);
    double distance;
    if (!server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(point, &distance, &gradient)) { return false; }
    return true;
}

void PathFinder::pruneGraph(Graph & graph) {
    for(auto it = graph.begin(); it != graph.end(); it++) {
        if(graph.size() <= 2) { return; }
        if((*it)->getNumNeighbours() > 1) { continue; }
        for(auto& node : graph) { node->deleteNeighbour((*it)->getID()); }
        graph.erase(it);
        it = graph.begin();
    }
}

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

bool PathFinder::hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& threshold) {
    Eigen::Vector3d ray_pt;
    return hasLineOfSight(start, end, ray_pt, threshold);
}


bool PathFinder::hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Eigen::Vector3d& point, const double& threshold) {
    if(!caster_.setInput(start / voxel_size_, end / voxel_size_)){ 
        ROS_INFO("BRUH");
        return true; 
    } // check logic

    Path cast;
    while(caster_.step(point)) {
        cast.push_back(point);
        if(getMapDistance(point) <= threshold) { return false; }
    }
    ROS_WARN_STREAM(cast.size());
    visualizer_.visualizePath("cast", cast, "world", PathVisualizer::ColorType::PINK, 0.1);
    return true;
}


bool PathFinder::checkConnection(const Node& start, const Node& end, const Eigen::Vector3d& point) {
    Path through_path = { start->getPosition(), point, end->getPosition() };
    Path direct_path(3);
    direct_path[0] = start->getPosition(); direct_path[2] = end->getPosition();

    for(auto& s_n : start->getNeighbours()) {
        for(auto& e_n : end->getNeighbours()) {
            if(s_n->getID() == e_n->getID()) {
                direct_path[1] = s_n->getPosition();
                bool same_path = checkPathSimilarity(through_path, direct_path);
                if(same_path) {
                    if(getPathLength(through_path) < getPathLength(direct_path)) { s_n->setPosition(point); }
                    return true;
                }
            }
        }
    }

    return false;
}

bool PathFinder::checkPathSimilarity(const Path& path1, const Path& path2, const double& threshold) {
    double len1 = getPathLength(path1);
    double len2 = getPathLength(path2);

    uint num_points = std::ceil(std::max(len1, len2) / voxel_size_);
    Path pts1 = discretizePath(path1, num_points);
    Path pts2 = discretizePath(path2, num_points);

    for(int i = 0; i < num_points; i++) {
        if(!hasLineOfSight(pts1[i], pts2[i], threshold)) { return false; }
    }

    return true;
}

Path PathFinder::discretizePath(const Path& path, const uint& num_points) {
    Path dis_path;
    std::vector<double> cum_length;
    cum_length.push_back(0.0);

    for(int i = 0; i < path.size() - 1; i++) {
        double inc_length = (path[i+1] - path[i]).norm();
        cum_length.push_back(inc_length);
    }
    double avg_length = cum_length.back() / double(num_points - 1);

    for(int i = 0; i < num_points; i++) {
        double curr_length = double(i) * avg_length;
        uint index = -1;
        for(int j = 0; j < cum_length.size(); j++) {
            if(curr_length >= cum_length[j] - 1e-4 && curr_length <= cum_length[j+1] + 1e-4) {
                index = j; break;
            }
        }
        double lambda = (curr_length - cum_length[index]) / (cum_length[index + 1] - cum_length[index]);
        Eigen::Vector3d interp_pt = (1 - lambda) * path[index] + lambda * path[index + 1];
        dis_path.push_back(interp_pt);
    }

    return dis_path;
}

Path PathFinder::linearizePath(const Path& path) {
    if(path.size() < 2){ return path; }
    
    Path lin_path;
    for(int i = 0; i < path.size()-1; i++) {
        Path segment = linearize(path[i], path[i+1]);
        if(segment.size() < 1){ continue; }
        lin_path.insert(lin_path.end(), segment.begin(), segment.end());
        if(i != path.size() - 2){ lin_path.pop_back(); }
    }
    
    return lin_path;
}

Path PathFinder::linearize(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    Eigen::Vector3d direction = (end - start);
    double length = direction.norm();
    uint num_segments = std::ceil(length / voxel_size_);
    Path line_points;
    if(num_segments == 0){ return line_points; }
    for(int i = 0; i <= num_segments; i++) { line_points.push_back(start + direction * (double(i) / double(num_segments))); }
    return line_points;
}

double PathFinder::getPathLength(const Path& path) {
    double length = 0.0;
    for(int i = 0; i < path.size() - 1; i++) { length += (path[i+1] - path[i]).norm(); }
    return length;
}

void PathFinder::visualizePaths() {
    if(!visualize_) { return; }
    // visualizer_.visualizePaths("raw_paths", raw_paths_, "world", PathVisualizer::ColorType::TEAL);
    visualizer_.visualizePath("best_path", best_candidate_path_, "world", PathVisualizer::ColorType::RED, 1);
    visualizer_.visualizeGraph("graph", graph_);
}

} // namespace ariitk::local_planner
