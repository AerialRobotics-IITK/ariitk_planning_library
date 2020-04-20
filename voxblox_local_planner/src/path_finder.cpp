#include <voxblox_local_planner/path_finder.hpp>

namespace ariitk::local_planner {

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    region_ = Eigen::Vector3d(1.0, 10.0, 1.0); // parametrize
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

void PathFinder::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    CHECK_NOTNULL(esdf_map_ptr_);
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("visualize", visualize_);
    voxel_size_ = double(esdf_map_ptr_->voxel_size());
    
    visualizer_.init(nh, nh_private);

    if(visualize_) {
        visualizer_.createPublisher("raw_paths");
        visualizer_.createPublisher("graph");
        visualizer_.createPublisher("best_path");
        visualizer_.createPublisher("unique_paths");
    }

    // Eigen::Vector3d origin = esdf_map_ptr_->getOrigin();
    Eigen::Vector3d offset = Eigen::Vector3d(0.5, 0.5, 0.5) - origin_ / voxel_size_; // why?
    caster_.setOffset(offset);
}

void PathFinder::findBestPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    graph_ = createGraph(start_pt, end_pt);
    Paths raw_paths = searchPaths(graph_);
    visualizer_.visualizePaths("raw_paths", raw_paths, "world", PathVisualizer::ColorType::TEAL);
    trim(raw_paths);
    Paths unique_paths = removeDuplicates(raw_paths);
    best_candidate_path_ = evaluatePaths(unique_paths);
    visualizer_.visualizePaths("unique_paths", unique_paths, "world", PathVisualizer::ColorType::GREEN);
}

Graph PathFinder::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    CHECK_NOTNULL(esdf_map_ptr_);
    Graph graph;

    Node start_node = Node(new GraphNode(start, GraphNode::NodeType::GUARD, 0));
    Node end_node = Node(new GraphNode(end, GraphNode::NodeType::GUARD, 1));
    
    graph.push_back(start_node);
    graph.push_back(end_node);

    sampler_.init(start, end);

    uint node_id = 1;
    uint num_sample = 0;
    double sample_time = 0.0;

    double max_sample_time = 10.0; // privatize
    uint max_samples = 1000; // privatize
    
    ros::Time last_sample_time = ros::Time::now();
    while(sample_time < max_sample_time && num_sample++ < max_samples) {
        Eigen::Vector3d sample = sampler_.getSample();
        double distance = getMapDistance(sample);
        
        if(distance <= robot_radius_) {
            sample_time += (ros::Time::now() - last_sample_time).toSec();
            last_sample_time = ros::Time::now();
            continue;
        }

        Nodes visible_guards = findVisibleGuards(graph, sample);
        if(visible_guards.size() == 0) {
            Node guard = Node(new GraphNode(sample, GraphNode::NodeType::GUARD, ++node_id));
            graph.push_back(guard);
        } else if(visible_guards.size() == 2) {
            bool success = checkConnection(visible_guards[0], visible_guards[1], sample); // TODO
            if(success) {
                sample_time += (ros::Time::now() - last_sample_time).toSec();
                last_sample_time = ros::Time::now();
                continue;
            } else {
                Node connector = Node(new GraphNode(sample, GraphNode::NodeType::CONNECTOR, ++node_id));
                graph.push_back(connector);
                visible_guards[0]->addNeighbour(connector);
                visible_guards[1]->addNeighbour(connector);
                connector->addNeighbour(visible_guards[0]);
                connector->addNeighbour(visible_guards[1]);
            }
        } // else ?

        sample_time += (ros::Time::now() - last_sample_time).toSec();
        last_sample_time = ros::Time::now();
    }

    // pruneGraph(graph);
    return graph;
}

Paths PathFinder::searchPaths(const Graph& graph) {
    raw_paths_.clear();

    Nodes visited;
    visited.push_back(graph.front());
    traverseGraph(visited);

    int min_size = INT_MAX, max_size = 0;
    int max_raw_paths = 300; // parametrize
    std::vector<std::vector<int>> index_map(max_raw_paths);
    for(int i = 0; i < raw_paths_.size(); i++) {
        if(int(raw_paths_[i].size()) > max_size) { max_size = raw_paths_[i].size(); }
        if(int(raw_paths_[i].size()) < min_size) { min_size = raw_paths_[i].size(); }
        index_map[int(raw_paths_[i].size())].push_back(i);
    }

    Paths filter_paths;
    int max_filter_paths = 25; // parametrize
    for(int i = min_size; i <= max_size; i++) {
        bool reach_max = false;
        for(int j = 0; j < index_map[i].size(); j++) {
            filter_paths.push_back(raw_paths_[index_map[i][j]]);
            if(filter_paths.size() >= max_filter_paths) {
                reach_max = true;
                break;
            }
        }
        if(reach_max) break;
    }

    raw_paths_ = filter_paths;
    return filter_paths;
}

void PathFinder::traverseGraph(Nodes& visited) {
    Node curr = visited.back();
    for(auto& node : curr->getNeighbours()) {
        if(node->getID() == 1) {
            Path path;
            for(auto& vis : visited) { path.push_back(vis->getPosition()); }
            path.push_back(node->getPosition());
            raw_paths_.push_back(path);
            // add memory limit max_num_paths
            break;
        }
    }

    for(auto& node : curr->getNeighbours()) {
        if(node->getID() == 1) continue;

        bool revisit = false;
        for(auto& vis : visited) {
            if(node->getID() == vis->getID()) {
                revisit = true;
                break;
            }
        }
        if(revisit) continue;

        visited.push_back(node);
        traverseGraph(visited); 
        // memory limits
        visited.pop_back();
    }
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

    trimPath(best_path, 5);
    return best_path;
}

double PathFinder::getMapDistance(const Eigen::Vector3d& point) {
    CHECK(esdf_map_ptr_);
    double distance = 0.0;
    if (!esdf_map_ptr_->getDistanceAtPosition(point, &distance)) { return 0.0; }
    return distance;
}

bool PathFinder::getMapGradient(const Eigen::Vector3d& point, Eigen::Vector3d& gradient) {
    CHECK(esdf_map_ptr_);
    double distance;
    if (!esdf_map_ptr_->getDistanceAndGradientAtPosition(point, &distance, &gradient)) { return false; }
    return true;
}

void PathFinder::pruneGraph(Graph & graph) {
    for(auto it = graph.begin(); it != graph.end(); it++) {
        if(graph.size() <= 2) { return; }
        if((*it)->getID() <= 1 || (*it)->getNumNeighbours() > 1) { continue; }
        for(auto& node : graph) { node->deleteNeighbour((*it)->getID()); }
        graph.erase(it);
        it = graph.begin();
    }
}

Nodes PathFinder::findVisibleGuards(const Graph& graph, const Eigen::Vector3d& point) {
    Nodes guards;
    int num_visible = 0;

    for(auto& node : graph) {
        if(node->getType() == GraphNode::NodeType::CONNECTOR) continue;
        if(hasLineOfSight(point, node->getPosition())) {
            guards.push_back(node);
            if(++num_visible > 2) { break; }
        }
    }
    return guards;
}

bool PathFinder::hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double& threshold) {
    Eigen::Vector3d ray_pt;
    return hasLineOfSight(start, end, ray_pt, threshold);
}


bool PathFinder::hasLineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Eigen::Vector3d& point, const double& threshold) {
    if(!caster_.setInput(start / voxel_size_, end / voxel_size_)){ return true; } // check logic

    while(caster_.step(point)) {
        if(getMapDistance(point) <= threshold) { return false; }
    }

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
