#include <voxblox_local_planner/path_finder.hpp>

namespace ariitk::local_planner {

void PointSampler::init(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    region_ = Eigen::Vector3d(0.5, 1.0, 0.5);
    region_(0) += 0.5 * (end - start).norm();
    translation_ = 0.5 * (start + end);

    rotation_.col(0) = (end - translation_).normalized();
    rotation_.col(1) = rotation_.col(0).cross(Eigen::Vector3d(0, 0, -1)).normalized();
    rotation_.col(2) = rotation_.col(0).cross(rotation_.col(1));
}

Eigen::Vector3d PointSampler::getSample() {
    Eigen::Vector3d point;
    point(0) = dist_(engine_) * region_(0);
    point(1) = dist_(engine_) * region_(1);
    point(2) = dist_(engine_) * region_(2);

    return (rotation_ * point + translation_);
}

void PointSampler::expandRegion(const double& size) { region_(1) += size; }

PathFinder::PathFinder(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : sampler_()
    , server_(nh, nh_private)
    , p_sample_(10)
    , expand_region_(false)
    , expand_size_(5)
    , inc_density_(false)
    , density_factor_(1.0)
    , inflate_radius_(false)
    , inflate_factor_(1.0) {
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("num_neighbours", num_neighbours_);

    voxel_size_ = double(server_.getEsdfMapPtr()->voxel_size());

    if (visualize_) {
        visualizer_.init(nh, nh_private);
        visualizer_.createPublisher("raw_path");
        visualizer_.createPublisher("graph");
        visualizer_.createPublisher("short_path");
    }

    auto vs = voxel_size_;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++) {
                if ((abs(i) + abs(j) + abs(k)) == 0) { continue; }
                neighbor_voxels_.push_back(Eigen::Vector3d(i * vs, j * vs, k * vs));
            }
        }
    }
}

void PathFinder::findPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    createGraph(start_pt, end_pt);
    if (visualize_) { visualizer_.visualizeGraph("graph", convertGraph(graph_)); }

    searchPath(0, 1);
    if (raw_path_.empty()) {
        ROS_WARN("Plan failed! Adding more nodes!");

        increaseSamplingDensity(4.0);
        expandSamplingRegion(2.0);
        createGraph(start_pt, end_pt);

        if (visualize_) { visualizer_.visualizeGraph("graph", convertGraph(graph_)); }
        searchPath(0, 1);
    }

    if (visualize_) { visualizer_.visualizePath("raw_path", raw_path_, "world", Visualizer::ColorType::TEAL, 0.05); }

    shortenPath();
    if (!short_path_.empty()) {
        if (visualize_) { visualizer_.visualizePath("short_path", short_path_, "world", Visualizer::ColorType::GREEN, 0.1); }
        path_ = short_path_;
    } else {
        path_ = raw_path_;
    }
}

void PathFinder::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    graph_.clear();
    sampler_.init(start, end);

    double p_sample = p_sample_;
    double robot_radius = robot_radius_;

    if (inc_density_) {
        p_sample *= density_factor_;
        inc_density_ = false;  // use and throw
    }

    if (expand_region_) {
        sampler_.expandRegion(expand_size_);
        expand_region_ = false;  // use and throw
    }

    if (inflate_radius_) {
        robot_radius *= inflate_factor_;
        inflate_radius_ = false;  // use and throw
    }

    uint max_samples = p_sample * (start - end).norm() * sampler_.getWidth() / voxel_size_;
    uint num_sample = 0;

    graph_.push_back(Node(new GraphNode(start, 0)));
    graph_.push_back(Node(new GraphNode(end, 1)));

    uint node_id = 2;
    while (num_sample++ < max_samples) {
        Eigen::Vector3d sample = sampler_.getSample();
        double distance = 0.0;
        if (getMapDistance(sample, distance) && distance >= robot_radius) { graph_.push_back(Node(new GraphNode(sample, node_id++))); }
    }

    tree_.clear();
    for (auto& node : graph_) {
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.insert(std::make_pair(pt, node->getID()));
    }

    for (auto& node : graph_) {
        std::vector<Value> neighbours;
        Point pt = Point(node->getPosition().x(), node->getPosition().y(), node->getPosition().z());
        tree_.query(boost::geometry::index::nearest(pt, (unsigned) num_neighbours_ + 1), std::back_inserter(neighbours));
        for (auto& neighbour : neighbours) {
            if (neighbour.second != node->getID()) { node->addNeighbour(graph_[neighbour.second]); }
        }
    }
}

void PathFinder::searchPath(const uint& start_index, const uint& end_index) {
    if (graph_.empty()) { return; }
    raw_path_.clear();

    Eigen::Vector3d end_pos = graph_[end_index]->getPosition();
    typedef std::pair<double, uint> f_score_map;

    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(graph_.size(), DBL_MAX);
    std::vector<uint> parent(graph_.size(), INT_MAX);

    open_set.push(std::make_pair((end_pos - graph_[start_index]->getPosition()).norm(), start_index));
    g_score[start_index] = 0.0;

    while (!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = graph_[curr_index]->getPosition();
        open_set.pop();

        if (curr_index == end_index) {
            Path curr_path;
            while (parent[curr_index] != INT_MAX) {
                curr_path.push_back(graph_[curr_index]->getPosition());
                curr_index = parent[curr_index];
            }

            curr_path.push_back(graph_[start_index]->getPosition());
            std::reverse(curr_path.begin(), curr_path.end());
            raw_path_ = curr_path;
            return;
        }

        for (auto& neigh : graph_[curr_index]->getNeighbours()) {
            uint neigh_index = neigh->getID();
            Eigen::Vector3d neigh_pos = neigh->getPosition();

            double score = g_score[curr_index] + (neigh_pos - curr_pos).norm();
            if (score < g_score[neigh_index]) {
                g_score[neigh_index] = score;
                parent[neigh_index] = curr_index;
                open_set.push(std::make_pair(score + (end_pos - neigh_pos).norm(), neigh_index));
            }
        }
    }
}

void PathFinder::shortenPath() {
    if (raw_path_.empty()) { return; }
    short_path_.clear();
    std::vector<bool> retain(raw_path_.size(), false);

    findMaximalIndices(0, raw_path_.size() - 1, &retain);
    for (uint i = 0; i < raw_path_.size(); i++) {
        if (retain[i]) short_path_.push_back(raw_path_[i]);
    }
}

void PathFinder::findMaximalIndices(const uint& start, const uint& end, std::vector<bool>* map) {
    if (start >= end) { return; }

    if (!isLineInCollision(raw_path_[start], raw_path_[end])) {
        (*map)[start] = (*map)[end] = true;
        return;
    } else {
        if (start == end) { return; }
        uint centre = (start + end) / 2;
        findMaximalIndices(start, centre, map);
        findMaximalIndices(centre, end, map);
    }
}

bool PathFinder::isLineInCollision(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    double distance = (end - start).norm();
    if (distance < voxel_size_) { return false; }
    Eigen::Vector3d direction = (end - start).normalized();

    Eigen::Vector3d curr_pos = start;
    double cum_dist = 0.0;

    while (cum_dist <= distance) {
        voxblox::EsdfVoxel* esdf_voxel_ptr = server_.getEsdfMapPtr()->getEsdfLayerPtr()->getVoxelPtrByCoordinates(curr_pos.cast<voxblox::FloatingPoint>());
        if (esdf_voxel_ptr == nullptr) { return true; }
        if (esdf_voxel_ptr->distance < robot_radius_) { return true; }

        double step_size = std::max(voxel_size_, esdf_voxel_ptr->distance - robot_radius_);

        curr_pos += direction * step_size;
        cum_dist += step_size;
    }
    return false;
}

double PathFinder::getPathLength(const Path& path) {
    double length = 0.0;
    for (int i = 0; i < path.size() - 1; i++) { length += (path[i + 1] - path[i]).norm(); }
    return length;
}

double PathFinder::getMapDistanceAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient) {
    double distance = 0.0;
    const bool kInterpolate = false;
    if (!server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(position, kInterpolate, &distance, gradient)) { return 0.0; }
    return distance;
}

void PathFinder::expandSamplingRegion(const double& size) {
    expand_region_ = true;
    expand_size_ = size;
}

void PathFinder::increaseSamplingDensity(const double& factor) {
    inc_density_ = true;
    density_factor_ = factor;
}

void PathFinder::inflateRadius(const double& factor) {
    inflate_radius_ = true;
    inflate_factor_ = factor;
}

ariitk::rviz_visualizer::Graph PathFinder::convertGraph(const Graph& graph) {
    ariitk::rviz_visualizer::Graph ret_graph;
    for (auto& node : graph) { ret_graph.push_back(ariitk::rviz_visualizer::Node(new ariitk::rviz_visualizer::GraphNode(node->getPosition(), node->getID()))); }
    for (auto& node : graph) {
        for (auto& neigh : node->getNeighbours()) { ret_graph[node->getID()]->addNeighbour(ret_graph[neigh->getID()]); }
    }
    return ret_graph;
}

}  // namespace ariitk::local_planner
