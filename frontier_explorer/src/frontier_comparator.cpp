#include <frontier_explorer/frontier_comparator.hpp>

namespace ariitk::frontier_explorer {

FrontierComparator::FrontierComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
    : nh_(nh)
    , nh_private_(nh_private) {
    nh_private_.getParam("min_frontier_size", min_size_);
    nh_private_.getParam("clear_radius", clear_radius_);
    nh_private_.getParam("slice_level", slice_level_);
    nh_private_.getParam("voxel_size", voxel_size_);
    nh_private_.getParam("robot_radius", robot_radius_);
    nh_private_.getParam("visualize", visualize_);

    esdf_slice_sub_ = nh_.subscribe("esdf_slice", 1, &FrontierComparator::sliceCallback, this);
    odom_sub_ = nh_.subscribe("odometry", 1, &FrontierComparator::odometryCallback, this);

    if(visualize_) {
        visualizer_.init(nh, nh_private);
        visualizer_.createPublisher("graph");
    }
}

bool FrontierComparator::operator()(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2) {
    double cost1 = getDistance(f1.center);
    double cost2 = getDistance(f2.center);
    
    double gain1 = (f1.num_points/min_size_)*(f1.num_points/min_size_);
    double gain2 = (f2.num_points/min_size_)*(f2.num_points/min_size_);

    if(cost1 < clear_radius_) cost1 = DBL_MAX;
    else if(cost2 < clear_radius_) cost2 = DBL_MAX;
    else return ((gain1 - cost1) > (gain2 - cost2)); // need formal policy
}

void FrontierComparator::odometryCallback(const nav_msgs::Odometry& odom) {
    curr_pose_ = odom.pose.pose;
    distance_map_.clear();
}

void FrontierComparator::sliceCallback(const sensor_msgs::PointCloud2& msg) {
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(msg, cloud);
    pcl::PointCloud<pcl::PointXYZI> slice;
    pcl::fromPCLPointCloud2(cloud, slice);
    generateGraph(slice);
}

double FrontierComparator::getDistance(const geometry_msgs::Point& point) {
    std::string hash = getHash(point);
    if(distance_map_.find(hash) != distance_map_.end()) {
        return distance_map_[hash];
    } else {
        double distance = getPathLength(findPath(curr_pose_.position, point));
        distance_map_.insert(std::make_pair(hash, distance));
        return distance;
    }
}

double FrontierComparator::getPathLength(const Path& path) {
    double length = 0.0;
    for(auto i = 0; i < path.size() - 1; i++) {
        length += (path[i+1] - path[i]).norm();
    }
    return length;
}

Path FrontierComparator::findPath(const geometry_msgs::Point& start, const geometry_msgs::Point& end) {
    insertNode(start); insertNode(end);
    
    uint end_index = esdf_graph_.size() - 1;
    uint start_index = end_index - 1;

    typedef std::pair<double, uint> f_score_map;
    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(esdf_graph_.size(), DBL_MAX);
    std::vector<uint> parent(esdf_graph_.size(), INT_MAX);

    Eigen::Vector3d end_pos = esdf_graph_[end_index]->pos;

    open_set.push(std::make_pair((end_pos - esdf_graph_[start_index]->pos).norm(), start_index));
    g_score[start_index] = 0.0;

    while(!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = esdf_graph_[curr_index]->pos;
        open_set.pop();

        if(curr_index == end_index) {
            Path path;
            while(parent[curr_index] != INT_MAX) {
                path.push_back(esdf_graph_[curr_index]->pos);
                curr_index = parent[curr_index];
            }
            path.push_back(esdf_graph_[start_index]->pos);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for(auto& node : esdf_graph_[curr_index]->neighbours) {
            double score = g_score[curr_index] + (node->pos - curr_pos).norm();
            if(score < g_score[node->id]) {
                g_score[node->id] = score;
                parent[node->id] = curr_index;
                open_set.push(std::make_pair(score + (end_pos - node->pos).norm(), node->id));
            }
        }
    }
}

void FrontierComparator::insertNode(const geometry_msgs::Point& point) {
    esdf_graph_.push_back(Node::Ptr(new Node(convertToVoxel(point), esdf_graph_.size())));
    auto node = esdf_graph_.back();
    
    unsigned k = 8; // parametrize
    std::vector<Value> tree_neighbours;

    Point pt = Point(node->pos.x(), node->pos.y(), node->pos.z());
    esdf_tree_.query(boost::geometry::index::nearest(pt, k+1), std::back_inserter(tree_neighbours));
    
    for(auto& neighbour : tree_neighbours) {
        if(neighbour.second != node->id) {
            node->addNeighbour(esdf_graph_[neighbour.second]);
        }
    }    
}

void FrontierComparator::generateGraph(const pcl::PointCloud<pcl::PointXYZI>& slice) {
    esdf_graph_.clear();

    uint node_id = 0;
    for(auto& voxel : slice){
        if(voxel.intensity >= robot_radius_) {
            esdf_graph_.push_back(Node::Ptr(new Node(voxel, node_id++)));
        }
    }

    unsigned k = 8; // parametrize
    esdf_tree_.clear();

    for(auto& node : esdf_graph_) {
        Point pt = Point(node->pos.x(), node->pos.y(), node->pos.z());
        esdf_tree_.insert(std::make_pair(pt, node->id));
    }

    for(auto& node : esdf_graph_) {
        std::vector<Value> tree_neighbours;
        Point pt = Point(node->pos.x(), node->pos.y(), node->pos.z());
        esdf_tree_.query(boost::geometry::index::nearest(pt, k+1), std::back_inserter(tree_neighbours));
        for(auto& neighbour : tree_neighbours) {
            if(neighbour.second != node->id) {
                node->addNeighbour(esdf_graph_[neighbour.second]);
            }
        }
    }

    if(visualize_) {
        visualizer_.visualizeGraph("graph", convertGraph(esdf_graph_));
    }
}

pcl::PointXYZI FrontierComparator::convertToVoxel(const geometry_msgs::Point& point) {
    pcl::PointXYZI voxel;
    voxel.x = point.x;
    voxel.y = point.y;
    voxel.z = point.z;
    voxel.intensity = 0.0;
    return voxel;
}

ariitk::rviz_visualizer::Graph FrontierComparator::convertGraph(const Graph& graph) {
    ariitk::rviz_visualizer::Graph ret_graph;
    for(auto& node : graph) {
        ret_graph.push_back(ariitk::rviz_visualizer::Node(new ariitk::rviz_visualizer::GraphNode(node->pos, node->id)));
    }
    for(auto& node : graph) {
        for(auto& neigh : node->neighbours) {
            ret_graph[node->id]->addNeighbour(ret_graph[neigh->id]);
        }
    }
    return ret_graph;
}

} // namespace ariitk::frontier_explorer
