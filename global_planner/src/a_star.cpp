#include <global_planner/a_star.hpp>

namespace ariitk::global_planner {

AStar::~AStar() {
    for(int i = 0; i < allocate_num_; i++) { delete node_pool_[i]; }
}

void AStar::setParams(const ros::NodeHandle& nh_private) {
    nh_private.getParam("resolution", resolution_);
    nh_private.getParam("time_resolution", time_resolution_);
    nh_private.getParam("lambda_heu", lambda_heu_);
    nh_private.getParam("margin", margin_);
    nh_private.getParam("allocate_num", allocate_num_);
}

void AStar::setMapPtr(const voxblox::EsdfMap::Ptr& map_ptr) { map_ptr_ = map_ptr; }

void AStar::retrievePath(const NodePtr& end_node) {
    NodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);
    
    while(cur_node->parent != NULL) {
        cur_node = cur_node->parent;
        path_nodes_.push_back(cur_node);
    }

    std::reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> AStar::getPath() {
    std::vector<Eigen::Vector3d> path;
    for(auto& node : path_nodes_) { path.push_back(node->position); }
    return path;
}

std::vector<NodePtr> AStar::getVisitedNodes() {
    std::vector<NodePtr> visited;
    visited.assign(node_pool_.begin(), node_pool_.begin() + use_node_num_ - 1);
    return visited;
}

Eigen::Vector3i AStar::posToIndex(const Eigen::Vector3d& pt) {
    Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    return idx; 
}

int AStar::timeToIndex(const double& time) {
    return floor((time - time_origin_) * inv_time_resolution_);
}

double AStar::getEuclHeu(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) {
    return tie_breaker_ * (n2 - n1).norm();
}

double AStar::getManhHeu(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) {
    double dx = fabs(n1(0) - n2(0));
    double dy = fabs(n1(1) - n2(1));
    double dz = fabs(n1(2) - n2(2));

    return tie_breaker_ * (dx + dy + dz);
}

double AStar::getDiagHeu(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) {
    double dx = fabs(n1(0) - n2(0));
    double dy = fabs(n1(1) - n2(1));
    double dz = fabs(n1(2) - n2(2));

    double diag = std::min(std::min(dx, dy), dz); // changed type to double
    double h = 1.0 * sqrt(3.0) * diag; 
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if(dx < 1e-4) {
        h = h + sqrt(2.0) * std::min(dy, dz) + 1.0 * fabs(dy - dz);
    }
    if(dy < 1e-4) {
        h = h + sqrt(2.0) * std::min(dx, dz) + 1.0 * fabs(dx - dz);
    }
    if(dz < 1e-4) {
        h = h + sqrt(2.0) * std::min(dx, dy) + 1.0 * fabs(dx - dy);
    }

    return tie_breaker_ * h;
}

void AStar::init() {
    inv_resolution_ = 1.0/resolution_; // removed this here
    inv_time_resolution_ = 1.0/time_resolution_;

    node_pool_.resize(allocate_num_);
    for(auto& node : node_pool_) { node = new Node; }

    use_node_num_ = 0;
    iter_num_ = 0;
}

AStar::Result AStar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt, const bool& dynamic, const double& time_start) {
    NodePtr cur_node = node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    Eigen::Vector3d end_state(6);
    Eigen::Vector3i end_index = posToIndex(end_pt);
    double time_to_goal;
    
    cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
    cur_node->node_state = NodeState::IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_++;

    if(dynamic) {
        time_origin_ = time_start;
        cur_node->time = time_start;
        cur_node->time_idx = timeToIndex(time_start);
        expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    } else { expanded_nodes_.insert(cur_node->index, cur_node); }

    NodePtr neighbour = NULL;
    NodePtr terminate_node = NULL;

    while(!open_set_.empty()) {
        cur_node = open_set_.top();
        
        bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;

        if(reach_end) {
            terminate_node = cur_node;
            retrievePath(terminate_node);
            has_path_ = true;

            return Result::REACH_END;
        }

        open_set_.pop();
        cur_node->node_state = NodeState::IN_CLOSE_SET;
        iter_num_++;

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d pro_pos;
        double pro_t; // was uninitialized

        std::vector<Eigen::Vector3d> inputs;
        Eigen::Vector3d d_pos;

        for(double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_) {
            for(double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_) {
                for(double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
                    d_pos << dx, dy, dz;

                    if(d_pos.norm() < 1e-3) { continue; }
                    pro_pos = cur_pos + d_pos;
                
                    if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) ||
                        pro_pos(1) <= origin_(1) || pro_pos(1) >= map_size_3d_(1) ||
                        pro_pos(2) <= origin_(2) || pro_pos(2) >= map_size_3d_(2) ) { continue; }
                
                    Eigen::Vector3i pro_id = posToIndex(pro_pos);
                    int pro_t_id = timeToIndex(pro_t);

                    NodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

                    if (pro_node != NULL && pro_node->node_state == NodeState::IN_CLOSE_SET) { continue; }

                    double dist = 0;
                    if(!map_ptr_->getDistanceAtPosition(pro_pos, &dist)) { continue; } // TODO : horizon trigger
                    
                    // double dist = edt_environment_->evaluateCoarseEDT(pro_pos, -1.0);
                    
                    if(dist <= margin_) { continue; }

                    double time_to_goal;
                    double tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
                    double tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

                    if(pro_node == NULL) {
                        pro_node = node_pool_[use_node_num_];
                        pro_node->index = pro_id;
                        pro_node->position = pro_pos;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->parent = cur_node;
                        pro_node->node_state = NodeState::IN_OPEN_SET;
                        if(dynamic) {
                            pro_node->time = cur_node->time + 1.0;
                            pro_node->time_idx = timeToIndex(pro_node->time);
                        }
                        open_set_.push(pro_node);

                    if(dynamic) { 
                        expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
                    } else {
                        expanded_nodes_.insert(pro_id, pro_node);
                    }

                    } else if (pro_node->node_state == NodeState::IN_OPEN_SET) {
                        if(tmp_g_score < pro_node->g_score) {
                            pro_node->position = pro_pos;
                            pro_node->f_score = tmp_f_score;
                            pro_node->g_score = tmp_g_score;
                            pro_node->parent = cur_node;
                            if(dynamic) pro_node->time = cur_node->time + 1.0;
                        }
                    }
                }
            }
        }
    }
    return Result::NO_PATH;
}

void AStar::reset() {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for(int i = 0; i < use_node_num_; i++) {
        NodePtr node = node_pool_[i];
        node->parent = NULL;
        node->node_state = NodeState::NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
}

} // namespace ariitk::global_planner