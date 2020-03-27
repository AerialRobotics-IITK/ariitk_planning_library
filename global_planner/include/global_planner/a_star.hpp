#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <queue>
#include <boost/functional/hash.hpp>
#include <voxblox/core/esdf_map.h>

namespace ariitk::global_planner {

enum class NodeState { IN_CLOSE_SET = 1, IN_OPEN_SET = 2, NOT_EXPAND = 3};

struct Node {
    Eigen::Vector3i index;
    Eigen::Vector3d position;

    double g_score;
    double f_score;
    double time;
    
    int time_idx;
    Node* parent;
    
    NodeState node_state;

    Node() {
        parent = NULL;
        node_state = NodeState::NOT_EXPAND;
    }
};
typedef Node* NodePtr;

struct NodeComparator {
    bool operator()(const NodePtr& node1, const NodePtr& node2){
        return node1->f_score > node2->f_score;
    }
};

template <typename T>
struct matrix_hash : std::unary_function<T, std::size_t> {
    std::size_t operator()(T const& matrix) const {
        std::size_t seed = 0;
        for(std::size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2); 
        }
        return seed;
    }
};

class NodeHashTable {
    public:
        void insert(const Eigen::Vector3i& idx, const NodePtr& node) {
            data_3d_.insert(std::make_pair(idx, node));
        }
        void insert(const Eigen::Vector3i& idx, const int& time_idx, const NodePtr& node) {
            data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
        }

        NodePtr find(const Eigen::Vector3i& idx) {
            auto iter = data_3d_.find(idx);
            return iter == data_3d_.end() ? NULL : iter->second;
        }
        NodePtr find(const Eigen::Vector3i& idx, const int& time_idx) {
            auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
            return iter == data_4d_.end() ? NULL : iter->second;
        }

        void clear() {
            data_3d_.clear();
            data_4d_.clear();
        }
    
    private:
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
        std::unordered_map<Eigen::Vector4i, NodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;
};

class AStar {
    public:
        AStar() {};
        ~AStar();
        
        enum class Result { REACH_END = 1, NO_PATH = 2 };
        
        typedef std::shared_ptr<AStar> Ptr;

        void init();
        void reset();
        
        void setParams(const ros::NodeHandle& nh_private);
        void setMapPtr(const voxblox::EsdfMap::Ptr& map_ptr);
        
        Result search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt, const bool& dynamic = false, const double& time_start = -1.0);
        
        std::vector<Eigen::Vector3d> getPath();
        std::vector<NodePtr> getVisitedNodes();

    private:
        std::vector<NodePtr> node_pool_;
        std::vector<NodePtr> path_nodes_;

        int use_node_num_;
        int iter_num_;
        
        NodeHashTable expanded_nodes_;
        
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;

        bool has_path_ = false;

        double lambda_heu_ = 1.0;
        double margin_ = 1.0;
        double tie_breaker_ = 1.0 + 1.0/10000;
        
        int allocate_num_ = 1000;

        double resolution_ = 1.0;
        double inv_resolution_ = 1.0;
        double time_resolution_ = 1.0;
        double inv_time_resolution_ = 1.0;

        voxblox::EsdfMap::Ptr map_ptr_;

        Eigen::Vector3d origin_;
        Eigen::Vector3d map_size_3d_;

        double time_origin_ = 0;

        Eigen::Vector3i posToIndex(const Eigen::Vector3d& pt);
        
        int timeToIndex(const double& time);

        void retrievePath(const NodePtr& end_node);
        
        double getDiagHeu(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2);
        double getManhHeu(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2);
        double getEuclHeu(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2);
};

} // namespace ariitk::global_planner