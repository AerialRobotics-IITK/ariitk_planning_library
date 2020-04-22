#pragma once 

#include <Eigen/Eigen>

namespace ariitk::local_planner {

class GraphNode {
    public:
        enum class NodeType{GUARD, CONNECTOR};
        enum class NodeState{NEW, OPEN, CLOSE};
        GraphNode(const Eigen::Vector3d& pos, const NodeType& type, const uint& id) 
            : pos_(pos)
            , type_(type)
            , id_(id)
            , state_(NodeState::NEW) {}
        typedef std::shared_ptr<GraphNode> Ptr;
        uint getID() { return id_; }
        std::vector<GraphNode::Ptr> getNeighbours() { return neighbours_; }
        uint getNumNeighbours() { return neighbours_.size(); }
        NodeType getType() { return type_; }
        Eigen::Vector3d getPosition() { return pos_; }
        void setPosition(const Eigen::Vector3d& point) { pos_ = point; }
        void addNeighbour(const GraphNode::Ptr& node) { neighbours_.push_back(node); }
        void deleteNeighbour(const uint& id) { 
            for(auto it = neighbours_.begin(); it != neighbours_.end(); it++) {
                if(id == (*it)->getID()) {
                    neighbours_.erase(it);
                    return;
                }
            }
        }

    private:
        std::vector<GraphNode::Ptr> neighbours_;
        Eigen::Vector3d pos_;
        NodeType type_;
        NodeState state_;
        uint id_;
};

typedef GraphNode::Ptr Node;
typedef std::vector<Node> Graph;
// typedef std::vector<Node> Nodes;

} // namespace ariitk::local_planner
