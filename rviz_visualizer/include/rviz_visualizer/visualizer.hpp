#pragma once

#include <ros/ros.h>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <Eigen/Eigen>

namespace ariitk::rviz_visualizer {

class Color : public std_msgs::ColorRGBA {
    public:
        Color() : std_msgs::ColorRGBA() {}
        Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
        Color(double red, double green, double blue, double alpha) : Color() {
            r = red;
            g = green;
            b = blue;
            a = alpha;
        }

    static const Color White() { return Color(1.0, 1.0, 1.0); }
    static const Color Black() { return Color(0.0, 0.0, 0.0); }
    static const Color Gray() { return Color(0.5, 0.5, 0.5); }
    static const Color Red() { return Color(1.0, 0.0, 0.0); }
    static const Color Green() { return Color(0.0, 1.0, 0.0); }
    static const Color Blue() { return Color(0.0, 0.0, 1.0); }
    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
    static const Color Orange() { return Color(1.0, 0.5, 0.0); }
    static const Color Purple() { return Color(0.5, 0.0, 1.0); }
    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
    static const Color Teal() { return Color(0.0, 1.0, 1.0); }
    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

class GraphNode {
    public:
        enum class NodeState{NEW, OPEN, CLOSE};
        GraphNode(const Eigen::Vector3d& pos, const uint& id) 
            : pos_(pos)
            , id_(id)
            , state_(NodeState::NEW) {}
        typedef std::shared_ptr<GraphNode> Ptr;
        uint getID() { return id_; }
        std::vector<GraphNode::Ptr> getNeighbours() { return neighbours_; }
        uint getNumNeighbours() { return neighbours_.size(); }
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
        NodeState state_;
        uint id_;
};

typedef GraphNode::Ptr Node;
typedef std::vector<Node> Graph;

class Visualizer {
    public:
        void init(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        void createPublisher(const std::string& topic_name);
        enum class ColorType{WHITE, BLACK, GRAY, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE, CHARTREUSE, TEAL, PINK};
        void visualizePath(const std::string& topic_name, const std::vector<Eigen::Vector3d>& path, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::PINK, const double& size_factor = 0.5);
        void visualizePaths(const std::string& topic_name, const std::vector<std::vector<Eigen::Vector3d>>& paths, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::PINK, const double& size_factor = 0.5);
        void visualizeGraph(const std::string& topic_name, const Graph& graph, const std::string& frame_id = "world",
                        const ColorType& vertex_color = ColorType::ORANGE, const ColorType& edge_color = ColorType::BLUE, const double& size_factor = 0.5);
        void visualizePoint(const std::string& topic_name, const Eigen::Vector3d& point, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::RED, const double& size_factor = 1.0);
        void visualizePoints(const std::string& topic_name, const std::vector<Eigen::Vector3d>& point, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::RED, const double& size_factor = 1.0);
        void visualizeTrajectory(const std::string& topic_name, const mav_msgs::EigenTrajectoryPointVector& trajectory, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::BLACK, const double& size_factor = 1.0);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        double voxel_size_;

        std::unordered_map<ColorType, Color> color_map_;
        std::unordered_map<std::string, ros::Publisher> publisher_map_;
};

} // namespace ariitk::rviz_visualizer