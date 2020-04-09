#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <voxblox/core/tsdf_map.h>
#include <Eigen/Eigen>

namespace ariitk::frontier_explorer {

using ShouldVisualizeVoxelFunctionType = std::function<bool(const voxblox::TsdfVoxel& voxel, const voxblox::Point& coord)>;
using ShouldVisualizeFunctionType = std::function<bool(const Eigen::Vector3d& coord)>;

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

class FrontierVisualizer {
    public:
        FrontierVisualizer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        void setTsdfLayerPtr(voxblox::Layer<voxblox::TsdfVoxel>* ptr){ tsdf_layer_ptr_ = ptr; };
        enum class ColorType{WHITE, BLACK, GRAY, RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE, CHARTREUSE, TEAL, PINK};
        void visualizeFromLayer(const std::string& topic_name, const ShouldVisualizeFunctionType& vis_function, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::PINK);
        void visualizeFromPoints(const std::string& topic_name, const std::vector<Eigen::Vector3d>& points, 
                       const std::string& frame_id = "world", const ColorType& color = ColorType::PINK);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_; 

        voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_ptr_;

        bool visualize_;

        std::unordered_map<ColorType, Color> color_map_;
};

} // namespace ariitk::frontier_explorer