#include <frontier_explorer/frontier_visualizer.hpp>

namespace ariitk::frontier_explorer {

FrontierVisualizer::FrontierVisualizer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
    :   nh_(nh)
    ,   nh_private_(nh_private)
    ,   visualize_(false) {
    nh_.getParam("visualize", visualize_);

    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::WHITE,      Color::White()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::BLACK,      Color::Black()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::BLUE,       Color::Blue()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::ORANGE,     Color::Orange()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::YELLOW,     Color::Yellow()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::RED,        Color::Red()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::PINK,       Color::Pink()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::GREEN,      Color::Green()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::GRAY,       Color::Gray()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::TEAL,       Color::Teal()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::CHARTREUSE, Color::Chartreuse()));
    color_map_.insert(std::make_pair(FrontierVisualizer::ColorType::PURPLE,     Color::Purple()));
}

void FrontierVisualizer::createPublisher(const std::string& topic_name) {
    ros::Publisher marker_pub = nh_private_.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
    publisher_map_.insert(std::make_pair(topic_name, marker_pub));
}

void FrontierVisualizer::visualizeFromLayer(const std::string& topic_name, const ShouldVisualizeFunctionType& vis_function
                                ,  const std::string& frame_id, const ColorType& color) {
    CHECK_NOTNULL(tsdf_layer_ptr_);
    
    size_t vps = tsdf_layer_ptr_->voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = tsdf_layer_ptr_->voxel_size()/2.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = color_map_[color];

    voxblox::BlockIndexList blocks;
    tsdf_layer_ptr_->getAllAllocatedBlocks(&blocks);
    for(const auto& index : blocks) {
        voxblox::Block<voxblox::TsdfVoxel>& block = tsdf_layer_ptr_->getBlockByIndex(index);
        for(size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index) {
            Eigen::Vector3d point = block.computeCoordinatesFromLinearIndex(linear_index).cast<double>();
            voxblox::TsdfVoxel* voxel_ptr = block.getVoxelPtrByCoordinates(point.cast<voxblox::FloatingPoint>());
            if(voxel_ptr && voxel_ptr->weight > 1e-3 && vis_function(point)) {
                geometry_msgs::Point center;
                center.x = point.x();
                center.y = point.y();
                center.z = point.z();
                marker.points.push_back(center);
            }
        }
    }

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);
    publisher_map_[topic_name].publish(markers);    // protect this 
}

void FrontierVisualizer::visualizeFromPoints(const std::string& topic_name, const std::vector<Eigen::Vector3d>& points, 
                       const std::string& frame_id, const ColorType& color, const double& size_factor) {
    CHECK_NOTNULL(tsdf_layer_ptr_);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = tsdf_layer_ptr_->voxel_size() * size_factor;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = color_map_[color];

    for(auto& point : points) {
        geometry_msgs::Point center;
        center.x = point.x();
        center.y = point.y();
        center.z = point.z();
        marker.points.push_back(center);
    }
    
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);
    publisher_map_[topic_name].publish(markers);    // protect this 
}

} // namespace ariitk::frontier_explorer
