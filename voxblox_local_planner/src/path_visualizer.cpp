#include <voxblox_local_planner/path_visualizer.hpp>

namespace ariitk::local_planner {

void PathVisualizer::init(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) {
    nh_ = nh; nh_private_ = nh_private;
    
    nh_private_.getParam("visualize", visualize_);
    nh_private_.getParam("voxel_size", voxel_size_);

    color_map_.insert(std::make_pair(PathVisualizer::ColorType::WHITE,      Color::White()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::BLACK,      Color::Black()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::BLUE,       Color::Blue()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::ORANGE,     Color::Orange()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::YELLOW,     Color::Yellow()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::RED,        Color::Red()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::PINK,       Color::Pink()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::GREEN,      Color::Green()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::GRAY,       Color::Gray()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::TEAL,       Color::Teal()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::CHARTREUSE, Color::Chartreuse()));
    color_map_.insert(std::make_pair(PathVisualizer::ColorType::PURPLE,     Color::Purple()));
}

void PathVisualizer::createPublisher(const std::string& topic_name) {
    ros::Publisher marker_pub = nh_private_.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
    publisher_map_.insert(std::make_pair(topic_name, marker_pub));
}

void PathVisualizer::visualizePath(const std::string& topic_name, const std::vector<Eigen::Vector3d>& path, 
                       const std::string& frame_id, const ColorType& color, const double& size_factor) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_ * size_factor;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = color_map_[color];

    geometry_msgs::Point prev_center;
    prev_center.x = path[0].x();
    prev_center.y = path[0].y();
    prev_center.z = path[0].z();

    for(uint i = 1; i < path.size(); i++) {
        marker.points.push_back(prev_center);
        geometry_msgs::Point center;
        center.x = path[i].x();
        center.y = path[i].y();
        center.z = path[i].z();
        marker.points.push_back(center);
        prev_center = center;
    }
    
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);
    publisher_map_[topic_name].publish(markers);    // protect this 
}


void PathVisualizer::visualizePoints(const std::string& topic_name, const std::vector<Eigen::Vector3d>& points, 
                       const std::string& frame_id, const ColorType& color, const double& size_factor) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_ * size_factor;
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

void PathVisualizer::visualizePaths(const std::string& topic_name, const std::vector<std::vector<Eigen::Vector3d>>& paths, 
                       const std::string& frame_id, const ColorType& color, const double& size_factor) {
    visualization_msgs::MarkerArray markers;
    int path_seq = 0;

    for(auto& path : paths) { 
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = topic_name + "_" + std::to_string(++path_seq);
        marker.id = path_seq;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = marker.scale.y = marker.scale.z = voxel_size_ * size_factor;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = color_map_[color];

        geometry_msgs::Point prev_center;
        prev_center.x = path[0].x();
        prev_center.y = path[0].y();
        prev_center.z = path[0].z();

        for(uint i = 1; i < path.size(); i++) {
            marker.points.push_back(prev_center);
            geometry_msgs::Point center;
            center.x = path[i].x();
            center.y = path[i].y();
            center.z = path[i].z();
            marker.points.push_back(center);
            prev_center = center;
        }

        markers.markers.push_back(marker);
    }
    
    publisher_map_[topic_name].publish(markers);    // protect this 
}

void PathVisualizer::visualizePoint(const std::string& topic_name, const Eigen::Vector3d& point, 
                       const std::string& frame_id, const ColorType& color, const double& size_factor) {
    visualization_msgs::MarkerArray markers;
    int path_seq = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name + "_" + std::to_string(++path_seq);
    marker.id = path_seq;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.z = voxel_size_ * size_factor;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = color_map_[color];

    geometry_msgs::Point center;
    center.x = point.x();
    center.y = point.y();
    center.z = point.z();
    marker.points.push_back(center);

    markers.markers.push_back(marker);
    
    publisher_map_[topic_name].publish(markers);    // protect this 
}

void PathVisualizer::visualizeGraph(const std::string& topic_name, const Graph& graph,
                        const std::string& frame_id, const ColorType& vertex_color, const ColorType& edge_color, const double& size_factor) {
    visualization_msgs::MarkerArray markers;
    
    visualization_msgs::Marker vertex_marker;
    vertex_marker.header.frame_id = frame_id;
    vertex_marker.header.stamp = ros::Time::now();
    vertex_marker.ns = "vertices";
    vertex_marker.id = 0;
    vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
    vertex_marker.scale.x = vertex_marker.scale.y = vertex_marker.scale.z = voxel_size_ * 0.1;
    vertex_marker.action = visualization_msgs::Marker::ADD;
    vertex_marker.color = color_map_[vertex_color];

    for(auto& node : graph) {
        geometry_msgs::Point point;
        Eigen::Vector3d pos = node->getPosition();
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        vertex_marker.points.push_back(point);
    }
    markers.markers.push_back(vertex_marker);

    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = frame_id;
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.ns = "edges";
    edge_marker.id = 0;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.scale.x = edge_marker.scale.y = edge_marker.scale.z = 0.005;
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.color = color_map_[edge_color];

    for(auto& node : graph) {
        geometry_msgs::Point start_point;
        Eigen::Vector3d start_pos = node->getPosition();

        start_point.x = start_pos.x();
        start_point.y = start_pos.y();
        start_point.z = start_pos.z();

        for(auto& neigh : node->getNeighbours()) {
            geometry_msgs::Point end_point;
            Eigen::Vector3d end_pos = neigh->getPosition();

            end_point.x = end_pos.x();
            end_point.y = end_pos.y();
            end_point.z = end_pos.z();

            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(end_point);
        }
    }
    markers.markers.push_back(edge_marker);

    publisher_map_[topic_name].publish(markers);
}

void PathVisualizer::visualizeTrajectory(const std::string& topic_name, const mav_msgs::EigenTrajectoryPointVector& trajectory,
                        const std::string& frame_id, const ColorType& color, const double& size_factor) {
visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_ * size_factor;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = color_map_[color];

    geometry_msgs::Point prev_center;
    prev_center.x = trajectory[0].position_W.x();
    prev_center.y = trajectory[0].position_W.y();
    prev_center.z = trajectory[0].position_W.z();

    for(uint i = 1; i < trajectory.size(); i++) {
        marker.points.push_back(prev_center);
        geometry_msgs::Point center;
        center.x = trajectory[i].position_W.x();
        center.y = trajectory[i].position_W.y();
        center.z = trajectory[i].position_W.z();
        marker.points.push_back(center);
        prev_center = center;
    }
    
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);
    publisher_map_[topic_name].publish(markers);    // protect this
}

} // namespace ariitk::local_planner