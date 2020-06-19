#include <rviz_visualizer/visualizer.hpp>

namespace ariitk::rviz_visualizer {

void Visualizer::init(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) {
    nh_ = nh;
    nh_private_ = nh_private;

    nh_private_.getParam("voxel_size", voxel_size_);

    color_map_.insert(std::make_pair(Visualizer::ColorType::WHITE, Color::White()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::BLACK, Color::Black()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::BLUE, Color::Blue()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::ORANGE, Color::Orange()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::YELLOW, Color::Yellow()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::RED, Color::Red()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::PINK, Color::Pink()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::GREEN, Color::Green()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::GRAY, Color::Gray()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::TEAL, Color::Teal()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::CHARTREUSE, Color::Chartreuse()));
    color_map_.insert(std::make_pair(Visualizer::ColorType::PURPLE, Color::Purple()));
}

void Visualizer::createPublisher(const std::string& topic_name) {
    ros::Publisher marker_pub = nh_private_.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
    publisher_map_.insert(std::make_pair(topic_name, marker_pub));
}

visualization_msgs::Marker Visualizer::createMarker(const std::string& ns,
    const ColorType& color,
    const double& scale,
    const uint& type,
    const std::string& frame_id,
    const uint& id,
    const uint& action) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.scale.x = marker.scale.y = marker.scale.z = scale;
    marker.action = action;
    marker.color = color_map_[color];

    return marker;
}

geometry_msgs::Point Visualizer::convertEigenToGeometryMsg(const Eigen::Vector3d& point) {
    geometry_msgs::Point point_msg;

    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();

    return point_msg;
}

void Visualizer::publishMarkers(const std::string& topic_name, const visualization_msgs::MarkerArray& markers) {
    if (publisher_map_.find(topic_name) == publisher_map_.end()) {
        ROS_ERROR_STREAM("No publisher found for topic: " << topic_name);
        return;
    }

    publisher_map_[topic_name].publish(markers);
}

void Visualizer::publishMarker(const std::string& topic_name, const visualization_msgs::Marker& marker) {
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    publishMarkers(topic_name, marker_array);
}

void Visualizer::visualizePath(const std::string& topic_name,
    const std::vector<Eigen::Vector3d>& path,
    const std::string& frame_id,
    const ColorType& color,
    const double& size_factor) {
    if (path.empty()) { return; }

    visualization_msgs::Marker path_marker = createMarker(topic_name, color, voxel_size_ * size_factor, visualization_msgs::Marker::LINE_LIST, frame_id);
    geometry_msgs::Point prev_center = convertEigenToGeometryMsg(path[0]);

    for (uint i = 1; i < path.size(); i++) {
        path_marker.points.push_back(prev_center);
        geometry_msgs::Point center = convertEigenToGeometryMsg(path[i]);
        path_marker.points.push_back(center);
        prev_center = center;
    }

    publishMarker(topic_name, path_marker);
}

void Visualizer::visualizePoints(const std::string& topic_name,
    const std::vector<Eigen::Vector3d>& points,
    const std::string& frame_id,
    const ColorType& color,
    const double& size_factor) {
    if (points.empty()) { return; }

    visualization_msgs::Marker points_marker = createMarker(topic_name, color, voxel_size_ * size_factor, visualization_msgs::Marker::SPHERE_LIST, frame_id);
    for (auto& point : points) { points_marker.points.push_back(convertEigenToGeometryMsg(point)); }
    publishMarker(topic_name, points_marker);
}

void Visualizer::visualizePaths(const std::string& topic_name,
    const std::vector<std::vector<Eigen::Vector3d>>& paths,
    const std::string& frame_id,
    const ColorType& color,
    const double& size_factor) {
    if (paths.empty()) { return; }

    visualization_msgs::MarkerArray path_markers;
    int path_seq = 0;

    for (auto& path : paths) {
        visualization_msgs::Marker path_marker = createMarker(
            topic_name + "_" + std::to_string(++path_seq), color, voxel_size_ * size_factor, visualization_msgs::Marker::LINE_LIST, frame_id, path_seq);
        geometry_msgs::Point prev_center = convertEigenToGeometryMsg(path[0]);

        for (uint i = 1; i < path.size(); i++) {
            path_marker.points.push_back(prev_center);
            geometry_msgs::Point center = convertEigenToGeometryMsg(path[i]);
            path_marker.points.push_back(center);
            prev_center = center;
        }

        path_markers.markers.push_back(path_marker);
    }

    publishMarkers(topic_name, path_markers);
}

void Visualizer::visualizePoint(const std::string& topic_name,
    const Eigen::Vector3d& point,
    const std::string& frame_id,
    const ColorType& color,
    const double& size_factor) {
    visualization_msgs::Marker point_marker = createMarker(topic_name, color, voxel_size_ * size_factor, visualization_msgs::Marker::SPHERE_LIST, frame_id);
    point_marker.points.push_back(convertEigenToGeometryMsg(point));
    publishMarker(topic_name, point_marker);
}

void Visualizer::visualizeGraph(const std::string& topic_name,
    const Graph& graph,
    const std::string& frame_id,
    const ColorType& vertex_color,
    const ColorType& edge_color,
    const double& size_factor) {
    if (graph.empty()) { return; }
    visualization_msgs::MarkerArray graph_markers;

    visualization_msgs::Marker vertex_marker =
        createMarker("vertices", vertex_color, voxel_size_ * size_factor, visualization_msgs::Marker::CUBE_LIST, frame_id);
    for (auto& node : graph) { vertex_marker.points.push_back(convertEigenToGeometryMsg(node->getPosition())); }
    graph_markers.markers.push_back(vertex_marker);

    visualization_msgs::Marker edge_marker =
        createMarker("edges", edge_color, 0.05 * voxel_size_ * size_factor, visualization_msgs::Marker::LINE_LIST, frame_id);
    for (auto& node : graph) {
        geometry_msgs::Point start_point = convertEigenToGeometryMsg(node->getPosition());

        for (auto& neigh : node->getNeighbours()) {
            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(convertEigenToGeometryMsg(neigh->getPosition()));
        }
    }
    graph_markers.markers.push_back(edge_marker);

    publishMarkers(topic_name, graph_markers);
}

void Visualizer::visualizeTrajectory(const std::string& topic_name,
    const mav_msgs::EigenTrajectoryPointVector& trajectory,
    const std::string& frame_id,
    const ColorType& color,
    const double& size_factor) {
    if (trajectory.empty()) { return; }

    visualization_msgs::Marker traj_marker = createMarker(topic_name, color, voxel_size_ * size_factor, visualization_msgs::Marker::LINE_LIST, frame_id);
    geometry_msgs::Point prev_center = convertEigenToGeometryMsg(trajectory[0].position_W);

    for (uint i = 1; i < trajectory.size(); i++) {
        traj_marker.points.push_back(prev_center);
        geometry_msgs::Point center = convertEigenToGeometryMsg(trajectory[i].position_W);
        ;
        traj_marker.points.push_back(center);
        prev_center = center;
    }

    publishMarker(topic_name, traj_marker);
}

}  // namespace ariitk::rviz_visualizer
