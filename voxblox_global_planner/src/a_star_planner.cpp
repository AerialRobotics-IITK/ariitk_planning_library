#include<voxblox_global_planner/a_star_planner.hpp>

namespace ariitk::global_planner {

GraphNode::GraphNode(const pcl::PointXYZI& point,uint id) {
  id_ = id;
  position_.x() = point.x;
  position_.y() = point.y;
  position_.z() = point.z;
}

AStarPlanner::AStarPlanner(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    voxblox_server_(nh,nh_private) {

    nh_private_.param("robot_radius",robot_radius_,0.45);
    nh_private_.param("visualize",visualize_,true);

    esdf_slice_sub_ = nh_private_.subscribe("esdf_slice",1,&AStarPlanner::esdfSliceCallback,this);
    esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_slice_out",1,true);

    visualizer_.init(nh,nh_private);
    visualizer_.createPublisher("graph");

    }

void AStarPlanner::generateGraph() {
  ROS_INFO("Hello world");
  uint i=0;
  graph_.clear();
  for(auto& point : pointcloud_.points) {
    if(point.intensity > robot_radius_) {
      graph_.push_back(Node(new GraphNode(point,i++)));
    }
  }
  
  tree_.clear();

  for(auto& node : graph_) {
    Point pt = Point(node->position_.x(),node->position_.y(),node->position_.z());
    tree_.insert(std::make_pair(pt, node->id_));
  }

  uint k=4;
  for(auto& node : graph_) {
        std::vector<Value> neighbours;
        Point pt = Point(node->position_.x(), node->position_.y(), node->position_.z());
        tree_.query(boost::geometry::index::nearest(pt, k+1), std::back_inserter(neighbours));
        for(auto& neighbour : neighbours) {
            if(neighbour.second != node->id_) {
                node->addNeighbour(graph_[neighbour.second]);
            }
        }
    }
    ROS_INFO("Hello World");
}

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

void PathVisualizer::visualizeGraph(const std::string& topic_name, const Graph& graph,
                        const std::string& frame_id, const ColorType& vertex_color, const ColorType& edge_color, const double& size_factor) {
  if(graph.empty()) {return;}

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
        Eigen::Vector3d pos = node->position_;
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
        Eigen::Vector3d start_pos = node->position_;

        start_point.x = start_pos.x();
        start_point.y = start_pos.y();
        start_point.z = start_pos.z();

        for(auto& neigh : node->neighbours_) {
            geometry_msgs::Point end_point;
            Eigen::Vector3d end_pos = neigh->position_;

            end_point.x = end_pos.x();
            end_point.y = end_pos.y();
            end_point.z = end_pos.z();

            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(end_point);
        }
        markers.markers.push_back(edge_marker);

    }
  publisher_map_[topic_name].publish(markers);
}

void AStarPlanner::esdfSliceCallback(pcl::PointCloud<pcl::PointXYZI> pointcloud) {

  pointcloud_ = pointcloud;
  esdf_slice_pub_.publish(pointcloud_);

  generateGraph();

  if(visualize_) {
      visualizer_.visualizeGraph("graph",graph_);
      }

}

}