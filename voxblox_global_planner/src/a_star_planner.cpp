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
    frame_id_("map"),
    visualize_(true),
    verbose_(false),
    voxblox_server_(nh,nh_private),
    skeleton_generator_() {
    constraints_.setParametersFromRos(nh_private_);
    
    nh_private_.param("robot_radius",robot_radius_,0.45);
    nh_private_.param("visualize",visualize_,true);
    nh_private_.param("frame_id", frame_id_, frame_id_);

    path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
    sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);  

    esdf_slice_sub_    = nh_private_.subscribe("esdf_slice",1,&AStarPlanner::esdfSliceCallback,this);
    esdf_slice_pub_    = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_slice_out",1,true);
    waypoint_list_pub_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

    planner_srv_ = nh_private_.advertiseService(
      "plan", &AStarPlanner::plannerServiceCallback, this);
    path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &AStarPlanner::publishPathCallback, this);

    voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

    // Now set up the skeleton generator.
    ROS_INFO("Initializing skeleton generator.");
    skeletonize(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  // Set up the A* planners.
    ROS_INFO("Initializing skeleton planner.");
    skeleton_planner_.setSkeletonLayer(skeleton_generator_.getSkeletonLayer());
    skeleton_planner_.setEsdfLayer(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
    skeleton_planner_.setMinEsdfDistance(constraints_.robot_radius);  

    visualizer_.init(nh,nh_private);
    visualizer_.createPublisher("graph");
    visualizer_.createPublisher("path");

    // Set up shortener.
    ROS_INFO("Initializing path shortener.");
    path_shortener_.setEsdfLayer(
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
    path_shortener_.setConstraints(constraints_);

  }


void AStarPlanner::findPath(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    createGraph(start_pt, end_pt);
    if(visualize_) {
      visualizer_.visualizeGraph("graph", graph_);
    }

    searchPath(0, 1);

}

void AStarPlanner::searchPath(const uint& start_index, const uint& end_index) {
    if(graph_.empty()) { return; }
    path_.clear();

    Eigen::Vector3d end_pos = graph_[end_index]->position_;
    typedef std::pair<double, uint> f_score_map;

    std::priority_queue<f_score_map, std::vector<f_score_map>, std::greater<f_score_map>> open_set;
    std::vector<double> g_score(graph_.size(), DBL_MAX);
    std::vector<uint> parent(graph_.size(), INT_MAX);

    open_set.push(std::make_pair((end_pos - graph_[start_index]->position_).norm(), start_index));
    g_score[start_index] = 0.0;

    while(!open_set.empty()) {
        uint curr_index = open_set.top().second;
        Eigen::Vector3d curr_pos = graph_[curr_index]->position_;
        open_set.pop();

        if(curr_index == end_index) {
            Path curr_path;
            while(parent[curr_index] != INT_MAX) {
                curr_path.push_back(graph_[curr_index]->position_);
                curr_index = parent[curr_index];
            }
            curr_path.push_back(graph_[start_index]->position_);
            std::reverse(curr_path.begin(), curr_path.end());
            path_ = curr_path;
            return;
        }   

        for(auto& neigh : graph_[curr_index]->neighbours_) {
            uint neigh_index = neigh->id_;
            Eigen::Vector3d neigh_pos = neigh->position_;

            double score = g_score[curr_index] + (neigh_pos - curr_pos).norm();
            if(score < g_score[neigh_index]) {
                g_score[neigh_index] = score;
                parent[neigh_index] = curr_index;
                open_set.push(std::make_pair(score + (end_pos - neigh_pos).norm(), neigh_index));
            }
        }
    }
}

void AStarPlanner::createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {

  pcl::PointXYZI Start;
  pcl::PointXYZI End;

  Start.x = start.x();
  Start.y = start.y();
  Start.z = start.z();

  End.x = end.x();
  End.y = end.y();
  End.z = end.z();
  
  graph_.clear();

  graph_.push_back(Node(new GraphNode(Start, 0)));
  graph_.push_back(Node(new GraphNode(End, 1)));

  uint i=2;

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

void PathVisualizer::visualizePath(const std::string& topic_name, const std::vector<Eigen::Vector3d>& path, 
                       const std::string& frame_id, const ColorType& color, const double& size_factor) {
    if(path.empty()) { return; }

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
    publisher_map_[topic_name].publish(markers);
}

void AStarPlanner::generateSparseGraph() {
  ROS_INFO("About to generate skeleton graph.");

  skeleton_generator_.updateSkeletonFromLayer();
  skeleton_generator_.generateSparseGraph();
  ROS_INFO("Generated skeleton graph.");

  if (visualize_) {
    // Now visualize the graph.
    const voxblox::SparseSkeletonGraph& graph =
        skeleton_generator_.getSparseGraph();
    visualization_msgs::MarkerArray marker_array;
    voxblox::visualizeSkeletonGraph(graph, frame_id_, &marker_array);
    sparse_graph_pub_.publish(marker_array);
  }
  if (verbose_){
    ROS_INFO_STREAM("[GP] Generation timings: " << std::endl
                                                << voxblox::timing::Timing::Print());
  }
}

bool AStarPlanner::plannerServiceCallback(mav_planning_msgs::PlannerServiceRequest& request,
                                          mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  ROS_INFO("Planning path.");
  skeleton_generator_.generateSkeleton();
  generateSparseGraph();
  if (verbose_) {
    ROS_INFO("Finished generating sparse graph.");
    ROS_INFO_STREAM("Total Timings: " << std::endl
                                      << voxblox::timing::Timing::Print());
  }
  findPath(start_pose.position_W, goal_pose.position_W);
  
  voxblox::Point start_point =
      start_pose.position_W.cast<voxblox::FloatingPoint>();
  voxblox::Point goal_point =
      goal_pose.position_W.cast<voxblox::FloatingPoint>();

  visualization_msgs::MarkerArray marker_array;
  bool shorten_graph = true;
  voxblox::AlignedVector<voxblox::Point> diagram_coordinate_path;
  mav_trajectory_generation::timing::Timer astar_diag_timer(
      "plan/astar_diag");

  bool success = skeleton_planner_.getPathUsingEsdfAndDiagram(
      start_point, goal_point, &diagram_coordinate_path);
  mav_msgs::EigenTrajectoryPointVector diagram_path;

  for (const voxblox::Point &voxblox_point : diagram_coordinate_path) {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = voxblox_point.cast<double>();
    diagram_path.push_back(point);
  }

  double path_length = mav_planning::computePathLength(diagram_path);
  int num_vertices = diagram_path.size();
  astar_diag_timer.Stop();
  
  if (visualize_) {
    marker_array.markers.push_back(mav_planning::createMarkerForPath(
        diagram_path, frame_id_, mav_visualization::Color::Purple(),
      "astar_diag", 0.1));
  }

  ROS_INFO("Diag A* Success? %d Path length: %f Vertices: %d", success,
            path_length, num_vertices);
  if (shorten_graph) {
    mav_trajectory_generation::timing::Timer shorten_timer(
        "plan/astar_diag/shorten");
    mav_msgs::EigenTrajectoryPointVector short_path;
    path_shortener_.shortenPath(diagram_path, &short_path);
    path_length = mav_planning::computePathLength(short_path);
    num_vertices = short_path.size();
    ROS_INFO("Diagram Shorten Success? %d Path length: %f Vertices: %d",
              success, path_length, num_vertices);

    if (visualize_)
    {
      marker_array.markers.push_back(mav_planning::createMarkerForPath(
          short_path, frame_id_, mav_visualization::Color::Pink(),
          "short_astar_plan", 0.1));
    }
    shorten_timer.Stop();
    last_waypoints_ = short_path;
  }

  if (visualize_) {
    path_marker_pub_.publish(marker_array);
  }

  if (verbose_) {
    ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
  }

  visualizer_.visualizePath("path", path_, "world", PathVisualizer::ColorType::TEAL, 0.05);
  
}

bool AStarPlanner::publishPathCallback(std_srvs::EmptyRequest& request,
                                        std_srvs::EmptyResponse& response) {
  ROS_INFO("Publishing waypoints.");
  geometry_msgs::PoseArray pose_array;
  pose_array.poses.reserve(last_waypoints_.size());
  for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
    geometry_msgs::PoseStamped pose_stamped;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
    pose_array.poses.push_back(pose_stamped.pose);
  }
  pose_array.header.frame_id = frame_id_;
  waypoint_list_pub_.publish(pose_array);
  return true;
}

void AStarPlanner::skeletonize(voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
  skeleton_generator_.setEsdfLayer(esdf_layer);

  voxblox::FloatingPoint min_separation_angle =
      skeleton_generator_.getMinSeparationAngle();
  nh_private_.param("min_separation_angle", min_separation_angle,
                    min_separation_angle);
  skeleton_generator_.setMinSeparationAngle(min_separation_angle);
  bool generate_by_layer_neighbors =
      skeleton_generator_.getGenerateByLayerNeighbors();
  nh_private_.param("generate_by_layer_neighbors", generate_by_layer_neighbors,
                    generate_by_layer_neighbors);
  skeleton_generator_.setGenerateByLayerNeighbors(generate_by_layer_neighbors);

  int num_neighbors_for_edge = skeleton_generator_.getNumNeighborsForEdge();
  nh_private_.param("num_neighbors_for_edge", num_neighbors_for_edge,
                    num_neighbors_for_edge);
  skeleton_generator_.setNumNeighborsForEdge(num_neighbors_for_edge);

  skeleton_generator_.setMinGvdDistance(constraints_.robot_radius);
}

void AStarPlanner::esdfSliceCallback(pcl::PointCloud<pcl::PointXYZI> pointcloud) {

  pointcloud_ = pointcloud;
  esdf_slice_pub_.publish(pointcloud_);

  Eigen::Vector3d start(1,2,3);
  Eigen::Vector3d end(0,1,2);
  createGraph(start,end);

  if(visualize_) {
      visualizer_.visualizeGraph("graph", graph_);
    }

}

}