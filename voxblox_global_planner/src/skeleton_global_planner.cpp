#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>

#include <voxblox_global_planner/skeleton_global_planner.hpp>

namespace ariitk::global_planner {

SkeletonGlobalPlanner::SkeletonGlobalPlanner(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    frame_id_("map"),
    visualize_(true),
    verbose_(false),
    voxblox_server_(nh_, nh_private_),
    skeleton_generator_() {
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &SkeletonGlobalPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &SkeletonGlobalPlanner::publishPathCallback, this);

  if (verbose_) {
    ROS_INFO(
        "Size: %f VPS: %zu",
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
        voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());
  }
  
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

  // Set up shortener.
  ROS_INFO("Initializing path shortener.");
  path_shortener_.setEsdfLayer(
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  path_shortener_.setConstraints(constraints_);
}

void SkeletonGlobalPlanner::generateSparseGraph() {
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

bool SkeletonGlobalPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
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

  if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_WARN("Goal pose occupied! Planning to Nearest Free Point");
    Eigen::Vector3d new_goal_pos;
    if(!getNearestFreeSpaceToPoint(goal_pose.position_W, new_goal_pos)) {
      ROS_ERROR("No free points near goal pose, and goal pose is occupied!");
      return false;
    }
    goal_pose.position_W = new_goal_pos;
  }

  if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
  ROS_WARN("Start pose occupied! Planning to Nearest Free Point");
  Eigen::Vector3d new_start_pos;
  if(!getNearestFreeSpaceToPoint(start_pose.position_W, new_start_pos)) {
    ROS_ERROR("No free points near start pose, and start pose is occupied!");
    return false;
  }
  start_pose.position_W = new_start_pos;
  return false;
  }

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
}

double SkeletonGlobalPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    return 0.0;
  }
  return distance;
}

bool SkeletonGlobalPlanner::getMapDistanceAndGradient(const Eigen::Vector3d& position,
    double& distance, Eigen::Vector3d& gradient) const {
  if(!voxblox_server_.getEsdfMapPtr()->
    getDistanceAndGradientAtPosition(position, false, &distance, &gradient)) {
    return false;
  }
  return true;
}


bool SkeletonGlobalPlanner::publishPathCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
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

void SkeletonGlobalPlanner::skeletonize(voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
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

bool SkeletonGlobalPlanner::getNearestFreeSpaceToPoint(const Eigen::Vector3d& pos, 
    Eigen::Vector3d& new_pos) const {
  Eigen::Vector3d final_pos = pos;
  double distance = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  
  const double angle_step = 0.1;
  const size_t max_iterations = 10;
  for(size_t step = 1; step <= max_iterations; step++) {
    for(double angle = -M_PI; angle < M_PI; angle += angle_step) {
      direction = Eigen::Vector3d(cos(angle), sin(angle), 0);
      final_pos = pos + direction * step * constraints_.robot_radius;
      if(getMapDistanceAndGradient(final_pos, distance, gradient)
          && distance >= constraints_.robot_radius) {
        new_pos = final_pos;
        ROS_INFO("Point shifted from: (%lf %lf %lf) to (%lf %lf %lf)", 
              pos.x(), pos.y(), pos.z(), new_pos.x(), new_pos.y(), new_pos.z());
        return true;
      }
    }
  }

  // const size_t max_iterations = 1000;
  // for(size_t iter = 0; iter < max_iterations; iter++) {
  //   bool result = getMapDistanceAndGradient(final_pos, distance, gradient);
  //   if(result && distance >= constraints_.robot_radius) {
  //     new_pos = final_pos;
  //     ROS_INFO("Point shifted from: (%lf %lf %lf) to (%lf %lf %lf)", 
  //             pos.x(), pos.y(), pos.z(), new_pos.x(), new_pos.y(), new_pos.z());
  //     return true;
  //   }
  //   if(gradient.norm() > 1e-6) {
  //     final_pos += gradient.normalized() * (iter/100 + 1) * constraints_.robot_radius;
  //   }
  // }
  return false;
}

}  // namespace mav_planning
