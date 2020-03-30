#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>

#include "voxblox_skeleton_planner/skeleton_global_planner.h"

namespace mav_planning {

SkeletonGlobalPlanner::SkeletonGlobalPlanner(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("map"),
      visualize_(true),
      update_esdf_(true),
      voxblox_server_(nh_, nh_private_),
      skeleton_generator_() {
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("update_esdf", update_esdf_, update_esdf_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  skeleton_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >(
      "skeleton", 1, true);
  sparse_graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "sparse_graph", 1, true);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  planner_srv_ = nh_private_.advertiseService(
      "plan", &SkeletonGlobalPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &SkeletonGlobalPlanner::publishPathCallback, this);

  std::shared_ptr<voxblox::EsdfMap> esdf_map = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map);

  ROS_INFO(
      "Size: %f VPS: %zu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

  // Now set up the skeleton generator.
  skeletonize(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  // Set up the A* planners.
  skeleton_planner_.setSkeletonLayer(skeleton_generator_.getSkeletonLayer());
  skeleton_planner_.setEsdfLayer(
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  skeleton_planner_.setMinEsdfDistance(constraints_.robot_radius);

  // Set up shortener.
  path_shortener_.setEsdfLayer(
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());
  path_shortener_.setConstraints(constraints_);

  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
  }
}

void SkeletonGlobalPlanner::generateSparseGraph() {
  ROS_INFO("About to generate skeleton graph.");

  skeleton_generator_.updateSkeletonFromLayer();
  ROS_INFO("Re-populated from layer.");

  skeleton_generator_.generateSparseGraph();
  ROS_INFO("Generated skeleton graph.");

  if (visualize_) {
    voxblox::Pointcloud pointcloud;
    std::vector<float> distances;
    skeleton_generator_.getSkeleton().getEdgePointcloudWithDistances(
        &pointcloud, &distances);

    // Publish the skeleton.
    pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
    voxblox::pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
    ptcloud_pcl.header.frame_id = frame_id_;
    skeleton_pub_.publish(ptcloud_pcl);

    // Now visualize the graph.
    const voxblox::SparseSkeletonGraph& graph =
        skeleton_generator_.getSparseGraph();
    visualization_msgs::MarkerArray marker_array;
    voxblox::visualizeSkeletonGraph(graph, frame_id_, &marker_array);
    sparse_graph_pub_.publish(marker_array);
  }

  ROS_INFO_STREAM("Generation timings: " << std::endl
                                         << voxblox::timing::Timing::Print());
}

bool SkeletonGlobalPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  ROS_INFO("Planning path.");

  if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Start pose occupied!");
    return false;
  }
  if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
    ROS_ERROR("Goal pose occupied!");
    return false;
  }

  voxblox::Point start_point =
      start_pose.position_W.cast<voxblox::FloatingPoint>();
  voxblox::Point goal_point =
      goal_pose.position_W.cast<voxblox::FloatingPoint>();
  
  skeleton_generator_.generateSkeleton();
  skeleton_generator_.generateSparseGraph();
  ROS_INFO("Finished generating sparse graph.");

  ROS_INFO_STREAM("Total Timings: " << std::endl
                                  << voxblox::timing::Timing::Print());

  // Now visualize the graph.
  const auto &graph = skeleton_generator_.getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, frame_id_, &marker_array);
  sparse_graph_pub_.publish(marker_array);
  marker_array.markers.clear();

  bool shorten_graph = true;
  bool smooth_path = true;

  voxblox::AlignedVector<voxblox::Point> diagram_coordinate_path;
  mav_trajectory_generation::timing::Timer astar_diag_timer(
      "plan/astar_diag");
  bool success = skeleton_planner_.getPathUsingEsdfAndDiagram(
      start_point, goal_point, &diagram_coordinate_path);
  mav_msgs::EigenTrajectoryPointVector diagram_path;
  for (const voxblox::Point &voxblox_point : diagram_coordinate_path)
  {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = voxblox_point.cast<double>();
    diagram_path.push_back(point);
  }
  double path_length = computePathLength(diagram_path);
  int num_vertices = diagram_path.size();
  astar_diag_timer.Stop();

  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
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
    path_length = computePathLength(short_path);
    num_vertices = short_path.size();
    ROS_INFO("Diagram Shorten Success? %d Path length: %f Vertices: %d",
              success, path_length, num_vertices);
    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          short_path, frame_id_, mav_visualization::Color::Pink(),
          "short_astar_plan", 0.1));
    }
    shorten_timer.Stop();

    last_waypoints_ = short_path;
  }

  if (visualize_) {
    path_marker_pub_.publish(marker_array);
  }

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
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
  if (update_esdf_ ||
      voxblox_server_.getEsdfMapPtr()
              ->getEsdfLayerPtr()
              ->getNumberOfAllocatedBlocks() == 0) {
    const bool full_euclidean_distance = true;
    voxblox_server_.updateEsdfBatch(full_euclidean_distance);
  }
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

}  // namespace mav_planning
