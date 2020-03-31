#pragma once

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <mav_msgs/conversions.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>
#include <voxblox_planning_common/path_shortening.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <voxblox_skeleton/skeleton_planner.h>
#include <voxblox_skeleton/sparse_graph_planner.h>

namespace ariitk::global_planner {

class SkeletonGlobalPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SkeletonGlobalPlanner(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
  virtual ~SkeletonGlobalPlanner() {}

  void generateSparseGraph();

  bool plannerServiceCallback(
      mav_planning_msgs::PlannerServiceRequest& request,
      mav_planning_msgs::PlannerServiceResponse& response);

  bool publishPathCallback(std_srvs::EmptyRequest& request,
                           std_srvs::EmptyResponse& response);

  void convertCoordinatePathToPath(
      const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector& path) ;

  double getMapDistance(const Eigen::Vector3d& position) const;

  void skeletonize(voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher path_marker_pub_;
  ros::Publisher sparse_graph_pub_;
  ros::Publisher path_pub_;
  ros::Publisher waypoint_list_pub_;

  ros::ServiceServer planner_srv_;
  ros::ServiceServer path_pub_srv_;

  // Settings for physical constriants.
  mav_planning::PhysicalConstraints constraints_;

  std::string frame_id_;
  bool visualize_;
  bool verbose_;
  double voxel_size_;  // Cache the size of the voxels used by the map.

  voxblox::EsdfServer voxblox_server_;
  voxblox::SkeletonGenerator skeleton_generator_;

  // Planners of all sorts.
  voxblox::SkeletonAStar skeleton_planner_;
  mav_planning::EsdfPathShortener path_shortener_;

  // Waypoints
  mav_msgs::EigenTrajectoryPointVector last_waypoints_;
};

}  // namespace ariitk::global_planner
