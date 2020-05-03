#include<voxblox_global_planner/a_star_planner.hpp>

namespace ariitk::global_planner {

AStarPlanner::AStarPlanner(const ros::NodeHandle& nh,
            const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_,nh_private_) {

    nh_private_.param("robot_radius",robot_radius_,0.45);

    esdf_slice_sub_ = nh_private_.subscribe("esdf_slice",1,&AStarPlanner::esdfSliceCallback,this);
    esdf_slice_pub_ = nh_private_.advertise<sensor_msgs::PointCloud>("esdf_slice_out",1,true);

    // generateGraph();

    }

// void AStarPlanner::generateGraph() {


// }

void AStarPlanner::esdfSliceCallback(sensor_msgs::PointCloud2 pointcloud) {
  pointcloud_ = pointcloud;
}

}