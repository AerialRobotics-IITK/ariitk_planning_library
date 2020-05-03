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
  // pointcloud_.header.seq = pointcloud.header.seq;
  // pointcloud_.header.stamp = pointcloud.header.stamp;
  // pointcloud_.header.frame_id = pointcloud.header.frame_id;
  // pointcloud_.height = pointcloud.height;
  // pointcloud_.width = pointcloud.width;
  // pointcloud_.fields.name = pointcloud.fields.name;
  // pointcloud_.fields.offset = pointcloud.fields.offset;
  // pointcloud_.fields.datatype = pointcloud.fields.datatype;
  // pointcloud_.fields.count = pointcloud.fields.count;
  // pointcloud_.is_bigendian = pointcloud.is_bigendian;
  // pointcloud_.point_step = pointcloud.point_step;
  // pointcloud_.row_step = pointcloud.row_step;
  // pointcloud_.data = pointcloud.data;
  // pointcloud_.is_dense = pointcloud.is_dense;
  pointcloud_ = pointcloud;
}

}