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
  voxblox_server_(nh_,nh_private_) {

    nh_private_.param("robot_radius",robot_radius_,0.45);

    esdf_slice_sub_ = nh_private_.subscribe("esdf_slice",1,&AStarPlanner::esdfSliceCallback,this);
    esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_slice_out",1,true);

    }

void AStarPlanner::generateGraph() {
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
}

void AStarPlanner::esdfSliceCallback(pcl::PointCloud<pcl::PointXYZI> pointcloud) {

  pointcloud_ = pointcloud;

   generateGraph();              //generating graph from Slices

}

}