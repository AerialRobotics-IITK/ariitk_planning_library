#include <global_planner/map_server.hpp>

namespace ariitk::global_planner {

MapServer::MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
    :   voxblox_server_(nh, nh_private)
    ,   nh_(nh)
    ,   nh_private_(nh_private)
    ,   visualize_(false) {
    tsdf_map_ptr_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ptr_ = voxblox_server_.getEsdfMapPtr();
    nh_private_.getParam("visualize", visualize_);
    nh_private_.getParam("robot_radius", robot_radius_);

    if(visualize_) {
        voxblox_server_.generateMesh();
        voxblox_server_.publishSlices();
        voxblox_server_.publishPointclouds();
        voxblox_server_.publishTraversable();
    }
}

voxblox::EsdfMap::Ptr MapServer::getESDFMapPtr() { return esdf_map_ptr_; }

} // namespace ariitk::global_planner