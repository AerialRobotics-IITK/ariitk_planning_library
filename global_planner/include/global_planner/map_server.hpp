#pragma once

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>

namespace ariitk::global_planner {

class MapServer {
    public:
        MapServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        voxblox::EsdfMap::Ptr getESDFMapPtr();

    private:
        voxblox::EsdfServer voxblox_server_;
        voxblox::TsdfMap::Ptr tsdf_map_ptr_;
        voxblox::EsdfMap::Ptr esdf_map_ptr_;

        bool visualize_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        double robot_radius_;
};

} // namespace ariitk::global_planner