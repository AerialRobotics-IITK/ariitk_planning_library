#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <voxblox_ros/esdf_server.h> 
#include <Eigen/Eigen>

#include <voxblox_local_planner/path_finder.hpp>

namespace ariitk::local_planner {

class LocalPlanner {
    public:
        LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    
    private:
        static inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return std::sqrt(std::pow(p1.x-p2.x,2) + std::pow(p1.y-p2.y, 2) + std::pow(p1.z-p2.z, 2));
        }

        static inline double norm(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
            return std::sqrt(std::pow(p1.x()-p2.x(),2) + std::pow(p1.y()-p2.y(), 2) + std::pow(p1.z()-p2.z(), 2));
        }
        
        void odometryCallback(const nav_msgs::Odometry& msg){ odometry_ = msg; }
        void waypointCallback(const geometry_msgs::PoseStamped& msg);
        void waypointListCallback(const geometry_msgs::PoseArray& msg);
        
        void setYawFacing(geometry_msgs::PoseStamped& msg);
        bool checkReplan();
        void replan(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

        uint curr_index_;
        Path waypoints_;

        PathFinder pathfinder_;
        PathVisualizer visualizer_;

        bool visualize_;
        double last_yaw_;
        double robot_radius_;
        double voxel_size_;

        ros::Publisher command_pub_;

        ros::Subscriber odometry_sub_;
        ros::Subscriber waypoint_sub_;
        ros::Subscriber waypoint_list_sub_;

        nav_msgs::Odometry odometry_;
};

} // namespace ariitk::local_planner
