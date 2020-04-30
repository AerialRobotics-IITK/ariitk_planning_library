#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <voxblox_ros/esdf_server.h>
#include <tf/tf.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <Eigen/Eigen>

#include <voxblox_local_planner/path_finder.hpp>
#include <mav_trajectory_generation/trajectory_sampling.h>

namespace ariitk::local_planner {

typedef mav_msgs::EigenTrajectoryPointVector Trajectory;

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

        static inline double norm(const geometry_msgs::Point& p1, const Eigen::Vector3d& p2) {
            return std::sqrt(std::pow(p1.x-p2.x(),2) + std::pow(p1.y-p2.y(), 2) + std::pow(p1.z-p2.z(), 2));
        }
        
        void odometryCallback(const nav_msgs::Odometry& msg){ odometry_ = msg; }
        void waypointCallback(const geometry_msgs::PoseStamped& msg);
        void waypointListCallback(const geometry_msgs::PoseArray& msg);
        
        void applyYawToTrajectory();
        bool checkForReplan();
        void replan(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void generateTrajectoryBetweenTwoPoints(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void generateTrajectoryThroughWaypoints(const Path& waypoints);
        uint getTrajectorySegment(const Eigen::Vector3d& end_pt);
        void convertPathToTrajectory(const Path& path, Trajectory& trajectory);
        inline double getMapDistanceAndGradient(const Eigen::Vector3d& point, Eigen::Vector3d* gradient) const {
            return pathfinder_.getMapDistanceAndGradient(point, gradient);
        }

        uint curr_index_;
        Path waypoints_;
        Trajectory trajectory_;
        Trajectory segment_;
        uint path_index_;

        PathFinder pathfinder_;
        PathVisualizer visualizer_;
        mav_planning::LocoSmoother smoother_;

        bool visualize_;
        double last_yaw_;
        double robot_radius_;
        double voxel_size_;
        double sampling_dt_;
        double command_publishing_dt_;

        ros::Publisher command_pub_;
        ros::Publisher traj_pub_;

        ros::Subscriber odometry_sub_;
        ros::Subscriber waypoint_sub_;
        ros::Subscriber waypoint_list_sub_;

        nav_msgs::Odometry odometry_;
};

} // namespace ariitk::local_planner
