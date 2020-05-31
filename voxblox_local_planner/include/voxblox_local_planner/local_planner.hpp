#pragma once

#include <future>
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
        enum class PlanStatus{FAILURE, IN_PROGRESS, SUCCESS, IDLE, UNKNOWN};

        LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void setConstantYaw(const double& yaw) { const_yaw_ = yaw; }

    private:
        enum class YawPolicy{ POINT_FACING, ANTICIPATE_VELOCITY, FOLLOW_VELOCITY, CONSTANT };

        static inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return std::sqrt(std::pow(p1.x-p2.x,2) + std::pow(p1.y-p2.y, 2) + std::pow(p1.z-p2.z, 2));
        }
        geometry_msgs::Point convertEigenToGeometryMsg(const Eigen::Vector3d& point);

        void odometryCallback(const nav_msgs::Odometry& msg){ odometry_ = msg; }
        void waypointCallback(const geometry_msgs::PoseStamped& msg);
        void waypointListCallback(const geometry_msgs::PoseArray& msg);

        Trajectory plan(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void executePlan(const Trajectory& trajectory);
        bool checkForReplan(const Trajectory& trajectory);

        Trajectory generateTrajectoryThroughWaypoints(const Path& waypoints);
        void applyYawToTrajectory(Trajectory& trajectory, const YawPolicy& policy = YawPolicy::ANTICIPATE_VELOCITY);
        void generateTrajectoryBetweenTwoPoints(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        void convertPathToTrajectory(const Path& path, Trajectory& trajectory);

        inline double getMapDistanceAndGradient(const Eigen::Vector3d& point, Eigen::Vector3d* gradient) {
            return pathfinder_.getMapDistanceAndGradient(point, gradient);
        }

        void clear();
        void setStatus(const PlanStatus& status);

        Path waypoints_;
        Trajectory trajectory_;

        PathFinder pathfinder_;
        Visualizer visualizer_;
        mav_planning::LocoSmoother smoother_;

        bool visualize_;
        double robot_radius_;
        double voxel_size_;
        double sampling_dt_;
        double const_yaw_;

        uint curr_waypt_;
        size_t path_index_;
        size_t pub_index_;

        ros::Publisher command_pub_;
        ros::Publisher traj_pub_;
        ros::Publisher plan_status_pub_;

        ros::Subscriber odometry_sub_;
        ros::Subscriber waypoint_sub_;
        ros::Subscriber waypoint_list_sub_;

        nav_msgs::Odometry odometry_;

        PlanStatus status_;
        std::future<void> status_thread_;
};

} // namespace ariitk::local_planner
