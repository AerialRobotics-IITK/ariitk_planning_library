#include <voxblox_local_planner/local_planner.hpp>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : pathfinder_(nh, nh_private) 
    , last_yaw_(0) {
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("voxel_size", voxel_size_);
    nh_private.getParam("sampling_dt", sampling_dt_);

    smoother_.setParametersFromRos(nh_private);
    smoother_.setMinCollisionCheckResolution(voxel_size_);
    smoother_.setDistanceAndGradientFunction(
        std::bind(&LocalPlanner::getMapDistanceAndGradient, this,
                    std::placeholders::_1, std::placeholders::_2));
    smoother_.setOptimizeTime(true);
    smoother_.setResampleTrajectory(true);
    smoother_.setResampleVisibility(true);
    smoother_.setNumSegments(5);
    smoother_.setVerbose(false);

    visualizer_.init(nh, nh_private);
    visualizer_.createPublisher("occ_shield");
    visualizer_.createPublisher("free_shield");
    visualizer_.createPublisher("trajectory");
    visualizer_.createPublisher("segment");

    odometry_sub_ = nh.subscribe("odometry", 1, &LocalPlanner::odometryCallback, this);
    waypoint_sub_ = nh.subscribe("waypoint", 1, &LocalPlanner::waypointCallback, this);
    waypoint_list_sub_ = nh.subscribe("waypoint_list", 1, &LocalPlanner::waypointListCallback, this);
    command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
}

void LocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
    ROS_INFO("Recieved waypoint!");
    mav_msgs::EigenOdometry start_odom;
    mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

    mav_msgs::EigenTrajectoryPoint waypoint;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

    plan(start_odom.position_W, waypoint.position_W);
    executePlan();

    ros::Rate loop_rate(10);
    
    while(ros::ok() && norm(odometry_.pose.pose.position, trajectory_.back().position_W) > 0.2) {
        ros::spinOnce();
        if(checkForReplan()) { 
            ROS_WARN("Replanning!");
            geometry_msgs::PoseStamped stop_msg;
            stop_msg.header.stamp = ros::Time::now();
            stop_msg.pose = odometry_.pose.pose;
            command_pub_.publish(stop_msg);

            Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
            pathfinder_.inflateRadius(2.0);
            plan(curr_pos, trajectory_.back().position_W);
            ROS_INFO("Replanned!");
            executePlan();
        }
        loop_rate.sleep();
    }
}

bool LocalPlanner::checkForReplan() {
    bool need_replan = false;
    std::vector<Eigen::Vector3d> free_points;
    std::vector<Eigen::Vector3d> occ_points;

    double distance = 0.0;
    for(auto& point : trajectory_) {    // optimize
        if(pathfinder_.getMapDistance(point.position_W, distance) && distance < voxel_size_) {
            occ_points.push_back(point.position_W);
        } else {
            free_points.push_back(point.position_W);
        }
    }

    need_replan = (occ_points.size() > 0);
    visualizer_.visualizePoints("occ_shield", occ_points, "world", PathVisualizer::ColorType::RED, 1);
    visualizer_.visualizePoints("free_shield", free_points, "world", PathVisualizer::ColorType::GREEN, 0.5);
    return need_replan;
}

void LocalPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    pathfinder_.findPath(start, end);
    waypoints_ = pathfinder_.getPath();
    generateTrajectoryThroughWaypoints(waypoints_);
}

void LocalPlanner::executePlan() {
    trajectory_msgs::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_, &traj_msg);
    traj_pub_.publish(traj_msg);
}

void LocalPlanner::waypointListCallback(const geometry_msgs::PoseArray& msg) {
    waypoints_.clear();
    trajectory_.clear();
    for(auto& pose : msg.poses) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose = pose;
        waypointCallback(msg);
    }
}

void LocalPlanner::generateTrajectoryBetweenTwoPoints(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    trajectory_.clear();
    mav_msgs::EigenTrajectoryPoint start_pt, end_pt;
    start_pt.position_W = start;
    end_pt.position_W = end;
    if(norm(start_pt.position_W, end_pt.position_W) < voxel_size_ || !smoother_.getPathBetweenTwoPoints(start_pt, end_pt, &trajectory_)){
        Path dummy_path;
        dummy_path.push_back(start);
        dummy_path.push_back(end);
        convertPathToTrajectory(dummy_path, trajectory_);
    }
    applyYawToTrajectory();
}

void LocalPlanner::generateTrajectoryThroughWaypoints(const Path& waypoints) {
    trajectory_.clear();
    if(waypoints.empty()) { return; }

    Trajectory eigen_waypts;
    convertPathToTrajectory(waypoints, eigen_waypts);

    if(norm(waypoints.front(), waypoints.back()) < voxel_size_) { 
        trajectory_ = eigen_waypts; 
        applyYawToTrajectory();
        return;
    }

    mav_trajectory_generation::Trajectory gen_traj;
    
    ros::spinOnce();
    eigen_waypts[0].orientation_W_B = mav_msgs::quaternionFromMsg(odometry_.pose.pose.orientation);
    eigen_waypts[0].velocity_W = mav_msgs::vector3FromMsg(odometry_.twist.twist.linear);
    eigen_waypts[0].angular_velocity_W = mav_msgs::vector3FromMsg(odometry_.twist.twist.angular);

    bool success = smoother_.getTrajectoryBetweenWaypoints(eigen_waypts, &gen_traj);
    if(success) { 
        mav_trajectory_generation::sampleWholeTrajectory(gen_traj, sampling_dt_, &trajectory_); 
    } 
    else { trajectory_ = eigen_waypts; }

    applyYawToTrajectory();
}

void LocalPlanner::applyYawToTrajectory() {
    if(trajectory_.size() < 2) { return; }
    for(auto i = 0; i < trajectory_.size() - 1; i++) {
        Eigen::Vector3d heading = trajectory_[i+1].position_W - trajectory_[i].position_W;
        double desired_yaw = 0.0;
        if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
           desired_yaw = std::atan2(heading.y(), heading.x());
        }
        trajectory_[i].setFromYaw(desired_yaw);
    }
    visualizer_.visualizeTrajectory("trajectory", trajectory_, "world", PathVisualizer::ColorType::BLACK, 0.2);
}

void LocalPlanner::convertPathToTrajectory(const Path& path, Trajectory& trajectory) {
    for(auto& point : path) {
        mav_msgs::EigenTrajectoryPoint traj_pt;
        traj_pt.position_W = point;
        trajectory.push_back(traj_pt);
    }
}

} // namespace ariitk::local_planner
