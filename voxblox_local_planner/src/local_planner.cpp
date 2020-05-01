#include <voxblox_local_planner/local_planner.hpp>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : pathfinder_(nh, nh_private) 
    , const_yaw_(0) {
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

    if(visualize_) {
        visualizer_.init(nh, nh_private);
        visualizer_.createPublisher("occupied_path");
        visualizer_.createPublisher("free_path");
        visualizer_.createPublisher("trajectory");
    }

    odometry_sub_ = nh.subscribe("odometry", 1, &LocalPlanner::odometryCallback, this);
    waypoint_sub_ = nh.subscribe("waypoint", 1, &LocalPlanner::waypointCallback, this);
    waypoint_list_sub_ = nh.subscribe("waypoint_list", 1, &LocalPlanner::waypointListCallback, this);
    command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
}

void LocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
    ROS_INFO("Recieved waypoint!");
    clear();
    mav_msgs::EigenOdometry start_odom;
    mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

    mav_msgs::EigenTrajectoryPoint waypoint;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

    trajectory_ = plan(start_odom.position_W, waypoint.position_W);
    executePlan(trajectory_);

    ros::Rate loop_rate(10);
    
    while(ros::ok() && norm(odometry_.pose.pose.position, trajectory_.back().position_W) > voxel_size_) {
        ros::spinOnce();
        if(checkForReplan(trajectory_)) { 
            ROS_WARN("Replanning!");
            geometry_msgs::PoseStamped stop_msg;
            stop_msg.header.stamp = ros::Time::now();
            stop_msg.pose = odometry_.pose.pose;
            command_pub_.publish(stop_msg);

            Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
            pathfinder_.inflateRadius(2.0);
            trajectory_ = plan(curr_pos, trajectory_.back().position_W);
            path_index_ = pub_index_ = 0;
            ROS_INFO("Replanned!");
            executePlan(trajectory_);
        }
        loop_rate.sleep();
    }
}

bool LocalPlanner::checkForReplan(const Trajectory& segment) {
    bool need_replan = false;
    uint occupied = 0;
    std::vector<Eigen::Vector3d> free_points, occ_points;

    double distance = 0.0;
    for(auto& point : segment) {
        if(pathfinder_.getMapDistance(point.position_W, distance) && distance < voxel_size_) {
            occupied++;
            if(visualize_) { occ_points.push_back(point.position_W); }
        } else if(visualize_) {
            free_points.push_back(point.position_W);
        }
    }

    need_replan = (occupied > 0);

    if(visualize_) {
        visualizer_.visualizePoints("occupied_path", occ_points, "world", PathVisualizer::ColorType::RED, 1);
        visualizer_.visualizePoints("free_path", free_points, "world", PathVisualizer::ColorType::GREEN, 0.5);
    }

    return need_replan;
}

Trajectory LocalPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    pathfinder_.findPath(start, end);
    waypoints_ = pathfinder_.getPath();
    return generateTrajectoryThroughWaypoints(waypoints_);
}

void LocalPlanner::executePlan(const Trajectory& trajectory) {
    if(trajectory.empty()) { 
        ROS_WARN("Nothing to publish!"); 
        return;
    }

    path_index_ = pub_index_;
    trajectory_msgs::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &traj_msg);
    pub_index_ += trajectory.size();
    
    traj_pub_.publish(traj_msg);
}

void LocalPlanner::waypointListCallback(const geometry_msgs::PoseArray& msg) {
    clear();
    
    uint num_waypts = msg.poses.size();
    if(num_waypts == 0) { 
        ROS_INFO("No waypoints!"); 
        return; 
    }

    mav_msgs::EigenOdometry start_odom;
    mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

    mav_msgs::EigenTrajectoryPointVector waypoints;
    for(auto& pose : msg.poses) {
        geometry_msgs::PoseStamped conv_msg;
        mav_msgs::EigenTrajectoryPoint waypoint;
        conv_msg.pose = pose;
        mav_msgs::eigenTrajectoryPointFromPoseMsg(conv_msg, &waypoint);
        waypoints.push_back(waypoint);
    }

    Trajectory path = plan(start_odom.position_W, waypoints[curr_waypt_].position_W);
    trajectory_.insert(trajectory_.end(), path.begin(), path.end());
    executePlan(trajectory_);

    ros::Rate loop_rate(10);
    while(ros::ok() && (curr_waypt_ < num_waypts || norm(odometry_.pose.pose.position, trajectory_.back().position_W) > voxel_size_)) {
        ros::spinOnce();
        if(norm(odometry_.pose.pose.position, trajectory_.back().position_W) < robot_radius_ && curr_waypt_ < num_waypts - 1) {
            Trajectory next_path = plan(waypoints[curr_waypt_].position_W, waypoints[curr_waypt_ + 1].position_W);
            curr_waypt_++;
            trajectory_.insert(trajectory_.end(), next_path.begin(), next_path.end());
            executePlan(next_path); // should we execute immediately? TODO: separate preplan and execution
        }
        Trajectory segment(trajectory_.begin() + path_index_, trajectory_.begin() + pub_index_);
        if(checkForReplan(segment)) {
            ROS_WARN("Replanning!");
            geometry_msgs::PoseStamped stop_msg;
            stop_msg.header.stamp = ros::Time::now();
            stop_msg.pose = odometry_.pose.pose;
            command_pub_.publish(stop_msg);

            Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
            pathfinder_.inflateRadius(2.0); // TODO: Blacklist occupied region from sampling
            trajectory_ = plan(curr_pos, trajectory_.back().position_W); // what if trajectory_.back() is also occupied? TODO: Nearest Free Goal
            path_index_ = pub_index_ = 0;
            ROS_INFO("Replanned!");
            executePlan(trajectory_);
        }
        loop_rate.sleep();
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
    applyYawToTrajectory(trajectory_);
    if(visualize_) { visualizer_.visualizeTrajectory("trajectory", trajectory_, "world", PathVisualizer::ColorType::BLACK, 0.2); }
}

Trajectory LocalPlanner::generateTrajectoryThroughWaypoints(const Path& waypoints) {
    Trajectory traj;
    if(waypoints.empty()) { return traj; }

    Trajectory eigen_waypts;
    convertPathToTrajectory(waypoints, eigen_waypts);

    if(pathfinder_.getPathLength(waypoints) < 0.05) { 
        traj = eigen_waypts; 
        applyYawToTrajectory(traj);
        if(visualize_) {
            visualizer_.visualizeTrajectory("trajectory", traj, "world", PathVisualizer::ColorType::BLACK, 0.2);
        }
        return traj;
    }

    mav_trajectory_generation::Trajectory gen_traj;
    ros::spinOnce();

    // TODO: Preserve velocities when switching waypoints
    bool success = smoother_.getTrajectoryBetweenWaypoints(eigen_waypts, &gen_traj);
    if(success) { 
        mav_trajectory_generation::sampleWholeTrajectory(gen_traj, sampling_dt_, &traj); 
    } 
    else { traj = eigen_waypts; }

    applyYawToTrajectory(traj);
    if(visualize_) {
        visualizer_.visualizeTrajectory("trajectory", traj, "world", PathVisualizer::ColorType::BLACK, 0.2);
    }
    return traj;
}

void LocalPlanner::applyYawToTrajectory(Trajectory& trajectory, const YawPolicy& policy) {
    if(trajectory.size() < 2) { return; }
    double last_yaw = trajectory.front().getYaw();

    if(policy == YawPolicy::POINT_FACING) {
        for(auto i = 0; i < trajectory.size() - 1; i++) {
            Eigen::Vector3d heading = trajectory[i+1].position_W - trajectory[i].position_W;
            double desired_yaw = 0.0;
            if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
                desired_yaw = std::atan2(heading.y(), heading.x());
            } else {
                desired_yaw = last_yaw;
            }
            trajectory[i].setFromYaw(desired_yaw);
            last_yaw = desired_yaw;
        }
    } else if(policy == YawPolicy::FOLLOW_VELOCITY) {
        const double minVelocityNorm = 0.1;
        for(auto i = 0; i < trajectory.size(); i++) {
            Eigen::Vector3d velocity_xy = trajectory[i].velocity_W;
            velocity_xy.z() = 0;
            if(velocity_xy.norm() > minVelocityNorm) {
                double desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            } else {
                double desired_yaw = last_yaw;
                auto j = i + 1;
                while(j < trajectory.size() && velocity_xy.norm() < minVelocityNorm) {
                    velocity_xy = trajectory[j].velocity_W;
                    velocity_xy.z() = 0; j++;
                }
                if(velocity_xy.norm() > minVelocityNorm) {
                    desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                } 
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            }
        }
    } else if(policy == YawPolicy::ANTICIPATE_VELOCITY) {
        const double minVelocityNorm = 0.1;
        double initial_yaw = last_yaw;
        for(auto i = trajectory.size(); i > 0; i--) {
            Eigen::Vector3d velocity_xy = trajectory[i].velocity_W;
            velocity_xy.z() = 0;
            if(velocity_xy.norm() > minVelocityNorm) {
                double desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            } else {
                double desired_yaw = last_yaw;
                for(auto j = i; j > 0 && velocity_xy.norm() < minVelocityNorm; j--) {
                    velocity_xy = trajectory[j-1].velocity_W;
                    velocity_xy.z() = 0;
                }
                if(velocity_xy.norm() > minVelocityNorm) {
                    desired_yaw = atan2(velocity_xy.y(), velocity_xy.x());
                } 
                trajectory[i].setFromYaw(desired_yaw);
                last_yaw = desired_yaw;
            }
        }
        trajectory[0].setFromYaw(initial_yaw);
    } else if(policy == YawPolicy::CONSTANT) {
        for(auto i = 0; i < trajectory.size(); i++) {
            trajectory[i].setFromYaw(const_yaw_);
        }
    }
}

void LocalPlanner::convertPathToTrajectory(const Path& path, Trajectory& trajectory) {
    for(auto& point : path) {
        mav_msgs::EigenTrajectoryPoint traj_pt;
        traj_pt.position_W = point;
        trajectory.push_back(traj_pt);
    }
}

void LocalPlanner::clear() {
    waypoints_.clear();
    trajectory_.clear();
    curr_waypt_ = path_index_ = pub_index_ = 0;
}

} // namespace ariitk::local_planner
