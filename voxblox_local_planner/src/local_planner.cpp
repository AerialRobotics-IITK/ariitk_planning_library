#include <voxblox_local_planner/local_planner.hpp>
#include <mav_msgs/conversions.h>

namespace ariitk::local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : pathfinder_(nh, nh_private) 
    , last_yaw_(0) {
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("voxel_size", voxel_size_);

    visualizer_.init(nh, nh_private);
    visualizer_.createPublisher("occ_shield");
    visualizer_.createPublisher("free_shield");

    odometry_sub_ = nh.subscribe("odometry", 1, &LocalPlanner::odometryCallback, this);
    waypoint_sub_ = nh.subscribe("waypoint", 1, &LocalPlanner::waypointCallback, this);
    waypoint_list_sub_ = nh.subscribe("waypoint_list", 1, &LocalPlanner::waypointListCallback, this);
    command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
}

void LocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
    ROS_INFO("Recieved waypoint!");
    mav_msgs::EigenOdometry start_odom;
    mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

    mav_msgs::EigenTrajectoryPoint waypoint;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

    pathfinder_.findPath(start_odom.position_W, waypoint.position_W);
    waypoints_ = pathfinder_.getPath();
    ros::Rate loop_rate(10);
    
    for(curr_index_ = 0; curr_index_ < waypoints_.size(); curr_index_++) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();

        msg.pose.position.x = waypoints_[curr_index_].x();
        msg.pose.position.y = waypoints_[curr_index_].y();
        msg.pose.position.z = waypoints_[curr_index_].z();

        setYawFacing(msg);
        command_pub_.publish(msg);

        while(ros::ok() && norm(odometry_.pose.pose.position, msg.pose.position) > 0.2) {
            ros::spinOnce();
            if(checkReplan()) { 
                ROS_WARN("Replanning!");
                
                geometry_msgs::PoseStamped stop_msg;
                stop_msg.header.stamp = ros::Time::now();
                stop_msg.pose = odometry_.pose.pose;
                command_pub_.publish(stop_msg);

                Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
                replan(curr_pos, waypoints_[curr_index_]);
                break;
            }
            loop_rate.sleep();
        }
    }
}

bool LocalPlanner::checkReplan() {
    Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);

    double view_angle = M_PI/6;
    double angle_step = 0.2;
    double check_radius = 2 * robot_radius_;

    Eigen::Vector3d heading = waypoints_[curr_index_] - curr_pos;
    
    double desired_yaw = 0.0;
    if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
        desired_yaw = std::atan2(heading.y(), heading.x());
    }

    bool need_replan = false;

    std::vector<Eigen::Vector3d> free_shield;
    std::vector<Eigen::Vector3d> occupied_shield;

    for(double angle = -view_angle; angle < view_angle; angle += angle_step) {
        for(double step = -1; step <= 1; step++) {
            Eigen::Vector3d pt = curr_pos + Eigen::Vector3d(cos(angle + desired_yaw), sin(angle + desired_yaw), 0) * check_radius;
            pt.z() += step * voxel_size_;
            double distance = 0.0;
            if(pathfinder_.getMapDistance(pt, distance)) { 
                if(distance < 0.0) {
                    occupied_shield.push_back(pt);
                } else { free_shield.push_back(pt); }
            }
        }
    }

    need_replan = (occupied_shield.size() >= 0.5 * free_shield.size());
    // ROS_WARN_STREAM(occupied_shield.size() << " " << (free_shield.size() + occupied_shield.size()));
    visualizer_.visualizePoints("occ_shield", occupied_shield, "world", PathVisualizer::ColorType::RED, 1);
    visualizer_.visualizePoints("free_shield", free_shield, "world", PathVisualizer::ColorType::GREEN, 0.5);
    return need_replan;
}

void LocalPlanner::replan(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    pathfinder_.findPath(start, end);
    waypoints_ = pathfinder_.getPath();
    curr_index_ = 0;
    ROS_INFO("Replanned!");
}

void LocalPlanner::waypointListCallback(const geometry_msgs::PoseArray& msg) {
    for(auto& pose : msg.poses) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose = pose;
        waypointCallback(msg);
    }
}

void LocalPlanner::setYawFacing(geometry_msgs::PoseStamped& msg) {
    geometry_msgs::PoseStamped odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose = odometry_.pose.pose;
   
    mav_msgs::EigenTrajectoryPoint curr_pose, facing_pose;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(odom_msg, &curr_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(msg, &facing_pose);
   
    Eigen::Vector3d heading = facing_pose.position_W - curr_pose.position_W;
    
    double desired_yaw = 0.0;
    if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
        desired_yaw = std::atan2(heading.y(), heading.x());
    }
    
    Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ()));
    msg.pose.orientation.w = quat.w();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
}

} // namespace ariitk::local_planner
