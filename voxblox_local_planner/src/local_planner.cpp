#include <voxblox_local_planner/local_planner.hpp>
#include <mav_msgs/conversions.h>

namespace ariitk::local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : pathfinder_(nh, nh_private) 
    , last_yaw_(0) {
    // pathfinder_.setEsdfMapPtr(server_.getEsdfMapPtr());
    // pathfinder_.init(nh, nh_private);
    nh_private.getParam("visualize", visualize_);

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

    pathfinder_.findBestPath(start_odom.position_W, waypoint.position_W);
    Path waypoints = pathfinder_.getBestPath();
    if(visualize_) { pathfinder_.visualizePaths(); }

    for(auto& point : waypoints) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = point.x();
        msg.pose.position.y = point.y();
        msg.pose.position.z = point.z();
        setYawFacing(msg);
        command_pub_.publish(msg);
        ros::Rate loop_rate(10);
        while(ros::ok() && norm(odometry_.pose.pose.position, msg.pose.position) > 0.2) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
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
    
    double desired_yaw;
    if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
        desired_yaw = std::atan2(heading.y(), heading.x());
    }
    
    double yaw_mod = fmod(desired_yaw - last_yaw_, 2 * M_PI);
    if (yaw_mod < -M_PI) {
        yaw_mod += 2 * M_PI;
    } else if (yaw_mod > M_PI) {
        yaw_mod -= 2 * M_PI;
    }

    double yaw_rate_max = M_PI/4.0;
    double sampling_dt = 0.01;

    if (std::fabs(yaw_mod) > yaw_rate_max * sampling_dt) {
        double yaw_direction = yaw_mod > 0.0 ? 1.0 : -1.0;
        desired_yaw = last_yaw_ + yaw_direction * yaw_rate_max * sampling_dt;
    }
    last_yaw_ = desired_yaw;

    Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ()));
    ROS_WARN_STREAM(desired_yaw);
    msg.pose.orientation.w = quat.w();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
}

} // namespace ariitk::local_planner
