#include <voxblox_local_planner/local_planner.hpp>
#include <mav_msgs/conversions.h>

namespace ariitk::local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : server_(nh, nh_private)
    , pathfinder_() {
    pathfinder_.setEsdfMapPtr(server_.getEsdfMapPtr());
    pathfinder_.init(nh, nh_private);
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
        msg.pose.orientation.w = 1.0;
        msg.pose.position.x = point.x();
        msg.pose.position.y = point.y();
        msg.pose.position.z = point.z();
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

} // namespace ariitk::local_planner
