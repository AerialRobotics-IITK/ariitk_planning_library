#include <frontier_explorer/frontier_planner.hpp>

#include <ariitk_planning_msgs/Frontiers.h>

namespace ariitk::frontier_explorer {

FrontierPlanner::FrontierPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    :   evaluator_(nh, nh_private) {
    nh_private.getParam("min_frontier_size", min_frontier_size_);
    nh_private.getParam("voxel_size", voxel_size_);
    nh_private.getParam("world_frame", world_frame_id_);

    odom_sub_ = nh.subscribe("odometry", 1, &FrontierPlanner::odometryCallback, this);
    waypt_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("waypoint", 1);  
    waypt_server_ = nh_private.advertiseService("send_waypt", &FrontierPlanner::wayptServerCallback, this);
    global_planner_ = nh.serviceClient<mav_planning_msgs::PlannerService>("plan");
    path_publisher_ = nh.serviceClient<std_srvs::Empty>("publish_path");

    last_waypt_.position.z = -1;
}

void FrontierPlanner::run() {
    if(last_waypt_.position.z == -1) {
        ROS_INFO("Publishing first waypoint!");
        ariitk_planning_msgs::Frontiers frontiers;
        while(frontiers.frontiers.empty()) {
            evaluator_.run();
            ros::spinOnce();
            frontiers = evaluator_.getFrontiers();
        }
        getActiveFrontiers();
        findBestActiveFrontier();
        geometry_msgs::PoseStamped waypt_msg;
        waypt_msg.header.frame_id = world_frame_id_;
        waypt_msg.header.stamp = ros::Time::now();
        waypt_msg.pose.position = best_frontier_.center;
        waypt_msg.pose.orientation = pose_.orientation;
        while(!waypt_pub_.getNumSubscribers());
        waypt_pub_.publish(waypt_msg);
        last_waypt_ = waypt_msg.pose;
    } else {
        evaluator_.run();
        getActiveFrontiers();
    }
}

void FrontierPlanner::getActiveFrontiers() {
    active_frontiers_.clear();
    ariitk_planning_msgs::Frontiers frontiers = evaluator_.getFrontiers();
    for(auto& frontier : frontiers.frontiers) {
        if(frontier.num_points < min_frontier_size_) { continue; }
        frontier_queue_.push(frontier);
        std::string hash = getHash(frontier.center);
        if(hash_map_.find(hash) == hash_map_.end()) {
            hash_map_[hash] = frontier.center;
            active_frontiers_.push_back(frontier);
        }
    }
}

void FrontierPlanner::findBestActiveFrontier() {
    if(!active_frontiers_.empty()) {
        // select from active frontiers
        std::sort(active_frontiers_.begin(), active_frontiers_.end(), FrontierPlanner::frontierComparator);
        best_frontier_ = active_frontiers_[0];
    } else if(!frontier_queue_.empty()){
        // select based on stored frontiers
        best_frontier_ = frontier_queue_.top();
    } else {
        ROS_ERROR("No frontiers left!");
    }
}

void FrontierPlanner::findNextBestFrontier() {
    if(!active_frontiers_.empty()) {
        active_frontiers_.erase(active_frontiers_.begin());
    } else {
        frontier_queue_.pop();
    }
    findBestActiveFrontier();
}   

bool FrontierPlanner::frontierComparator(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2) {
    bool result = (f1.num_points > f2.num_points);
    return result;
}

void FrontierPlanner::odometryCallback(const nav_msgs::Odometry& msg) {
    pose_ = msg.pose.pose;
}

bool FrontierPlanner::wayptServerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    ROS_INFO("Serving new point.");

    if(!req.data) {
        // what to do if local plan failed.
        ROS_WARN("Planning failed!");
        mav_planning_msgs::PlannerService msg;
        msg.request.start_pose.pose = pose_;
        msg.request.goal_pose.pose = last_waypt_;
        msg.request.goal_pose.header.stamp = ros::Time::now();
        global_planner_.waitForExistence();
        global_planner_.call(msg);

        ROS_WARN("%lf %lf %lf --> %lf %lf %lf", msg.request.start_pose.pose.position.x, msg.request.start_pose.pose.position.y,
                                                msg.request.start_pose.pose.position.z, msg.request.goal_pose.pose.position.x,
                                                msg.request.goal_pose.pose.position.y, msg.request.goal_pose.pose.position.z);

        while(!msg.response.success) {
            ROS_INFO("Gonna check other frontiers!");
            // findNextBestFrontier();
            msg.request.start_pose.pose = pose_;
            msg.request.goal_pose.pose.position = last_waypt_.position;
            msg.request.goal_pose.pose.orientation = pose_.orientation;
            msg.request.goal_pose.header.stamp = ros::Time::now();

            ROS_WARN("%lf %lf %lf --> %lf %lf %lf", msg.request.start_pose.pose.position.x, msg.request.start_pose.pose.position.y,
                                                msg.request.start_pose.pose.position.z, msg.request.goal_pose.pose.position.x,
                                                msg.request.goal_pose.pose.position.y, msg.request.goal_pose.pose.position.z);
            global_planner_.call(msg);
        }
        std_srvs::Empty trig;
        path_publisher_.call(trig);
    } else {
        // Next frontier
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = world_frame_id_;
        msg.pose.orientation = pose_.orientation;
        ROS_INFO("Moving to next frontier!");
        findNextBestFrontier();
        msg.pose.position = best_frontier_.center;
        last_waypt_.position = msg.pose.position;
        waypt_pub_.publish(msg);
    }

    return true;
}

} // namespace ariitk::frontier_explorer
