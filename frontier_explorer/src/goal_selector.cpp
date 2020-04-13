#include <frontier_explorer/goal_selector.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace ariitk::frontier_explorer {

GoalSelector::GoalSelector(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : evaluator_(nh, nh_private) {
    nh_private.getParam("voxel_size", voxel_size_);
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("clear_radius", clear_radius_);
    
    odom_sub_ = nh.subscribe("odometry", 1, &GoalSelector::odometryCallback, this);
    goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("goal", 1);
    active_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>("active_frontiers", 1);
}

void GoalSelector::run() {
    evaluator_.run();
    getActiveFrontiers();
    getBestGoal();
    if(visualize_) {
        visualizeActiveFrontiers();
        visualizeActiveGoal();
    }
}

void GoalSelector::getActiveFrontiers() {
    active_frontiers_.clear(); // clear active cache
    frontier_cache_ = evaluator_.getFrontiers();
    for(auto& frontier : frontier_cache_.frontiers) {
        std::string hash = getHash(frontier.center);
        if(hash_map_.find(hash) == hash_map_.end()) {
            hash_map_[hash] = frontier.center;
            active_frontiers_.push_back(frontier);
        }
    }
}

void GoalSelector::visualizeActiveFrontiers() {
    // cache active frontiers for viewing
    static std::vector<ariitk_planning_msgs::Frontier> active_frontiers;

    if(!active_frontiers_.empty()) active_frontiers = active_frontiers_;
    else {return;}

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "active_frontiers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_ * 2.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0; marker.color.b = 0.0;
    marker.color.a = 1.0; marker.color.g = 0.0;

    for(auto& point : active_frontiers) {  marker.points.push_back(point.center); }
    
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);
    active_pub_.publish(markers);
}

void GoalSelector::scoreFrontiers(std::vector<ariitk_planning_msgs::Frontier>& frontiers) {
    if(frontiers.empty()) { return; }
    std::sort(frontiers.begin(), frontiers.end(), GoalSelector::FrontierComparator(clear_radius_, curr_pose_.position));
}

void GoalSelector::getBestGoal() {
    if(!active_frontiers_.empty()) {
        scoreFrontiers(active_frontiers_);
        active_goal_ = active_frontiers_[0];
    }
    else if(!frontier_cache_.frontiers.empty()) {
        scoreFrontiers(frontier_cache_.frontiers); // might need separate evaulation policies
        active_goal_ = frontier_cache_.frontiers[0];
    } 
    // need an else?
}

void GoalSelector::visualizeActiveGoal() {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position = active_goal_.center;
    msg.pose.position.z = curr_pose_.position.z;
    goal_pub_.publish(msg);
}

} // namespace ariitk::frontier_explorer
