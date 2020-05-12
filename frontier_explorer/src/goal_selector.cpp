#include <frontier_explorer/goal_selector.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace ariitk::frontier_explorer {

GoalSelector::GoalSelector(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private)
    , evaluator_(nh, nh_private)
    , comparator_(nh, nh_private) {
    nh_private_.getParam("voxel_size", voxel_size_);
    nh_private_.getParam("visualize", visualize_);
    nh_private_.getParam("slice_level", slice_level_);
    
    goal_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("goal", 1);
    
    if(visualize_) {
        visualizer_.init(nh, nh_private);
        visualizer_.createPublisher("active_frontiers");
    }
}

void GoalSelector::run() {
    evaluator_.run();
    getActiveFrontiers();
    findBestGoal();
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

    std::vector<Eigen::Vector3d> frontier_centers;
    for(auto& frontier : active_frontiers) {
        Eigen::Vector3d point(frontier.center.x, frontier.center.y, frontier.center.z);
        frontier_centers.push_back(point); 
    }

    visualizer_.visualizePoints("active_frontiers", frontier_centers, "world", Visualizer::ColorType::RED, 2.0);
}

void GoalSelector::scoreFrontiers(std::vector<ariitk_planning_msgs::Frontier>& frontiers) {
    if(frontiers.empty()) { return; }
    std::sort(frontiers.begin(), frontiers.end(), comparator_);
}

void GoalSelector::findBestGoal() {
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
    msg.pose.position.z = slice_level_;
    goal_pub_.publish(msg);
}

} // namespace ariitk::frontier_explorer
