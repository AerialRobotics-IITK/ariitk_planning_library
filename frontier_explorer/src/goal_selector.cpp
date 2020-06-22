#include <frontier_explorer/goal_selector.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ariitk::frontier_explorer {

FrontierComparator::FrontierComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private) {
    nh_private_.getParam("min_frontier_size", min_size_);
    nh_private_.getParam("clear_radius", clear_radius_);

    odom_sub_ = nh_.subscribe("odometry", 1, &FrontierComparator::odometryCallback, this);
}

GoalSelector::GoalSelector(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh)
    , nh_private_(nh_private)
    , evaluator_(nh, nh_private)
    , comparator_(nh, nh_private) {
    nh_private_.getParam("voxel_size", voxel_size_);
    nh_private_.getParam("visualize", visualize_);
    nh_private_.getParam("slice_level", slice_level_);

    goal_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("goal", 1);
    active_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("active_frontiers", 1);
}

void GoalSelector::run() {
    evaluator_.run();

    getActiveFrontiers();
    getBestGoal();

    if (visualize_) {
        visualizeActiveFrontiers();
        visualizeActiveGoal();
    }
}

void GoalSelector::getActiveFrontiers() {
    active_frontiers_.clear();  // clear active cache
    frontier_cache_ = evaluator_.getFrontiers();

    for (auto& frontier : frontier_cache_.frontiers) {
        std::string hash = getHash(frontier.center);
        if (hash_map_.find(hash) == hash_map_.end()) {
            hash_map_[hash] = frontier.center;
            active_frontiers_.push_back(frontier);
        }
    }
}

void GoalSelector::visualizeActiveFrontiers() {
    // cache active frontiers for viewing
    static std::vector<ariitk_planning_msgs::Frontier> active_frontiers;

    if (!active_frontiers_.empty())
        active_frontiers = active_frontiers_;
    else {
        return;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "active_frontiers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = marker.scale.y = marker.scale.y = voxel_size_ * 2.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.color.g = 0.0;

    for (auto& point : active_frontiers) { marker.points.push_back(point.center); }

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker);
    active_pub_.publish(markers);
}

bool FrontierComparator::operator()(const ariitk_planning_msgs::Frontier f1, const ariitk_planning_msgs::Frontier f2) {
    double cost1 = getDistance(f1.center, mav_pose_.position);
    double cost2 = getDistance(f2.center, mav_pose_.position);

    double gain1 = std::pow((f1.num_points / min_size_), 2.0);
    double gain2 = std::pow((f2.num_points / min_size_), 2.0);

    if (cost1 < clear_radius_)
        cost1 = DBL_MAX;
    else if (cost2 < clear_radius_)
        cost2 = DBL_MAX;
    else
        return ((gain1 - cost1) > (gain2 - cost2));  // TODO: mathematically proven evaluation policy
}

void GoalSelector::scoreFrontiers(std::vector<ariitk_planning_msgs::Frontier>& frontiers) {
    if (frontiers.empty()) { return; }
    std::sort(frontiers.begin(), frontiers.end(), comparator_);
}

void GoalSelector::getBestGoal() {
    if (!active_frontiers_.empty()) {
        scoreFrontiers(active_frontiers_);
        active_goal_ = active_frontiers_[0];
    } else if (!frontier_cache_.frontiers.empty()) {
        scoreFrontiers(frontier_cache_.frontiers);  // TODO: Consider separate evaluation policies for active and cached frontiers
        active_goal_ = frontier_cache_.frontiers[0];
    }
    // FIXME: need an else?
}

void GoalSelector::visualizeActiveGoal() {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position = active_goal_.center;
    msg.pose.position.z = slice_level_;
    goal_pub_.publish(msg);
}

}  // namespace ariitk::frontier_explorer
