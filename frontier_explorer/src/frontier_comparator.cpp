#include <frontier_explorer/frontier_comparator.hpp>

namespace ariitk::frontier_explorer {

FrontierComparator::FrontierComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) 
    : nh_(nh)
    , nh_private_(nh_private) {
    nh_private_.getParam("min_frontier_size", min_size_);
    nh_private_.getParam("clear_radius", clear_radius_);
    ROS_INFO("%lf", clear_radius_);
    odom_sub_ = nh_.subscribe("odometry", 1, &FrontierComparator::odometryCallback, this);
}

bool FrontierComparator::operator()(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2) {
    double cost1 = norm(f1.center, curr_pose_.position);
    double cost2 = norm(f2.center, curr_pose_.position);
    
    double gain1 = (f1.num_points/min_size_)*(f1.num_points/min_size_);
    double gain2 = (f2.num_points/min_size_)*(f2.num_points/min_size_);

    if(cost1 < clear_radius_) cost1 = DBL_MAX;
    else if(cost2 < clear_radius_) cost2 = DBL_MAX;
    else return ((gain1 - cost1) > (gain2 - cost2)); // need formal policy
}

} // namespace ariitk::frontier_explorer
