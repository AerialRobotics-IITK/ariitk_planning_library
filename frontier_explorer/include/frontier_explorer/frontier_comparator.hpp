#pragma once

#include <ros/ros.h>
#include <ariitk_planning_msgs/Frontier.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace ariitk::frontier_explorer {

class FrontierComparator {
    public:
        FrontierComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        bool operator()(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2);
    
    private:
        void odometryCallback(const nav_msgs::Odometry& msg) { curr_pose_ = msg.pose.pose; }
        static inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z), 2));
        }

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber odom_sub_;

        double clear_radius_;
        double min_size_;
        geometry_msgs::Pose curr_pose_;
};

} // namespace ariitk::frontier_explorer
