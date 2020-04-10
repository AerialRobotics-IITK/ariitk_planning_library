#pragma once

#include <frontier_explorer/frontier_evaluator.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ariitk_planning_msgs/Frontier.h>

namespace ariitk::frontier_explorer {

class GoalSelector {
    public:
        GoalSelector(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();
        void getActiveFrontiers();
        void scoreFrontiers(std::vector<ariitk_planning_msgs::Frontier>& frontiers);
        void getBestGoal();
        void visualizeActiveFrontiers();
        // void visualizeActiveGoal();

    private:
        static bool selector(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2);
        void odometryCallback(const nav_msgs::Odometry& msg) {  curr_pose_ = msg.pose.pose;  }
        inline std::string getHash(const geometry_msgs::Point& point) const {
            return std::to_string(int(point.x / voxel_size_)) + "," + std::to_string(int(point.y / voxel_size_));
        };
        inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z), 2));
        }

        bool visualize_;
        double voxel_size_;
        double robot_radius_;

        FrontierEvaluator evaluator_;
        
        ros::Subscriber odom_sub_;
        ros::Publisher goal_pub_;
        ros::Publisher active_pub_;

        std::vector<ariitk_planning_msgs::Frontier> active_frontiers_;
        ariitk_planning_msgs::Frontiers frontier_cache_;
        ariitk_planning_msgs::Frontier active_goal_;

        geometry_msgs::Pose curr_pose_;

        std::unordered_map<std::string, geometry_msgs::Point> hash_map_;
};

} // namespace ariitk::frontier_explorer
