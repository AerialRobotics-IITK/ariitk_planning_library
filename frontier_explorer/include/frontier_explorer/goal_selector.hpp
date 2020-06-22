#pragma once

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <ariitk_planning_msgs/Frontier.h>
#include <frontier_explorer/frontier_evaluator.hpp>

namespace ariitk::frontier_explorer {

class FrontierComparator {
    public:
        FrontierComparator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
        bool operator()(const ariitk_planning_msgs::Frontier f1, const  ariitk_planning_msgs::Frontier f2);

    private:
        void odometryCallback(const nav_msgs::Odometry& msg) { mav_pose_ = msg.pose.pose; }
        static inline double getDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
        }

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber odom_sub_;

        double clear_radius_;
        double min_size_;
        geometry_msgs::Pose mav_pose_;
};

class GoalSelector {
    public:
        GoalSelector(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

        void run();
        void scoreFrontiers(std::vector<ariitk_planning_msgs::Frontier>& frontiers);

        void getActiveFrontiers();
        void getBestGoal();

        void visualizeActiveFrontiers();
        void visualizeActiveGoal();

    private:
        inline std::string getHash(const geometry_msgs::Point& point) const {
            return std::to_string(int(point.x / voxel_size_)) + "," + std::to_string(int(point.y / voxel_size_));
        };

        FrontierEvaluator evaluator_;
        FrontierComparator comparator_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher goal_pub_;
        ros::Publisher active_pub_;

        std::vector<ariitk_planning_msgs::Frontier> active_frontiers_;
        ariitk_planning_msgs::Frontiers frontier_cache_;
        ariitk_planning_msgs::Frontier active_goal_;

        std::unordered_map<std::string, geometry_msgs::Point> hash_map_;

        bool visualize_;

        double voxel_size_;
        double slice_level_;
};

}  // namespace ariitk::frontier_explorer
