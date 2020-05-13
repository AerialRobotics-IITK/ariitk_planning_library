#pragma once

#include <frontier_explorer/frontier_evaluator.hpp>
#include <frontier_explorer/frontier_comparator.hpp>
#include <geometry_msgs/Point.h>
#include <ariitk_planning_msgs/Frontier.h>
#include <rviz_visualizer/visualizer.hpp>

namespace ariitk::frontier_explorer {

typedef ariitk::rviz_visualizer::Visualizer Visualizer;

class GoalSelector {
    public:
        GoalSelector(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();
        ariitk_planning_msgs::Frontier getBestGoal() { return active_goal_; }
        void getActiveFrontiers();
        void scoreFrontiers(std::vector<ariitk_planning_msgs::Frontier>& frontiers);
        void findBestGoal();
        void visualizeActiveFrontiers();
        void visualizeActiveGoal();

    private:
        inline std::string getHash(const geometry_msgs::Point& point) {
            return std::to_string(int(point.x / voxel_size_)) + "," + std::to_string(int(point.y / voxel_size_));
        };

        bool visualize_;
        double voxel_size_;
        double slice_level_;

        FrontierEvaluator evaluator_;
        FrontierComparator comparator_;

        Visualizer visualizer_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        
        ros::Publisher goal_pub_;

        std::vector<ariitk_planning_msgs::Frontier> active_frontiers_;
        ariitk_planning_msgs::Frontiers frontier_cache_;
        ariitk_planning_msgs::Frontier active_goal_;

        std::unordered_map<std::string, geometry_msgs::Point> hash_map_;
};

} // namespace ariitk::frontier_explorer
