#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <frontier_explorer/frontier_evaluator.hpp>
#include <ariitk_planning_msgs/Frontier.h>
#include <mav_planning_msgs/PlannerService.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

namespace ariitk::frontier_explorer {

class FrontierPlanner {
    public:
        FrontierPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();

    private:
        struct queueComparator{
            bool operator()(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2) {
                return frontierComparator(f1, f2);
            }
        };
        static bool frontierComparator(ariitk_planning_msgs::Frontier f1, ariitk_planning_msgs::Frontier f2);
        inline std::string getHash(const geometry_msgs::Point& point) const {
            return std::to_string(int(point.x / voxel_size_)) + "," + std::to_string(int(point.y / voxel_size_));
        };
        inline double norm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
            return std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2) + std::pow((p1.z - p2.z), 2));
        }
        void getActiveFrontiers();
        void findBestActiveFrontier();
        void findNextBestFrontier();
        void odometryCallback(const nav_msgs::Odometry& msg);
        bool wayptServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

        FrontierEvaluator evaluator_;

        std::string world_frame_id_;

        double min_frontier_size_;
        double voxel_size_;

        ros::Subscriber odom_sub_;
        ros::Publisher waypt_pub_;
        
        ros::ServiceClient global_planner_;
        ros::ServiceClient path_publisher_;
        ros::ServiceServer waypt_server_;

        std::vector<ariitk_planning_msgs::Frontier> active_frontiers_;
        ariitk_planning_msgs::Frontier best_frontier_;
        std::priority_queue<ariitk_planning_msgs::Frontier, std::vector<ariitk_planning_msgs::Frontier>, queueComparator> frontier_queue_;

        geometry_msgs::Pose pose_;
        geometry_msgs::Pose last_waypt_;

        std::unordered_map<std::string, geometry_msgs::Point> hash_map_;
};

} // namespace ariitk::frontier_planner
