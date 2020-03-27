#pragma once

#include <global_planner/a_star.hpp>
#include <global_planner/map_server.hpp>

#include <mav_planning_msgs/PlannerService.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

namespace ariitk::global_planner {

class AStarPlanner {
    public:
        AStarPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    private:
        AStar pathfinder_;
        MapServer map_server_;
        
        ros::ServiceServer plan_server_;

        bool visualize_;

        bool planServiceCallback(mav_planning_msgs::PlannerService::Request& req, mav_planning_msgs::PlannerService::Response& res);
};

} // namespace ariitk::global_planner