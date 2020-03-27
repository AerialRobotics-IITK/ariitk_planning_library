#include <global_planner/a_star_planner.hpp>

namespace ariitk::global_planner {

AStarPlanner::AStarPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : map_server_(nh, nh_private) {
    
    nh_private.getParam("visualize", visualize_);

    pathfinder_.setParams(nh_private);
    pathfinder_.init();
    pathfinder_.setMapPtr(map_server_.getESDFMapPtr());

    plan_server_ = nh_private.advertiseService("plan", &AStarPlanner::planServiceCallback, this);
}

bool AStarPlanner::planServiceCallback(mav_planning_msgs::PlannerService::Request& req, mav_planning_msgs::PlannerService::Response& resp) {
    mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

    mav_msgs::eigenTrajectoryPointFromPoseMsg(req.start_pose, &start_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(req.goal_pose, &goal_pose);

    pathfinder_.reset();
    AStar::Result res = pathfinder_.search(start_pose.position_W, goal_pose.position_W);
    if(res == AStar::Result::REACH_END) {
        resp.success = true;
        std::vector<Eigen::Vector3d> waypoints = pathfinder_.getPath();
        ROS_INFO_STREAM( waypoints.size() );
    } else {
        resp.success = false;    
        return false;
    }

    return true;
}

} // namespace ariitk::global_planner