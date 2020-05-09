#include <voxblox_global_planner/a_star_planner.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_star_planner");
    
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ariitk::global_planner::AStarPlanner planner_node(nh,nh_private);

    ros::Rate loop_rate(10);
    int count=0;
    while(ros::ok()) {
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}