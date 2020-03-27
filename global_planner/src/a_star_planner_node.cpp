#include <global_planner/a_star_planner.hpp>

using namespace ariitk::global_planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "voxblox_astar_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    AStarPlanner planner(nh, nh_private);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}