#include <global_planner/map_server.hpp>

using namespace ariitk::global_planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_server_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    MapServer map_server(nh, nh_private);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
