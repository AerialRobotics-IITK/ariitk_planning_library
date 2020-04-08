#include <frontier_explorer/frontier_planner.hpp>

using namespace ariitk::frontier_explorer;

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_planner_node");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh, nh_private("~");

    FLAGS_alsologtostderr = true;

    FrontierPlanner planner(nh, nh_private);
    ROS_INFO_STREAM("Frontier Planner has started");

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        planner.run();
        loop_rate.sleep();
    }
}
