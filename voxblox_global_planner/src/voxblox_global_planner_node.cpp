#include <voxblox_global_planner/skeleton_global_planner.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "skeleton_global_planner");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    FLAGS_alsologtostderr = true;

    ariitk::global_planner::SkeletonGlobalPlanner planner_node(nh, nh_private);
    ROS_INFO("Initialized skeleton global planner node.");

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
