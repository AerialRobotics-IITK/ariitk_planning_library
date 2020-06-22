#include <voxblox_local_planner/local_planner.hpp>

using namespace ariitk::local_planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_planner");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    FLAGS_alsologtostderr = true;

    LocalPlanner planner(nh, nh_private);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
