#include <frontier_explorer/goal_selector.hpp>

using namespace ariitk::frontier_explorer;

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_evaluator");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    FLAGS_alsologtostderr = true;

    GoalSelector evaluator(nh, nh_private);

    ros::Rate loop_rate(10);
    
    while(ros::ok()) {
        ros::spinOnce();
        evaluator.run();
        loop_rate.sleep();
    }
    
    return 0;
}
