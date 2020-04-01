#include <voxblox_global_planner/frontier_evaluator/frontier_evaluator.hpp>

using namespace ariitk::global_planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_evaluator");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    FLAGS_alsologtostderr = true;

    FrontierEvaluator evaluator(nh, nh_private);

    ros::Rate loop_rate(10);
    
    while(ros::ok()) {
        ros::spinOnce();
        evaluator.visualizeVoxelStates();
        evaluator.visualizeFrontiers();
        loop_rate.sleep();
    }
    
    return 0;
}