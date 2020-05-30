#include <frontier_explorer/frontier_evaluator.hpp>

using namespace ariitk::frontier_explorer;

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_evaluator");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    FLAGS_alsologtostderr = true;

    FrontierEvaluator evaluator(nh, nh_private);

    double update_rate = 10.0;
    nh_private.getParam("update_frontiers_every_n_sec", update_rate);

    ros::Rate loop_rate(update_rate);
    
    while(ros::ok()) {
        ros::spinOnce();
        evaluator.run();
        loop_rate.sleep();
    }
    
    return 0;
}
