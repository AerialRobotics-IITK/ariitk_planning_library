#include <obstacle_detector/obstacle_detector.hpp>

using namespace ariitk::obstacle_detector;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh, nh_private("~");

    ObstacleDetector detector;
    detector.init(nh, nh_private);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        detector.run();
        loop_rate.sleep();
    }

    return 0;
}
