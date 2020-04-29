#include <voxblox_local_planner/local_planner.hpp>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
    : pathfinder_(nh, nh_private) 
    , last_yaw_(0) {
    nh_private.getParam("visualize", visualize_);
    nh_private.getParam("robot_radius", robot_radius_);
    nh_private.getParam("voxel_size", voxel_size_);

    smoother_.setParametersFromRos(nh_private);
    smoother_.setMinCollisionCheckResolution(voxel_size_);
    smoother_.setDistanceAndGradientFunction(
        std::bind(&LocalPlanner::getMapDistanceAndGradient, this,
                    std::placeholders::_1, std::placeholders::_2));
    smoother_.setOptimizeTime(true);
    smoother_.setResampleTrajectory(true);
    smoother_.setResampleVisibility(true);
    smoother_.setNumSegments(5);

    visualizer_.init(nh, nh_private);
    visualizer_.createPublisher("occ_shield");
    visualizer_.createPublisher("free_shield");
    visualizer_.createPublisher("trajectory");

    odometry_sub_ = nh.subscribe("odometry", 1, &LocalPlanner::odometryCallback, this);
    waypoint_sub_ = nh.subscribe("waypoint", 1, &LocalPlanner::waypointCallback, this);
    waypoint_list_sub_ = nh.subscribe("waypoint_list", 1, &LocalPlanner::waypointListCallback, this);
    command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    traj_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
}

void LocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
    ROS_INFO("Recieved waypoint!");
    mav_msgs::EigenOdometry start_odom;
    mav_msgs::eigenOdometryFromMsg(odometry_, &start_odom);

    mav_msgs::EigenTrajectoryPoint waypoint;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

    pathfinder_.findPath(start_odom.position_W, waypoint.position_W);
    waypoints_ = pathfinder_.getPath();
    ros::Rate loop_rate(10);
    
    for(curr_index_ = 0; curr_index_ < waypoints_.size(); curr_index_++) {
        mav_msgs::EigenOdometry curr_odom;
        mav_msgs::eigenOdometryFromMsg(odometry_, &curr_odom);

        generateTrajectory(curr_odom.position_W, waypoints_[curr_index_]);

        trajectory_msgs::MultiDOFJointTrajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();

        visualizer_.visualizeTrajectory("trajectory", trajectory_, "world", PathVisualizer::ColorType::BLACK, 0.2);
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_, &traj_msg);
        traj_pub_.publish(traj_msg);

        while(ros::ok() && norm(odometry_.pose.pose.position, waypoints_[curr_index_]) > 0.2) {
            ros::spinOnce();
            if(checkForReplan()) { 
                ROS_WARN("Replanning!");
                geometry_msgs::PoseStamped stop_msg;
                stop_msg.header.stamp = ros::Time::now();
                stop_msg.pose = odometry_.pose.pose;
                command_pub_.publish(stop_msg);

                Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
                replan(curr_pos, waypoints_[curr_index_]);
                break;
            }
            loop_rate.sleep();
        }
    }
}

// bool LocalPlanner::checkForReplan() {
//     // return false;
//     Eigen::Vector3d curr_pos(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);

//     double view_angle = M_PI/8;
//     double angle_step = 0.2;
//     double check_radius = 3 * robot_radius_;

//     geometry_msgs::Quaternion curr_quat = odometry_.pose.pose.orientation;
//     tf::Quaternion quat(curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w);
//     double roll, pitch, curr_yaw;
//     tf::Matrix3x3(quat).getRPY(roll, pitch, curr_yaw);

//     bool need_replan = false;

//     std::vector<Eigen::Vector3d> free_shield;
//     std::vector<Eigen::Vector3d> occupied_shield;

//     for(double angle = -view_angle; angle < view_angle; angle += angle_step) {
//         for(double z_step = -1; z_step <= 1; z_step++) {
//             int free_layer_count = 0;
//             for(double r_step = -1; r_step <= 1; r_step++) {
//                 Eigen::Vector3d pt = curr_pos + Eigen::Vector3d(cos(angle + curr_yaw), sin(angle + curr_yaw), 0) * (check_radius + r_step * voxel_size_);
//                 pt.z() += z_step * voxel_size_;
//                 double distance = 0.0;
//                 if(pathfinder_.getMapDistance(pt, distance)) { 
//                     if(distance < voxel_size_) {
//                         occupied_shield.push_back(pt);
//                     } else { 
//                         free_layer_count++;
//                         free_shield.push_back(pt); 
//                     }
//                 }
//                 if(!need_replan) { need_replan = (free_layer_count == 0); }
//             }
//         }
//     }

//     visualizer_.visualizePoints("occ_shield", occupied_shield, "world", PathVisualizer::ColorType::RED, 1);
//     visualizer_.visualizePoints("free_shield", free_shield, "world", PathVisualizer::ColorType::GREEN, 0.5);
//     return need_replan;
// }

bool LocalPlanner::checkForReplan() {
    if(trajectory_.empty()) { return false; }
    bool need_replan = false;
    std::vector<Eigen::Vector3d> free_points;
    std::vector<Eigen::Vector3d> occ_points;

    double distance = 0.0;
    for(auto& point : trajectory_) {
        if(pathfinder_.getMapDistance(point.position_W, distance) && distance < voxel_size_) {
            occ_points.push_back(point.position_W);
        } else {
            free_points.push_back(point.position_W);
        }
    }

    need_replan = (occ_points.size() > 0);
    visualizer_.visualizePoints("occ_shield", occ_points, "world", PathVisualizer::ColorType::RED, 1);
    visualizer_.visualizePoints("free_shield", free_points, "world", PathVisualizer::ColorType::GREEN, 0.5);
    return need_replan;
}

void LocalPlanner::replan(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    pathfinder_.findPath(start, end);
    waypoints_ = pathfinder_.getPath();
    curr_index_ = 0;
    ROS_INFO("Replanned!");
}

void LocalPlanner::waypointListCallback(const geometry_msgs::PoseArray& msg) {
    for(auto& pose : msg.poses) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose = pose;
        waypointCallback(msg);
    }
}

void LocalPlanner::generateTrajectory(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    trajectory_.clear();
    mav_msgs::EigenTrajectoryPoint start_pt, end_pt;
    start_pt.position_W = start;
    end_pt.position_W = end;
    if(norm(start_pt.position_W, end_pt.position_W) < voxel_size_ || !smoother_.getPathBetweenTwoPoints(start_pt, end_pt, &trajectory_)){
        Path dummy_path;
        dummy_path.push_back(start);
        dummy_path.push_back(end);
        convertPathToTrajectory(dummy_path, trajectory_);
    }
    applyYawToTrajectory();
}

void LocalPlanner::applyYawToTrajectory() {
    if(trajectory_.size() < 2) { return; }
    for(auto i = 0; i < trajectory_.size() - 1; i++) {
        Eigen::Vector3d heading = trajectory_[i+1].position_W - trajectory_[i].position_W;
        double desired_yaw = 0.0;
        if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
           desired_yaw = std::atan2(heading.y(), heading.x());
        }
        trajectory_[i].setFromYaw(desired_yaw);
    }
}

// void LocalPlanner::setYawFacing(geometry_msgs::PoseStamped& msg) {
//     geometry_msgs::PoseStamped odom_msg;
//     odom_msg.header.stamp = ros::Time::now();
//     odom_msg.pose = odometry_.pose.pose;
   
//     mav_msgs::EigenTrajectoryPoint curr_pose, facing_pose;
//     mav_msgs::eigenTrajectoryPointFromPoseMsg(odom_msg, &curr_pose);
//     mav_msgs::eigenTrajectoryPointFromPoseMsg(msg, &facing_pose);
   
//     Eigen::Vector3d heading = facing_pose.position_W - curr_pose.position_W;
    
//     double desired_yaw = 0.0;
//     if (std::fabs(heading.x()) > 1e-4 || std::fabs(heading.y()) > 1e-4) {
//         desired_yaw = std::atan2(heading.y(), heading.x());
//     }
    
//     Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(desired_yaw, Eigen::Vector3d::UnitZ()));
//     msg.pose.orientation.w = quat.w();
//     msg.pose.orientation.x = quat.x();
//     msg.pose.orientation.y = quat.y();
//     msg.pose.orientation.z = quat.z();
// }

void LocalPlanner::convertPathToTrajectory(const Path& path, Trajectory& trajectory) {
    for(auto& point : path) {
        mav_msgs::EigenTrajectoryPoint traj_pt;
        traj_pt.position_W = point;
        trajectory.push_back(traj_pt);
    }
}

} // namespace ariitk::local_planner
