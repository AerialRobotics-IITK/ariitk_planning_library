#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <Eigen/Core>

namespace ariitk::obstacle_detector {

class ObstacleDetector {
    public:
        ObstacleDetector() {};
        
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();
        
        inline bool isFreeSpaceAvailable() const { return freespace_available_; }
        inline Eigen::Vector3d getFreeSpaceCenter() const { return freespace_center_; }

    private:
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
        void detectObstacles(const pcl::PCLPointCloud2ConstPtr& cloud);

        double robot_radius_;
        double voxel_size_;
        double visible_horizon_;
        double slice_level_;
        double height_range_;
        
        std::string world_frame_id_;
        std::string quad_frame_id_;

        bool freespace_available_;

        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter_;
        pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud_;

        Eigen::Vector3d freespace_center_;
        Eigen::Vector3d freespace_boundary_[4];

        ros::Subscriber point_cloud_sub_;

        ros::Publisher freespace_center_pub_;
        ros::Publisher obstacle_cloud_pub_;
        ros::Publisher boundary_pub_;

        tf::TransformListener tf_listener_;
};

} // namespace ariitk::obstacle_detector
