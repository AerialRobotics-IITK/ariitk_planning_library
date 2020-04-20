#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

namespace ariitk::local_planner {

class RayCaster {
    public:
        void setOffset(const Eigen::Vector3d& offset) { offset_ = offset; }
        void addOffset(Eigen::Vector3d& point) { point += offset_; }
        bool setInput(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        bool step(Eigen::Vector3d& point);

    private:
        static inline int signum(const int& x) {
            return (x == 0) ? 0 : (x < 0) ? -1 : 1;
        }
        static inline double bound(const double& s, const double& ds) {
            assert(ds != 0);
            if(ds < 0) { return bound(-s, -ds); }
            else { return (1 - std::fmod(std::fmod(s, 1) + 1, 1)) / ds; }
        }

        Eigen::Vector3d start_, end_, offset_;
        Eigen::Vector3i start_corner_, end_corner_, step_;
        Eigen::Vector3d direction_, delta_;
        Eigen::Vector3d t_max_, t_delta_;
        double distance_, num_step_, max_dist_;
};

} // namespace ariitk::local_planner
