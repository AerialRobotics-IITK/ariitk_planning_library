#include <voxblox_local_planner/ray_caster.hpp>

namespace ariitk::local_planner {

bool RayCaster::setInput(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
    start_ = start;
    end_ = end;

    start_corner_ = Eigen::Vector3i(int(std::floor(start.x())), int(std::floor(start.y())), int(std::floor(start.z())));
    end_corner_ = Eigen::Vector3i(int(std::floor(end.x())), int(std::floor(end.y())), int(std::floor(end.z())));

    direction_ = (end - start);
    max_dist_ = direction_.norm();

    delta_ = (end_corner_ - start_corner_).cast<double>(); 
    step_ = Eigen::Vector3i(int(signum(delta_.x())), int(signum(delta_.y())), int(signum(delta_.z())));

    t_max_ = Eigen::Vector3d(bound(step_.x(), delta_.x()), bound(step_.y(), delta_.y()), bound(step_.z(), delta_.z()));
    t_delta_ = Eigen::Vector3d(double(step_.x() / delta_.x()), double(step_.y() / delta_.y()), double(step_.z() / delta_.z()));

    distance_ = 0.0;
    num_step_ = 0;

    return (step_ == Eigen::Vector3i::Zero());
}

bool RayCaster::step(Eigen::Vector3d& point) {
    point = start_corner_.cast<double>();
    if(start_corner_ == end_corner_) { return false; }
    
    if(t_max_.x() < t_max_.y()) {
        if(t_max_.x() < t_max_.z()) {
            start_corner_[0] += step_.x();
            t_max_[0] += t_delta_.x();
        } else {
            start_corner_[2] += step_.z();
            t_max_[2] += t_delta_.z();
        }
    } else {
        if(t_max_.y() < t_max_.z()) {
            start_corner_[1] += step_.y();
            t_max_[1] += t_delta_.y();
        } else {
            start_corner_[2] += step_.z();
            t_max_[2] += t_delta_.z();
        }
    }

    return true;
}

} // namespace ariitk::local_planner

