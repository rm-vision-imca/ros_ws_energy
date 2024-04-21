#include "energy_tracker/energy_tracker.hpp"
namespace rm_auto_aim
{
    EnTracker::EnTracker()
    {
        tracker_state = LOST;
        mode_state = NONE;
    }

    float EnTracker::angleSolver(auto_aim_interfaces::msg::Leaf leaf)
    {
        float y = leaf.leaf_center.y - leaf.r_center.y;
        float x = leaf.leaf_center.x - leaf.r_center.x;
        float angle = cv::fastAtan2(y, x) / 180.0 * M_PI;
        return angle;
    }

    void EnTracker::init(const Leaf &l)
    {
        initEKF(l);
        RCLCPP_DEBUG(rclcpp::get_logger("energy_tracker"), "Init EKF!");
    }

    void EnTracker::update(const Leaf &l)
    {
        RCLCPP_INFO(rclcpp::get_logger("energy_tracker"), "current angle:%f,e_x:%f,e_y:%f", angleSolver(l) * 180 / M_PI, l.leaf_center.x - l.r_center.x, l.leaf_center.y - l.r_center.y);
        Eigen::VectorXd ekf_prediction = ekf.predict();
        target_state = ekf_prediction;
        measurement = Eigen::VectorXd(1);
        measurement << angleSolver(l);
        target_state = ekf.update(measurement);
    }

    void EnTracker::initEKF(const Leaf &l)
    {
        target_state = Eigen::VectorXd::Zero(3);
        float angle = angleSolver(l);
        target_state << angle, 0, 0;
        ekf.setState(target_state);
    }
    float EnTracker::small_predict(const Leaf &leafs_msg,double dt_)
    {
        float current_angle=angleSolver(leafs_msg);
        const double angle_v=1/6;//rad/s
        current_angle+=angle_v*dt_;
        return current_angle>2*M_PI?current_angle-2*M_PI:current_angle;
    }
}
