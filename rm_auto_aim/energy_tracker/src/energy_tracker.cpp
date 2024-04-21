#include "energy_tracker/energy_tracker.hpp"
namespace rm_auto_aim
{
    EnTracker::EnTracker() : tracker_state(LOST), mode_state(NONE)
    {
    }

    float EnTracker::angleSolver(auto_aim_interfaces::msg::Leaf leaf)
    {
        float y = leaf.leaf_center.y - leaf.r_center.y;
        float x = leaf.leaf_center.x - leaf.r_center.x;
        float angle = cv::fastAtan2(y, x) / 180.0 * M_PI;
        return angle;
    }

    void EnTracker::init(const Leaf &leaf)
    {   
        initEKF(leaf);
        RCLCPP_DEBUG(rclcpp::get_logger("energy_tracker"), "Init EKF!");
        tracker_state = DETECTING;
    }

    void EnTracker::update(const Leaf &leaf)
    {
        Eigen::VectorXd ekf_prediction = ekf.predict();
        RCLCPP_DEBUG(rclcpp::get_logger("energy_tracker"), "EKF predict");

        target_state = ekf_prediction;
        measurement = Eigen::VectorXd(1);
        measurement << angleSolver(leaf);
        target_state = ekf.update(measurement);
        // Tracking state machine
        if (tracker_state == DETECTING)
        {
            if (matched)
            {
                detect_count_++;
                if (detect_count_ > tracking_thres)
                {
                    detect_count_ = 0;
                    tracker_state = TRACKING;
                }
            }
            else
            {
                detect_count_ = 0;
                tracker_state = LOST;
            }
        }
        else if (tracker_state == TRACKING)
        {
            if (!matched)
            {
                tracker_state = TEMP_LOST;
                lost_count_++;
            }
        }
        else if (tracker_state == TEMP_LOST)
        {
            if (!matched)
            {
                lost_count_++;
                if (lost_count_ > lost_thres)
                {
                    lost_count_ = 0;
                    tracker_state = LOST;
                }
            }
            else
            {
                tracker_state = TRACKING;
                lost_count_ = 0;
            }
        }
    }

    void EnTracker::initEKF(const Leaf &leaf)
    {
        target_state = Eigen::VectorXd::Zero(3);
        float angle = angleSolver(leaf);
        target_state << angle, 0, 0;
        ekf.setState(target_state);
    }
    float EnTracker::small_predict(const Leaf &leaf, double dt_)
    {
        float current_angle = angleSolver(tracked_leaf);
        const double angle_v = 1 / 6; // rad/s
        current_angle += angle_v * dt_;
        return current_angle > 2 * M_PI ? current_angle - 2 * M_PI : current_angle;
    }
}
