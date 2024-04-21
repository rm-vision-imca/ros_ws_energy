#ifndef ENERGY_TRACKER_TRACKER_HPP_
#define ENERGY_TRACKER_TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <memory>
#include <string>

#include "energy_extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/leaf.hpp"
#include "auto_aim_interfaces/msg/leafs.hpp"
#include <opencv2/opencv.hpp>
namespace rm_auto_aim
{
    class EnTracker
    {
    public:
        using Leafs = auto_aim_interfaces::msg::Leafs;
        using Leaf = auto_aim_interfaces::msg::Leaf;
        EnTracker();
        int tracking_thres;
        int lost_thres;
        bool matched=false;
        Leaf tracked_leaf;
        float angleSolver(auto_aim_interfaces::msg::Leaf leaf);
        float small_predict(const Leaf &leaf, double dt_);
        void init(const Leaf &leaf);
        void update(const Leaf &leaf);
        ExtendedKalmanFilter ekf;
        Eigen::VectorXd target_state;
        Eigen::VectorXd measurement;
        enum State
        {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
        } tracker_state;
        enum Mode
        {
            NONE = 0,
            AUTOFIRE,
            SMALL,
            BIG,
        };
        int mode_state;

        int detect_count_;
        int lost_count_;

    private:
        void initEKF(const Leaf &a);
    };

}
#endif