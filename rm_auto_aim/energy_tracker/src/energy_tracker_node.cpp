#include "energy_tracker/energy_tracker_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{

   EnergyTrackerNode::EnergyTrackerNode(const rclcpp::NodeOptions &options)
       : Node("energy_tracker", options)
   {
      RCLCPP_INFO(this->get_logger(), "Starting EnergyTarckerNode!");
      tracker_ = std::make_unique<EnTracker>();
      auto f = [this]()
      {
         Eigen::MatrixXd F(3, 3);
         // clang-format off
         F << 1*dt_, 1*dt_, 0.5*dt_*dt_,
              0,    1,      1*dt_,
              0,    0,      1;
         // clang-format on
         return F;
      };
      tracker_->mode_state=EnTracker::SMALL;
      tracker_->ekf = ExtendedKalmanFilter(f);
      // reset srv
      reset_EnTracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
          "/EnTracker/reset", [this](
                                  const std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
          {
      tracker_->tracker_state = EnTracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return; });
      // Subscriber with tf2 message_filter
      // tf2 relevant
      tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      // Create the timer interface before call to waitForTransform,
      // to avoid a tf2_ros::CreateTimerInterfaceException exception
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
          this->get_node_base_interface(), this->get_node_timers_interface());
      tf2_buffer_->setCreateTimerInterface(timer_interface);
      tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
      // subscriber and filter
      leafs_sub_.subscribe(this, "/detector/leafs", rmw_qos_profile_sensor_data);
      target_frame_ = this->declare_parameter("target_frame", "odom");
      tf2_filter_ = std::make_shared<tf2_filter>(
          leafs_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
          this->get_node_clock_interface(), std::chrono::duration<int>(1));
      // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
      tf2_filter_->registerCallback(&EnergyTrackerNode::LeafsCallback, this);
      // Publisher
      target_pub_ = this->create_publisher<auto_aim_interfaces::msg::EnTarget>(
          "/tracker/LeafTarget", rclcpp::SensorDataQoS());
      target_2d_pub_ = this->create_publisher<auto_aim_interfaces::msg::Tracker2D>("entracker/Target2D", rclcpp::SensorDataQoS());
   }
   float EnergyTrackerNode::monoDirectionalAirResistanceModel(float s, float v, float angle){
      float z,k=0.01;
    //t为给定v与angle时的飞行时间
    float t = (float)((exp(k * s) - 1) / (k * v * cos(angle)));
    if(t < 0)
    {
        //目标点超出最大射程
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
   }

   float EnergyTrackerNode::pitchTrajectoryCompensation(float s, float z, float v){
      float z_temp, z_actual, dz;
    float angle_pitch;
    z_temp = z;
    // iteration
    for (int i = 0; i < 30; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        //RCLCPP_INFO(get_logger(),"angle_pitch: %f,z_temp:%f,s:%f",angle_pitch,z_temp,s);
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        //RCLCPP_INFO(get_logger(),"z_actual: %f",z_actual);

        if(!z_actual)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
   }
   void EnergyTrackerNode::LeafsCallback(const auto_aim_interfaces::msg::Leafs::SharedPtr leafs_msg)
   {
      auto_aim_interfaces::msg::EnTarget target_msg;
      auto_aim_interfaces::msg::Tracker2D target_msg_2d;
      if (leafs_msg->leafs.empty()){
         target_msg.yaw=0;
         target_msg.pitch=0;
         target_pub_->publish(target_msg);
         target_2d_pub_->publish(target_msg_2d);
         return;
      }
      // find the best match leaf
      auto leaf_ = leafs_msg->leafs[0];
      tracker_->mode_state=leaf_.mode;
      for (auto &leaf : leafs_msg->leafs)
      {
         if (leaf_.prob > leaf.prob)
         {
            leaf_ = leaf;
         }
      }
      geometry_msgs::msg::PoseStamped ps;
      ps.header = leafs_msg->header;
      ps.pose = leaf_.pose;
      if (!leaf_.type)
         return;
      try
      {
         leaf_.pose = tf2_buffer_->transform(ps, target_frame_).pose;
         //RCLCPP_INFO(get_logger(),"odom x:%f y:%f z:%f ",leaf_.pose.position.x,leaf_.pose.position.y,leaf_.pose.position.z);
         
      }
      catch (const tf2::ExtrapolationException &ex)
      {
         RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
         return;
      }

      // Init message
      // auto_aim_interfaces::msg::TrackerInfo info_msg;
      
      rclcpp::Time time = leafs_msg->header.stamp;
      target_msg.header.stamp = time;
      target_msg.header.frame_id = target_frame_;
      float angle_;
      // small predict
      if (tracker_->mode_state == EnTracker::SMALL)
      {
         if (tracker_->tracker_state == EnTracker::LOST)
         {
            angle0 = tracker_->angleSolver(leaf_);
            last_time_ = time;
            tracker_->tracker_state = EnTracker::TRACKING;
            return;
         }
         dt_ = (time.seconds() - last_time_.seconds());
         if (rad_destation == 0)
            {
               rad_destation = angle1 - angle0 < 0 ? 1 : -1;
            //RCLCPP_INFO(get_logger(),"rad: %d",rad_destation);
            }
         //RCLCPP_INFO(get_logger(),"small pre");
         //angle_ = tracker_->small_predict(leaf_, dt_);
         angle_ = tracker_->angleSolver(leaf_);
      }
      // big predict
      else if (tracker_->mode_state == EnTracker::BIG)
      {
         // Update tracker
         if (tracker_->tracker_state == EnTracker::LOST)
         {
            tracker_->init(leaf_);
            angle0 = tracker_->angleSolver(leaf_);
            //RCLCPP_INFO(this->get_logger(), "angle0:%f", angle0);
            last_time_ = time;
            tracker_->tracker_state = EnTracker::TRACKING;
            return;
         }
         angle1 = tracker_->angleSolver(leaf_);
         if (rad_destation == 0)
         {
            rad_destation = angle1 - angle0 < 0 ? 1 : -1;
            //RCLCPP_INFO(this->get_logger(), "angle1:%f", angle1);
         }
         dt_ = (time.seconds() - last_time_.seconds()) ;
         tracker_->ekf.t = dt_;
         tracker_->update(leaf_);
         const auto &state = tracker_->target_state;
         angle_ = state(0);
         //float angle_v = state(1);
         //RCLCPP_INFO(rclcpp::get_logger("energy_tracker"), "predict angle:%f,angle_v:%f,dt_:%f", angle_ * 180 / M_PI, angle_v * tracker_->ekf.t, dt_);
      }
      else
         angle_ = tracker_->angleSolver(leaf_);
      Eigen::Vector2d p1(leaf_.leaf_center.x, leaf_.leaf_center.y);
      Eigen::Vector2d p2(leaf_.r_center.x, leaf_.r_center.y);
      float r_distance = (p1 - p2).norm();
      target_msg_2d.x = leaf_.r_center.x + r_distance * cos(angle_) * rad_destation;
      target_msg_2d.y = leaf_.r_center.y + r_distance * sin(angle_) * rad_destation;
      float x_ratio=leaf_.leaf_center.x/target_msg_2d.x,y_ratio=leaf_.leaf_center.y/target_msg_2d.y;
      //RCLCPP_INFO(rclcpp::get_logger("energy_tracker"),"x_ratio:%f,y_ratio:%f",x_ratio,y_ratio);
      target_msg.position.x = leaf_.pose.position.x;
      target_msg.position.y = leaf_.pose.position.y*x_ratio;
      target_msg.position.z = leaf_.pose.position.z*y_ratio;
      target_msg.angle = angle_;
      //RCLCPP_INFO(rclcpp::get_logger("energy_tracker"), "predict x: %f,predict y: %f,predict z: %f", target_msg.position.x, target_msg.position.y,target_msg.position.z);
      
      target_msg.yaw = atan2(target_msg.position.y, target_msg.position.x);
      float bottom_len = sqrt(pow(target_msg.position.x, 2.0) + pow(target_msg.position.y, 2.0));
      //target_msg.pitch = atan2(target_msg.position.y, bottom_len); //   pitch angle
      target_msg.pitch =pitchTrajectoryCompensation(bottom_len,target_msg.position.z,v);
      last_time_ = time;
      RCLCPP_INFO(rclcpp::get_logger("energy_tracker"), "y:%f,x:%f,yaw:%f,pitch:%f",target_msg.position.y,target_msg.position.x,target_msg.yaw,target_msg.pitch);

      target_pub_->publish(target_msg);
      target_2d_pub_->publish(target_msg_2d);
   }

} // namespace rm_auto_aim
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::EnergyTrackerNode)