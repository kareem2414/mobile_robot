#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "robot_msgs/srv/"
#include <memory>

class SimpleTFKinematics : public rclcpp::Node
{
    public:
        SimpleTFKinematics(const std::string &name);
    
    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
        geometry_msgs::msg::TransformStamped static_transform_stamped_;
        geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

        // rclcpp::Service<>::Shared_Ptr get_transform_srv_;

        rclcpp::TimerBase::SharedPtr timer_;

        double last_x_;
        double x_inc_;

        void timerCallback();

};

#endif