#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP
#include<rclcpp/rclcpp.hpp>
#include<tf2_ros/static_transform_broadcaster.h>
#include<tf2_ros/transform_broadcaster.h>
#include<geometry_msgs/msg/transform_stamped.hpp>

#include<memory>


class SimpleTFKinematics : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::TimerBase::SharedPtr timer_;

    double x_inc_;
    double last_x_;

    void timerCallback();
    
public:
    SimpleTFKinematics(const std::string &name);
};


#endif