#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <robot_msgs/srv/get_transform.hpp>

#include<memory>


class SimpleTFKinematics : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::Service<robot_msgs::srv::GetTransform>::SharedPtr get_transform_srv_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_lisenter_;

    double x_inc_;
    double last_x_;

    void timerCallback();
    bool getTransformCallback(const std::shared_ptr<robot_msgs::srv::GetTransform::Request> req,
                              const std::shared_ptr<robot_msgs::srv::GetTransform::Response> res);
    
public:
    SimpleTFKinematics(const std::string &name);
};


#endif