#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Core>

class SimpleController : public rclcpp::Node
{
    public:
    SimpleController(const std::string & name);

    private:
        void velCallback(const geometry_msgs::msg::TwistStamped & msg);
        void jointCallback(const sensor_msgs::msg::JointState &msg);

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        double wheel_radius_;
        double wheel_seperation_;
        Eigen::Matrix2d speed_conversion_;

        double left_wheel_prev_pose_;
        double right_wheel_prev_pose_;
        rclcpp::Time prev_time_;

        double x_;
        double y_;
        double theta_;

};

#endif