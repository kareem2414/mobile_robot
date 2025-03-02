#include "robot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string& name)
    : Node(name)
    , mean_(0.0)
    , variance_(1000.0)
    , imu_angular_z_(0.0)
    , last_angular_z_(0.0)
    , is_first_odom_(true)
    , motion_angular_z(0.0)
    , motion_variance_(4.0)
    , measurment_variance_(0.5)
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("robot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/out", 10, std::bind(&KalmanFilter::imuCallback, this, _1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("robot_controller/odom_kalman",10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry & odom)
{
    kalman_odom_  = odom;
    
    if(is_first_odom_)
    {
        mean_ = odom.twist.twist.angular.z ;
        last_angular_z_ = odom.twist.twist.angular.z ;
        is_first_odom_ = false;
        return;
    }
    motion_angular_z = odom.twist.twist.angular.z - last_angular_z_;
    
    statePrediction();
    measurmentUpdate();

    kalman_odom_.twist.twist.angular.z = mean_;
    odom_pub_ ->publish(kalman_odom_);



}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu & imu)
{
    imu_angular_z_ = imu.angular_velocity.z;

}

void KalmanFilter::measurmentUpdate()
{
    mean_ = (measurment_variance_ * mean_ + variance_ * last_angular_z_) / (variance_ * measurment_variance_);
    variance_ = (variance_ * measurment_variance_) / (variance_ + measurment_variance_);

}

void KalmanFilter::statePrediction()
{
    mean_ += motion_angular_z;
    variance_ += motion_variance_;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<KalmanFilter>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 1;
}

