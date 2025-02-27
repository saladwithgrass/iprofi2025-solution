#pragma once

#include <solution.hpp>

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class SimpleMover : public rclcpp::Node
{
public:
    SimpleMover(double rate = 60.0);
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void moveCallback();
    void finishControl();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double rate_;
    double init_time_sec_;
    double finish_time_;
    bool simulation_started_;

    bool gui_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMover>(30.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
