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
#include <solution.hpp>

SimpleMover::SimpleMover(double rate ): Node("solution"), rate_(rate){
    RCLCPP_INFO(this->get_logger(), "solution node started...");

    // Паблишер для управления (cmd_vel)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Подписка на топик с изображениями
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&SimpleMover::cameraCallback, this, std::placeholders::_1)
    );

    // Подписка на топик с одометрией
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&SimpleMover::gpsCallback, this, std::placeholders::_1)
    );

    // Подписка на топик с точечным облаком
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/points", 10,
        std::bind(&SimpleMover::pointsCallback, this, std::placeholders::_1)
    );

    // Инициализация параметров времени
    init_time_sec_ = this->get_clock()->now().nanoseconds() / 1e9;
    finish_time_ = 10 * 60; // Продолжительность работы: 10 минут
    simulation_started_ = false;
    char const* gui_param = std::getenv("GUI");
    gui_ = (std::string(gui_param) == "True" || std::string(gui_param) == "true" );

    auto timer_period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&SimpleMover::moveCallback, this)
    );
}

void SimpleMover::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    try{
        // Конвертация ROS-сообщения в cv::Mat (формат BGR8)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat cv_image = cv_ptr->image;

        int height = cv_image.rows;
        int width = cv_image.cols;
        int channels = cv_image.channels();

        RCLCPP_INFO(this->get_logger(), "Image received: %dx%d, channels: %d", width, height, channels);
        
        if(gui_){
            // Вывод изображения в окно OpenCV
            cv::imshow("Camera", cv_image);
            cv::waitKey(1);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}

void SimpleMover::gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(),
                "The position of robot is: x=%.2f, y=%.2f",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y);
    }

void SimpleMover::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    // Для демонстрации выводим размер полученных данных
    RCLCPP_INFO(this->get_logger(), "PointCloud2 received: data size = %zu", msg->data.size());
}

void SimpleMover::moveCallback(){
    auto twist_msg = geometry_msgs::msg::Twist();
    double now_sec = this->get_clock()->now().nanoseconds() / 1e9;
    double elapsed_sec = now_sec - init_time_sec_;

    if (now_sec > 0.0) {
        simulation_started_ = true;
    }

    if (simulation_started_)
    {
        if (elapsed_sec < finish_time_){
            // Первые 10 секунд движения
            if (elapsed_sec < 10.0){
                twist_msg.linear.x = 2.6;
                twist_msg.angular.z = 0.1;
            }
            else{
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
            }
            cmd_vel_pub_->publish(twist_msg);
            RCLCPP_INFO(this->get_logger(), "moving..!");
        } else{
            // Останавливаем робота по истечении времени
            cmd_vel_pub_->publish(twist_msg);
            finishControl();
        }
    }
}

void SimpleMover::finishControl()
{
    RCLCPP_INFO(this->get_logger(), "Controller stopped");
    timer_->cancel();
    rclcpp::shutdown();
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMover>(30.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}