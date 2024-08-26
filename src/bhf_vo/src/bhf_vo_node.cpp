#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node{
public:
    MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0), cap(0){

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
        cv::VideoCapture cap(0);
    }

private:

void timer_callback() {

    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat image(cv::Size(1280, 720), CV_8UC3);
    cap >> image;
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "publishing");

} 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  cv::VideoCapture cap;
  cv::Mat prev_image_;

};
int main(int argc, char *argv[]){

    printf("Starting.......");
    rclcpp::init(argc, argv);
    auto node =std::make_shared<MinimalImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
