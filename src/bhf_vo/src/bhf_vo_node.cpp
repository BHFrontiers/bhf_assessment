#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include "bhf_vo/msg/visual_custom.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node{
public:
    MinimalImagePublisher() : Node("bhf_vo_image_publisher"), count_(0), cap(0){

        bhf_vo_pub_ = this->create_publisher<bhf_vo::msg::VisualCustom>("bhf_vo_data", 10); // Initialize the publisher
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/camera/image_raw/compressed", 10, std::bind(&MinimalImagePublisher::imageCallback, this, std::placeholders::_1));

        // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("bhf_vo_image", 10);
        // timer_ = this->create_wall_timer(500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
        // cv::VideoCapture cap(0);
    }

private:

// void timer_callback() {

//     cv_bridge::CvImagePtr cv_ptr;

//     cv::Mat image(cv::Size(1280, 720), CV_8UC3);
//     cap >> image;
//     sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
//     publisher_->publish(*msg);
//     RCLCPP_INFO(this->get_logger(), "publishing");


void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){

    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);


if (!prev_image_.empty()){

                // Detect features in the previous and current image
                std::vector<cv::KeyPoint> keypoints_prev, keypoints_curr;
                cv::Mat descriptors_prev, descriptors_curr;

                // Use ORB for example
                cv::Ptr<cv::ORB> orb = cv::ORB::create();
                orb->detectAndCompute(prev_image_, cv::Mat(), keypoints_prev, descriptors_prev);
                orb->detectAndCompute(image, cv::Mat(), keypoints_curr, descriptors_curr);

                // Match features between the two images
                cv::BFMatcher matcher(cv::NORM_HAMMING);
                std::vector<cv::DMatch> matches;
                matcher.match(descriptors_prev, descriptors_curr, matches);

                // Extract matched points
                std::vector<cv::Point2f> points_prev, points_curr;
                for (const auto& match : matches)
                {
                    points_prev.push_back(keypoints_prev[match.queryIdx].pt);
                    points_curr.push_back(keypoints_curr[match.trainIdx].pt);
                }

                // Estimate the essential matrix
                cv::Mat essential_matrix = cv::findEssentialMat(points_prev, points_curr);

                // Recover the camera pose
                cv::Mat R, t;
                cv::recoverPose(essential_matrix, points_prev, points_curr, R, t);

                                // Prepare and publish the custom message
                bhf_vo::msg::VisualCustom bhf_vo_data_msg;
                bhf_vo_data_msg.camera_pose.position.x = t.at<double>(0);
                bhf_vo_data_msg.camera_pose.position.y = t.at<double>(1);
                bhf_vo_data_msg.camera_pose.position.z = t.at<double>(2);

                bhf_vo_data_msg.essential_matrix = {
                    essential_matrix.at<double>(0, 0), essential_matrix.at<double>(0, 1), essential_matrix.at<double>(0, 2),
                    essential_matrix.at<double>(1, 0), essential_matrix.at<double>(1, 1), essential_matrix.at<double>(1, 2),
                    essential_matrix.at<double>(2, 0), essential_matrix.at<double>(2, 1), essential_matrix.at<double>(2, 2)
                };
                bhf_vo_pub_->publish(bhf_vo_data_msg); // Publish the VO data

    
    }
    prev_image_ = image.clone();

} 
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<bhf_vo::msg::VisualCustom>::SharedPtr bhf_vo_pub_; // Publisher declaration
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
