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
    MinimalImagePublisher() : Node("bhf_vo_image_publisher"){

        RCLCPP_INFO(this->get_logger(), "Initialized Visual Odometry Node");
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/camera/image_raw/compressed", 10, std::bind(&MinimalImagePublisher::imageCallback, this, std::placeholders::_1));
        bhf_vo_pub_ = this->create_publisher<bhf_vo::msg::VisualCustom>("bhf_vo_data", 10);

    }

private:

    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){

        try{

            // Reading the Image-Data frame.
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            
            if (image.empty()){
                RCLCPP_ERROR(this->get_logger(), "Image Failed to load ......");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Ros Bag Data is loaded .......");

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
                bhf_vo_pub_->publish(bhf_vo_data_msg);  
            }   
        
        prev_image_ = image.clone();
        }
        
        catch (const std::exception &e){
            RCLCPP_INFO(this->get_logger(), "ERROR: In Loading Image: %s", e.what());
        }
    }
// Variable Declaration 
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<bhf_vo::msg::VisualCustom>::SharedPtr bhf_vo_pub_; 
  cv::Mat prev_image_;

};
int main(int argc, char *argv[]){

    rclcpp::init(argc, argv);
    auto node =std::make_shared<MinimalImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
