#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class B4MCameraNode : public rclcpp::Node {
public:
    B4MCameraNode() : Node("b4m_camera_node") {
        // Create publisher for compressed images with b4m format
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "b4m/camera/image", 10);

        // Subscribe to Webots camera images
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&B4MCameraNode::image_callback, this, std::placeholders::_1));

        // Create timer for 20-second interval
        timer_ = this->create_wall_timer(
            std::chrono::seconds(20),
            std::bind(&B4MCameraNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "B4M Camera Node started - Publishing every 20 seconds");
    }

private:
    void timer_callback() {
        if (latest_image_) {
            compressed_pub_->publish(*latest_image_);
            RCLCPP_DEBUG(this->get_logger(), "Published B4M compressed image on timer");
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert raw image to OpenCV format
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Create compressed image message
            auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header = msg->header;
            compressed_msg->format = "jpg";

            // Convert OpenCV image to JPEG format
            std::vector<uchar> jpeg_buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95}; // JPEG quality 95%
            cv::imencode(".jpg", cv_ptr->image, jpeg_buffer, params);

            // Store the compressed image
            latest_image_ = std::make_unique<sensor_msgs::msg::CompressedImage>();
            latest_image_->header = msg->header;
            latest_image_->format = "jpg";
            latest_image_->data = jpeg_buffer;
            
            RCLCPP_DEBUG(this->get_logger(), "Received new image");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<sensor_msgs::msg::CompressedImage> latest_image_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<B4MCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
