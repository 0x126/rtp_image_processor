#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_processor.h"
#include <opencv2/opencv.hpp>
#include <chrono>

namespace rtp_image_processor {

class RtpImageProcessorNode : public rclcpp::Node {
public:
    explicit RtpImageProcessorNode(const rclcpp::NodeOptions& options) 
        : Node("rtp_image_processor_node", options) {
        this->declare_parameter("udp_port", 5008);
        this->declare_parameter("jpeg_quality", 90);
        this->declare_parameter("buffer_size", 8388608);
        this->declare_parameter("max_buffers", 3);
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1280);
        this->declare_parameter("publish_raw", false);
        this->declare_parameter("publish_compressed", true);
        this->declare_parameter("frame_id", "camera");
        this->declare_parameter("camera_info_url", "");
        
        int udp_port = this->get_parameter("udp_port").as_int();
        int jpeg_quality = this->get_parameter("jpeg_quality").as_int();
        int buffer_size = this->get_parameter("buffer_size").as_int();
        int max_buffers = this->get_parameter("max_buffers").as_int();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        publish_raw_ = this->get_parameter("publish_raw").as_bool();
        publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string camera_info_url = this->get_parameter("camera_info_url").as_string();
        
        ProcessorConfig config;
        config.udp_port = udp_port;
        config.jpeg_quality = jpeg_quality;
        config.buffer_size = buffer_size;
        config.max_buffers = max_buffers;
        config.width = width;
        config.height = height;
        
        processor_ = std::make_unique<ImageProcessor>(config);
        
        if (publish_compressed_) {
            compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "~/image_raw/compressed", 10);
        }
        
        if (publish_raw_) {
            image_transport::ImageTransport it(shared_from_this());
            raw_pub_ = it.advertise("~/image_raw", 10);
        }
        
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/camera_info", 10);
        
        // Initialize camera info manager
        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, frame_id_);
        if (!camera_info_url.empty()) {
            camera_info_manager_->loadCameraInfo(camera_info_url);
            RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No camera_info_url provided, using default camera info");
        }
        
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RtpImageProcessorNode::publishStatistics, this));
        
        processor_->setFrameCallback(
            std::bind(&RtpImageProcessorNode::frameCallback, this,
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        if (!processor_->start()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start image processor");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "RTP Image Processor Node started on UDP port %d", udp_port);
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", width, height);
        RCLCPP_INFO(this->get_logger(), "Publishing: raw=%s, compressed=%s", 
                    publish_raw_ ? "true" : "false",
                    publish_compressed_ ? "true" : "false");
    }
    
    ~RtpImageProcessorNode() {
        if (processor_) {
            processor_->stop();
        }
    }
    
private:
    void frameCallback(const uint8_t* data, size_t size, const ImageProcessor::RTPTimestamp& timestamp) {
        auto now = this->now();
        
        if (publish_compressed_ && compressed_pub_->get_subscription_count() > 0) {
            auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header.stamp = now;
            compressed_msg->header.frame_id = frame_id_;
            
            // Add RTP timestamp info to header
            // Use the extracted timestamp if available
            if (timestamp.seconds > 0) {
                compressed_msg->header.stamp.sec = timestamp.seconds;
                compressed_msg->header.stamp.nanosec = timestamp.nanoseconds;
            }
            compressed_msg->format = "jpeg";
            compressed_msg->data.assign(data, data + size);
            
            compressed_pub_->publish(std::move(compressed_msg));
        }
        
        if (publish_raw_ && raw_pub_.getNumSubscribers() > 0) {
            std::vector<uint8_t> jpeg_data(data, data + size);
            cv::Mat compressed(1, jpeg_data.size(), CV_8UC1, jpeg_data.data());
            cv::Mat image = cv::imdecode(compressed, cv::IMREAD_COLOR);
            
            if (!image.empty()) {
                sensor_msgs::msg::Image::SharedPtr image_msg = 
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                image_msg->header.stamp = now;
                image_msg->header.frame_id = frame_id_;
                
                // Add RTP timestamp info to header
                if (timestamp.seconds > 0) {
                    image_msg->header.stamp.sec = timestamp.seconds;
                    image_msg->header.stamp.nanosec = timestamp.nanoseconds;
                }
                raw_pub_.publish(image_msg);
            }
        }
        
        if (camera_info_pub_->get_subscription_count() > 0) {
            auto camera_info_msg = camera_info_manager_->getCameraInfo();
            camera_info_msg.header.stamp = now;
            camera_info_msg.header.frame_id = frame_id_;
            
            // Add RTP timestamp info to header if available
            if (timestamp.seconds > 0) {
                camera_info_msg.header.stamp.sec = timestamp.seconds;
                camera_info_msg.header.stamp.nanosec = timestamp.nanoseconds;
            }
            
            camera_info_pub_->publish(camera_info_msg);
        }
        
        frame_count_++;
    }
    
    void publishStatistics() {
        auto stats = processor_->getStatistics();
        RCLCPP_INFO(this->get_logger(), 
                   "Stats: Frames=%lu, Dropped=%lu, FPS=%.2f",
                   stats.frames_processed, stats.frames_dropped,
                   stats.average_fps);
    }
    
    std::unique_ptr<ImageProcessor> processor_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    image_transport::Publisher raw_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    
    bool publish_raw_;
    bool publish_compressed_;
    std::string frame_id_;
    uint64_t frame_count_ = 0;
};

}  // namespace rtp_image_processor

RCLCPP_COMPONENTS_REGISTER_NODE(rtp_image_processor::RtpImageProcessorNode)