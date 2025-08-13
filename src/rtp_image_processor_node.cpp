#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_processor.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

class RtpImageProcessorNode : public rclcpp::Node {
public:
    RtpImageProcessorNode() : Node("rtp_image_processor_node") {
        this->declare_parameter("udp_port", 5008);
        this->declare_parameter("jpeg_quality", 90);
        this->declare_parameter("buffer_size", 8388608);
        this->declare_parameter("max_buffers", 3);
        this->declare_parameter("publish_raw", false);
        this->declare_parameter("publish_compressed", true);
        this->declare_parameter("frame_id", "camera");
        
        int udp_port = this->get_parameter("udp_port").as_int();
        int jpeg_quality = this->get_parameter("jpeg_quality").as_int();
        int buffer_size = this->get_parameter("buffer_size").as_int();
        int max_buffers = this->get_parameter("max_buffers").as_int();
        publish_raw_ = this->get_parameter("publish_raw").as_bool();
        publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        ProcessorConfig config;
        config.udp_port = udp_port;
        config.jpeg_quality = jpeg_quality;
        config.buffer_size = buffer_size;
        config.max_buffers = max_buffers;
        
        processor_ = std::make_unique<ImageProcessor>(config);
        
        if (publish_compressed_) {
            compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "~/image_raw/compressed", 10);
        }
        
        if (publish_raw_) {
            image_transport::ImageTransport it(shared_from_this());
            raw_pub_ = it.advertise("~/image_raw", 10);
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
        
        frame_count_++;
    }
    
    void publishStatistics() {
        auto stats = processor_->getStatistics();
        RCLCPP_INFO(this->get_logger(), 
                   "Stats: Frames=%lu, Dropped=%lu, FPS=%.2f, Latency=%.2fms",
                   stats.frames_processed, stats.frames_dropped,
                   stats.average_fps, stats.average_latency_ms);
    }
    
    std::unique_ptr<ImageProcessor> processor_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    image_transport::Publisher raw_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    bool publish_raw_;
    bool publish_compressed_;
    std::string frame_id_;
    uint64_t frame_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RtpImageProcessorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}