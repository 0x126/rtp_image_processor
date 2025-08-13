#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include <memory>
#include <functional>
#include <cstdint>
#include <cstddef>

enum class HardwareType {
    VAAPI_INTEL,
    VAAPI_AMD,
    NVIDIA_DESKTOP,
    NVIDIA_JETSON,
    SOFTWARE_FALLBACK
};

struct ProcessorConfig {
    int udp_port = 5008;
    int jpeg_quality = 90;
    int buffer_size = 8388608;  // 8MB
    int max_buffers = 3;
};

class ImageProcessor {
public:
    struct RTPTimestamp {
        uint32_t rtp_timestamp;     // RTP header timestamp (32-bit)
        uint32_t ssrc;               // Synchronization source identifier
        uint32_t csrc;               // Contributing source identifier (contains nanoseconds)
        int64_t pts_ms;              // Presentation timestamp in milliseconds
        uint64_t seconds;            // Reconstructed seconds (48-bit in original)
        uint32_t nanoseconds;        // Nanoseconds portion
        uint16_t fractions;          // Fractions of nanoseconds
    };
    
    using FrameCallback = std::function<void(const uint8_t* data, size_t size, const RTPTimestamp& timestamp)>;
    
    ImageProcessor(const ProcessorConfig& config);
    ~ImageProcessor();
    
    // Disable copy
    ImageProcessor(const ImageProcessor&) = delete;
    ImageProcessor& operator=(const ImageProcessor&) = delete;
    
    // Start/stop processing
    bool start();
    void stop();
    
    // Set callback for processed frames
    void setFrameCallback(FrameCallback callback);
    
    // Statistics
    struct Statistics {
        uint64_t frames_processed;
        uint64_t frames_dropped;
        double average_fps;
        double average_latency_ms;
    };
    Statistics getStatistics() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // IMAGE_PROCESSOR_H