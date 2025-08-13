#include "image_processor.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>

class ImageProcessor::Impl {
private:
    ProcessorConfig config_;
    HardwareType hw_type_;
    GstElement* pipeline_ = nullptr;
    GstElement* appsink_ = nullptr;
    FrameCallback frame_callback_;
    std::atomic<bool> running_{false};
    
    // Statistics
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    std::chrono::steady_clock::time_point start_time_;
    
public:
    Impl(const ProcessorConfig& config) : config_(config) {
        gst_init(nullptr, nullptr);
        hw_type_ = detectHardware();
        std::cout << "Detected hardware type: " << static_cast<int>(hw_type_) << std::endl;
    }
    
    ~Impl() {
        stop();
        if (pipeline_) {
            gst_object_unref(pipeline_);
        }
    }
    
    bool start() {
        if (running_) return false;
        
        // Build pipeline
        std::string pipeline_str = buildPipelineString();
        std::cout << "Pipeline string: " << pipeline_str << std::endl;
        
        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        
        if (error) {
            std::cerr << "Pipeline creation error: " << error->message << std::endl;
            g_error_free(error);
            return false;
        }
        
        // Setup AppSink
        setupAppSink();
        
        // Start
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        running_ = true;
        start_time_ = std::chrono::steady_clock::now();
        
        return true;
    }
    
    void stop() {
        if (!running_) return;
        
        running_ = false;
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
        }
    }
    
    void setFrameCallback(FrameCallback callback) {
        frame_callback_ = callback;
    }
    
    Statistics getStatistics() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
        
        Statistics stats;
        stats.frames_processed = frames_processed_;
        stats.frames_dropped = frames_dropped_;
        stats.average_fps = duration > 0 ? static_cast<double>(frames_processed_) / duration : 0;
        stats.average_latency_ms = 0; // TODO: Implement latency measurement
        
        return stats;
    }
    
private:
    HardwareType detectHardware() {
        // Platform detection logic
        #ifdef JETSON_PLATFORM
            return HardwareType::NVIDIA_JETSON;
        #else
            // Check VA-API availability
            if (checkVAAPI()) {
                std::string vendor = getGPUVendor();
                if (vendor.find("Intel") != std::string::npos) {
                    return HardwareType::VAAPI_INTEL;
                } else if (vendor.find("AMD") != std::string::npos) {
                    return HardwareType::VAAPI_AMD;
                }
            }
            
            // Check NVIDIA Desktop GPU
            if (checkNVIDIADesktop()) {
                return HardwareType::NVIDIA_DESKTOP;
            }
            
            return HardwareType::SOFTWARE_FALLBACK;
        #endif
    }
    
    std::string buildPipelineString() {
        std::string base = 
            "udpsrc port=" + std::to_string(config_.udp_port) + 
            " buffer-size=" + std::to_string(config_.buffer_size) + 
            " caps=\"application/x-rtp, sampling=(string)YCbCr-4:2:2, depth=(string)8, "
            "width=(string)1920, height=(string)1280\" ! "
            "rtpvrawdepay ! ";
        
        std::string processing;
        
        switch (hw_type_) {
            case HardwareType::VAAPI_INTEL:
            case HardwareType::VAAPI_AMD:
                processing = buildVAAPIPipeline();
                break;
                
            case HardwareType::NVIDIA_DESKTOP:
                processing = buildNVIDIADesktopPipeline();
                break;
                
            case HardwareType::NVIDIA_JETSON:
                processing = buildJetsonPipeline();
                break;
                
            default:
                processing = buildSoftwarePipeline();
                break;
        }
        
        return base + processing + " appsink name=sink";
    }
    
    std::string buildVAAPIPipeline() {
        return "videoconvert ! "
               "video/x-raw,format=NV12 !"
               "vaapijpegenc quality=" + std::to_string(config_.jpeg_quality) + " !";
    }
    // std::string buildVAAPIPipeline() {
    //     return
    //         "video/x-raw,format=UYVY,width=1920,height=1280 !"
    //         "vaapipostproc !"
    //         "video/x-raw,format=NV12,width=1920,height=1280 !"
    //         "vaapijpegenc quality=" + std::to_string(config_.jpeg_quality) + " !";
    // }

    std::string buildJetsonPipeline() {
        // Jetson pipeline - always use NVMM memory for best performance
        return "nvvidconv ! "
               "video/x-raw(memory:NVMM),format=RGBA ! "
               "nvjpegenc quality=" + std::to_string(config_.jpeg_quality) + 
               " idct-method=ifast !";
    }
    
    std::string buildNVIDIADesktopPipeline() {
        // Desktop NVIDIA GPU
        return "videoconvert ! "  // TODO: Add nvvidconv support
               "nvjpegenc quality=" + std::to_string(config_.jpeg_quality) + " !";
    }
    
    std::string buildSoftwarePipeline() {
        return "videoconvert ! "
               "video/x-raw,format=RGB ! "
               "jpegenc quality=" + std::to_string(config_.jpeg_quality) + " !"; 
    }
    
    void setupAppSink() {
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        
        g_object_set(appsink_,
            "emit-signals", TRUE,
            "sync", FALSE,
            "max-buffers", config_.max_buffers,
            "drop", TRUE,
            nullptr);
        
        // Set callback
        g_signal_connect(appsink_, "new-sample", 
                        G_CALLBACK(onNewSample), this);
    }
    
    static GstFlowReturn onNewSample(GstAppSink* sink, gpointer user_data) {
        auto* impl = static_cast<Impl*>(user_data);
        
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;
        
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // Get timestamp
            GstClockTime timestamp = GST_BUFFER_PTS(buffer);
            int64_t pts = timestamp != GST_CLOCK_TIME_NONE ? 
                         timestamp / 1000000 : 0; // nanoseconds to milliseconds
            
            // Call callback
            if (impl->frame_callback_) {
                impl->frame_callback_(map.data, map.size, pts);
            }
            
            impl->frames_processed_++;
            gst_buffer_unmap(buffer, &map);
        }
        
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    bool checkVAAPI() {
        // Check VA-API availability
        GstElement* test = gst_element_factory_make("vaapipostproc", nullptr);
        if (test) {
            gst_object_unref(test);
            std::cout << "VA-API is available" << std::endl;
            return true;
        }
        std::cout << "VA-API is NOT available" << std::endl;
        return false;
    }
    
    bool checkNVIDIADesktop() {
        // Check NVIDIA GPU existence
        #ifndef JETSON_PLATFORM
            // First check for nvjpegenc
            GstElement* test = gst_element_factory_make("nvjpegenc", nullptr);
            if (test) {
                gst_object_unref(test);
                return true;
            }
            // Also check for nvvidconv as indicator of NVIDIA support
            test = gst_element_factory_make("nvvidconv", nullptr);
            if (test) {
                gst_object_unref(test);
                // NVIDIA support exists but nvjpegenc might not be available
                // Fall back to software in this case
                return false;
            }
        #endif
        return false;
    }
    
    std::string getGPUVendor() {
        // GPU vendor detection
        // Try multiple possible locations for Intel GPU
        std::vector<std::string> paths = {
            "/sys/class/drm/card0/device/vendor",
            "/sys/class/drm/card1/device/vendor",
            "/sys/class/drm/renderD128/device/vendor"
        };
        
        for (const auto& path : paths) {
            std::ifstream file(path);
            if (file.is_open()) {
                std::string vendor_id;
                file >> vendor_id;
                file.close();
                
                std::cout << "Found GPU vendor ID: " << vendor_id << " at " << path << std::endl;
                
                if (vendor_id == "0x8086") return "Intel";
                if (vendor_id == "0x1002") return "AMD"; 
                if (vendor_id == "0x10de") return "NVIDIA";
            }
        }
        
        // Fallback: check if Intel GPU via lspci
        FILE* pipe = popen("lspci | grep -i 'vga\\|3d' | grep -i intel", "r");
        if (pipe) {
            char buffer[128];
            if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                pclose(pipe);
                std::cout << "Intel GPU detected via lspci" << std::endl;
                return "Intel";
            }
            pclose(pipe);
        }
        
        return "Unknown";
    }
};