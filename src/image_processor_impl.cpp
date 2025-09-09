#include "image_processor.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <thread>
#include <mutex>
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
    GstElement* identity_ = nullptr;
    FrameCallback frame_callback_;
    std::atomic<bool> running_{false};
    
    // RTP timestamp tracking
    std::mutex rtp_mutex_;
    ImageProcessor::RTPTimestamp last_rtp_timestamp_;
    
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
        if (identity_) {
            gst_object_unref(identity_);
        }
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
        
        // Setup RTP probe
        setupRTPProbe();
        
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
            // " address=10.42.0.1" + 
            " caps=\"application/x-rtp, sampling=(string)YCbCr-4:2:2, depth=(string)8, "
            "width=(string)" + std::to_string(config_.width) + ", height=(string)" + std::to_string(config_.height) + "\" ! "
            "identity name=rtpidentity ! "
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
    //         "video/x-raw,format=UYVY,width=" + std::to_string(config_.width) + ",height=" + std::to_string(config_.height) + " !"
    //         "vaapipostproc !"
    //         "video/x-raw,format=NV12,width=" + std::to_string(config_.width) + ",height=" + std::to_string(config_.height) + " !"
    //         "vaapijpegenc quality=" + std::to_string(config_.jpeg_quality) + " !";
    // }

    std::string buildJetsonPipeline() {
        // Jetson pipeline - always use NVMM memory for best performance
        return "nvvidconv ! "
               "video/x-raw(memory:NVMM),format=I420 ! "
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
            // Extract RTP timestamp information
            ImageProcessor::RTPTimestamp rtp_ts = impl->extractRTPTimestamp(buffer);
            
            // Call callback with RTP timestamp
            if (impl->frame_callback_) {
                impl->frame_callback_(map.data, map.size, rtp_ts);
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
    
    void setupRTPProbe() {
        // Get the identity element from the pipeline
        identity_ = gst_bin_get_by_name(GST_BIN(pipeline_), "rtpidentity");
        if (identity_) {
            GstPad* srcpad = gst_element_get_static_pad(identity_, "src");
            if (srcpad) {
                // Add probe to capture RTP buffers
                gst_pad_add_probe(srcpad, GST_PAD_PROBE_TYPE_BUFFER,
                                 onRTPBuffer, this, nullptr);
                gst_object_unref(srcpad);
            }
        }
    }
    
    static GstPadProbeReturn onRTPBuffer(GstPad* pad, GstPadProbeInfo* info, gpointer user_data) {
        auto* impl = static_cast<Impl*>(user_data);
        GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        
        if (buffer) {
            ImageProcessor::RTPTimestamp rtp_ts = {};
            
            // Try to extract RTP header information
            GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
            if (gst_rtp_buffer_map(buffer, GST_MAP_READ, &rtp)) {
                rtp_ts.rtp_timestamp = gst_rtp_buffer_get_timestamp(&rtp);
                rtp_ts.ssrc = gst_rtp_buffer_get_ssrc(&rtp);
                
                // Get CSRC if available
                guint8 csrc_count = gst_rtp_buffer_get_csrc_count(&rtp);
                if (csrc_count > 0) {
                    rtp_ts.csrc = gst_rtp_buffer_get_csrc(&rtp, 0);
                }
                
                // Reconstruct 96-bit custom timestamp according to Tier4 documentation:
                // - RTP timestamp (32 bits): High 32 bits of seconds
                // - SSRC (32 bits): Low 16 bits of seconds + High 16 bits of nanoseconds
                // - CSRC (32 bits): Low 16 bits of nanoseconds + 16 bits fractions
                
                if (csrc_count > 0) {
                    // Full custom timestamp is available (96-bit total)
                    // According to Tier4 documentation:
                    // - Bits [95:48] (48 bits): Seconds
                    // - Bits [47:16] (32 bits): Nanoseconds  
                    // - Bits [15:0] (16 bits): Fractions of nanoseconds
                    //
                    // RTP Header mapping:
                    // - RTP timestamp (32 bits): Seconds bits [47:16]
                    // - SSRC (32 bits): 
                    //   - Upper 16 bits: Seconds bits [15:0]
                    //   - Lower 16 bits: Nanoseconds bits [31:16]
                    // - CSRC (32 bits):
                    //   - Upper 16 bits: Nanoseconds bits [15:0]
                    //   - Lower 16 bits: Fractions bits [15:0]
                    
                    // Seconds: 48 bits total
                    // RTP timestamp contains seconds[47:16] (upper 32 bits of 48-bit seconds)
                    // SSRC upper 16 bits contains seconds[15:0] (lower 16 bits of 48-bit seconds)
                    uint64_t seconds_high_32 = static_cast<uint64_t>(rtp_ts.rtp_timestamp);
                    uint64_t seconds_low_16 = static_cast<uint64_t>((rtp_ts.ssrc >> 16) & 0xFFFF);
                    rtp_ts.seconds = (seconds_high_32 << 16) | seconds_low_16;
                    
                    // Nanoseconds: 32 bits total
                    // SSRC lower 16 bits contains nanoseconds[31:16] (upper 16 bits)
                    // CSRC upper 16 bits contains nanoseconds[15:0] (lower 16 bits)
                    uint32_t nanos_high_16 = (rtp_ts.ssrc & 0xFFFF);
                    uint32_t nanos_low_16 = ((rtp_ts.csrc >> 16) & 0xFFFF);
                    rtp_ts.nanoseconds = (nanos_high_16 << 16) | nanos_low_16;
                    
                    // Fractions: 16 bits from CSRC lower 16 bits
                    rtp_ts.fractions = (rtp_ts.csrc & 0xFFFF);
                } else {
                    // Fallback to system time if custom timestamp not available
                    auto now = std::chrono::system_clock::now();
                    auto duration = now.time_since_epoch();
                    auto total_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
                    
                    rtp_ts.seconds = total_nanos / 1000000000;
                    rtp_ts.nanoseconds = total_nanos % 1000000000;
                    rtp_ts.fractions = 0;
                }
                
                gst_rtp_buffer_unmap(&rtp);
                
                // Store the RTP timestamp
                std::lock_guard<std::mutex> lock(impl->rtp_mutex_);
                impl->last_rtp_timestamp_ = rtp_ts;
            }
        }
        
        return GST_PAD_PROBE_OK;
    }
    
    ImageProcessor::RTPTimestamp extractRTPTimestamp(GstBuffer* buffer) {
        ImageProcessor::RTPTimestamp rtp_ts;
        
        // Get the RTP timestamp calculated from RTP headers
        {
            std::lock_guard<std::mutex> lock(rtp_mutex_);
            rtp_ts = last_rtp_timestamp_;
        }
        
        // Update PTS from current buffer
        GstClockTime timestamp = GST_BUFFER_PTS(buffer);
        rtp_ts.pts_ms = timestamp != GST_CLOCK_TIME_NONE ? 
                        timestamp / 1000000 : 0; // nanoseconds to milliseconds
        
        // If no RTP custom timestamp available (no CSRC), use system time as fallback
        if (rtp_ts.seconds == 0 && rtp_ts.nanoseconds == 0) {
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto total_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
            
            rtp_ts.seconds = total_nanos / 1000000000;
            rtp_ts.nanoseconds = total_nanos % 1000000000;
        }
        
        return rtp_ts;
    }
    
    void extractRTPHeaderInfo(GstBuffer* rtp_buffer, ImageProcessor::RTPTimestamp& rtp_ts) {
        GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
        
        if (gst_rtp_buffer_map(rtp_buffer, GST_MAP_READ, &rtp)) {
            // Extract RTP header fields
            rtp_ts.rtp_timestamp = gst_rtp_buffer_get_timestamp(&rtp);
            rtp_ts.ssrc = gst_rtp_buffer_get_ssrc(&rtp);
            
            // Get CSRC if available
            guint8 csrc_count = gst_rtp_buffer_get_csrc_count(&rtp);
            if (csrc_count > 0) {
                rtp_ts.csrc = gst_rtp_buffer_get_csrc(&rtp, 0);
                
                // Reconstruct full 96-bit timestamp from RTP custom timestamp format
                // According to Tier4 documentation:
                // - RTP timestamp (32 bits): Seconds bits [47:16]
                // - SSRC upper 16 bits: Seconds bits [15:0]
                // - SSRC lower 16 bits: Nanoseconds bits [31:16]
                // - CSRC upper 16 bits: Nanoseconds bits [15:0]
                // - CSRC lower 16 bits: Fractions bits [15:0]
                
                // Seconds: 48 bits total
                uint64_t seconds_high_32 = static_cast<uint64_t>(rtp_ts.rtp_timestamp);
                uint64_t seconds_low_16 = static_cast<uint64_t>((rtp_ts.ssrc >> 16) & 0xFFFF);
                rtp_ts.seconds = (seconds_high_32 << 16) | seconds_low_16;
                
                // Nanoseconds: 32 bits total
                uint32_t nanos_high_16 = (rtp_ts.ssrc & 0xFFFF);
                uint32_t nanos_low_16 = ((rtp_ts.csrc >> 16) & 0xFFFF);
                rtp_ts.nanoseconds = (nanos_high_16 << 16) | nanos_low_16;
                
                // Fractions: 16 bits
                rtp_ts.fractions = (rtp_ts.csrc & 0xFFFF);
            }
            
            gst_rtp_buffer_unmap(&rtp);
        }
    }
};