#include "image_processor.h"
#include <iostream>
#include <fstream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

std::atomic<bool> running(true);

void signalHandler(int signal) {
    running = false;
}

int main(int argc, char* argv[]) {
    // Set signal handler
    std::signal(SIGINT, signalHandler);
    
    // Configuration
    ProcessorConfig config;
    config.udp_port = 5008;
    config.jpeg_quality = 90;
    
    // Create processor
    ImageProcessor processor(config);
    
    // Frame counter
    std::atomic<uint64_t> frame_count(0);
    
    // Set callback
    processor.setFrameCallback([&frame_count](const uint8_t* data, 
                                              size_t size, 
                                              int64_t timestamp) {
        uint64_t count = ++frame_count;
        
        // Display progress and save frame every 100 frames
        if (count % 100 == 0) {
            std::cout << "Processed " << count << " frames, "
                     << "JPEG size: " << size << " bytes, "
                     << "timestamp: " << timestamp << " ms" << std::endl;
            
            // Save sample frame
            std::string filename = "frame_" + std::to_string(count) + ".jpg";
            std::ofstream file(filename, std::ios::binary);
            file.write(reinterpret_cast<const char*>(data), size);
            std::cout << "Saved sample: " << filename << std::endl;
        }
    });
    
    // Start processing
    if (!processor.start()) {
        std::cerr << "Failed to start processor" << std::endl;
        return 1;
    }
    
    std::cout << "Processing started. Press Ctrl+C to stop." << std::endl;
    
    // Main loop
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Display statistics
        auto stats = processor.getStatistics();
        std::cout << "FPS: " << stats.average_fps 
                 << ", Dropped: " << stats.frames_dropped << std::endl;
    }
    
    // Stop processing
    processor.stop();
    std::cout << "Processing stopped." << std::endl;
    
    return 0;
}