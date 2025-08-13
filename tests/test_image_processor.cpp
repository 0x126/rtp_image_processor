#include <gtest/gtest.h>
#include "image_processor.h"
#include <atomic>
#include <chrono>
#include <thread>

// Test fixture for ImageProcessor
class ImageProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default configuration
        config_.udp_port = 5004;
        config_.jpeg_quality = 90;
        config_.enable_zero_copy = true;
    }
    
    void TearDown() override {
        // Cleanup if needed
    }
    
    ProcessorConfig config_;
};

// Test processor creation
TEST_F(ImageProcessorTest, CreateProcessor) {
    ImageProcessor processor(config_);
    // Should not crash
    SUCCEED();
}

// Test start and stop
TEST_F(ImageProcessorTest, StartStop) {
    ImageProcessor processor(config_);
    
    // Start should succeed
    EXPECT_TRUE(processor.start());
    
    // Give it some time to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Stop should work without issues
    processor.stop();
    SUCCEED();
}

// Test callback setting
TEST_F(ImageProcessorTest, SetCallback) {
    ImageProcessor processor(config_);
    
    std::atomic<bool> callback_called(false);
    
    processor.setFrameCallback([&callback_called](const uint8_t* data, 
                                                  size_t size, 
                                                  int64_t timestamp) {
        callback_called = true;
    });
    
    // Should not crash
    SUCCEED();
}

// Test statistics
TEST_F(ImageProcessorTest, GetStatistics) {
    ImageProcessor processor(config_);
    
    processor.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto stats = processor.getStatistics();
    
    // Initial statistics should be zero or non-negative
    EXPECT_GE(stats.frames_processed, 0);
    EXPECT_GE(stats.frames_dropped, 0);
    EXPECT_GE(stats.average_fps, 0.0);
    EXPECT_GE(stats.average_latency_ms, 0.0);
    
    processor.stop();
}

// Test multiple start/stop cycles
TEST_F(ImageProcessorTest, MultipleStartStop) {
    ImageProcessor processor(config_);
    
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(processor.start());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        processor.stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    SUCCEED();
}

// Test configuration variations
TEST_F(ImageProcessorTest, DifferentConfigs) {
    // Test with different port
    config_.udp_port = 5005;
    ImageProcessor processor1(config_);
    EXPECT_TRUE(processor1.start());
    processor1.stop();
    
    // Test with different quality
    config_.jpeg_quality = 50;
    ImageProcessor processor2(config_);
    EXPECT_TRUE(processor2.start());
    processor2.stop();
    
    // Test with zero-copy disabled
    config_.enable_zero_copy = false;
    ImageProcessor processor3(config_);
    EXPECT_TRUE(processor3.start());
    processor3.stop();
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}