# ROS2 RTP Image Processor Node

## Overview
ROS2 node for processing real-time video streams from UDP/RTP sources. Handles YCbCr 4:2:2 format video with hardware-accelerated RGB conversion and JPEG compression.

## Features
- Hardware-accelerated video processing (VA-API, NVIDIA, Jetson)
- Publishes both raw and compressed images
- Configurable via ROS2 parameters
- Real-time performance statistics

## Topics

### Published Topics
- `~/image_compressed` (sensor_msgs/CompressedImage): JPEG compressed images
- `~/image_raw` (sensor_msgs/Image): Raw BGR images (optional)

## Parameters
- `udp_port` (int, default: 5008): UDP port for RTP stream
- `jpeg_quality` (int, default: 90): JPEG compression quality (0-100)
- `buffer_size` (int, default: 8388608): Buffer size in bytes
- `max_buffers` (int, default: 3): Maximum number of buffers
- `publish_raw` (bool, default: false): Publish raw images
- `publish_compressed` (bool, default: true): Publish compressed images
- `frame_id` (string, default: "camera"): Frame ID for published images

## Build Instructions

### Prerequisites
```bash
# Install ROS2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport

# Install GStreamer dependencies (from CLAUDE.md)
sudo apt-get install \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-vaapi \
    gstreamer1.0-tools

# Install OpenCV
sudo apt install libopencv-dev
```

### Build
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package
cd /home/comlops/rtp_example
colcon build --packages-select rtp_image_processor

# Source the workspace
source install/setup.bash
```

## Usage

### Run the node
```bash
# Basic usage
ros2 run rtp_image_processor rtp_image_processor_node

# With parameters
ros2 run rtp_image_processor rtp_image_processor_node --ros-args \
    -p udp_port:=5008 \
    -p jpeg_quality:=95 \
    -p publish_raw:=true
```

### Using launch files
```bash
# Python launch file
ros2 launch rtp_image_processor rtp_image_processor.launch.py

# XML launch file
ros2 launch rtp_image_processor rtp_image_processor.launch.xml

# With arguments
ros2 launch rtp_image_processor rtp_image_processor.launch.py \
    udp_port:=5008 \
    publish_raw:=true \
    namespace:=my_camera
```

## Testing

### Send test RTP stream
```bash
# Generate test pattern and stream via RTP
gst-launch-1.0 videotestsrc ! \
    video/x-raw,width=2880,height=1860,format=UYVY,framerate=30/1 ! \
    rtpvrawpay ! udpsink host=127.0.0.1 port=5008
```

### View the output
```bash
# View compressed images
ros2 run image_view image_view --ros-args \
    -r image/compressed:=/rtp_processor/rtp_image_processor/image_compressed

# View raw images (if enabled)
ros2 run image_view image_view --ros-args \
    -r image:=/rtp_processor/rtp_image_processor/image_raw
```

### Monitor topics
```bash
# List topics
ros2 topic list

# Echo compressed image info
ros2 topic echo /rtp_processor/rtp_image_processor/image_compressed --no-arr

# Check publishing rate
ros2 topic hz /rtp_processor/rtp_image_processor/image_compressed
```

## Performance Tuning

### CPU Governor (x86)
```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### Jetson Optimization
```bash
sudo jetson_clocks
sudo nvpmodel -m 0
```

## Debugging

### Enable GStreamer debug output
```bash
export GST_DEBUG=3
ros2 run rtp_image_processor rtp_image_processor_node
```

### Check node info
```bash
ros2 node info /rtp_processor/rtp_image_processor
```