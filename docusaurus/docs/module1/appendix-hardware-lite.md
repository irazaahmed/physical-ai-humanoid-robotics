---
doc_id: project1_m1_ch10
title: "Appendix: Lightweight Hardware Recommendations"
module: module1
estimated_tokens: 700
embedding_required: true
chunk_hint_tokens: 500
---

<!-- original_metadata: {"doc_id": "project1_m1_ch10", "title": "Appendix: Lightweight Hardware Recommendations", "module": "module1", "estimated_tokens": 700, "embedding_required": true, "chunk_hint_tokens": 500} -->
<!-- embedding_placeholder: true -->

<!-- changelog: auto-fixed MDX parse error, converted frontmatter to YAML format -->

# Appendix: Lightweight Hardware Recommendations

## Objective

Provide recommendations for lightweight, accessible hardware platforms suitable for learning ROS 2 concepts, from simulation to physical implementation.

## Overview

This appendix outlines hardware platforms appropriate for learning ROS 2 concepts without requiring extensive resources. These recommendations focus on platforms that are affordable, well-documented, and have strong ROS 2 support.

## Simulation-First Approach

Before investing in physical hardware, it's recommended to start with simulation environments where you can learn ROS 2 concepts without the complexities of physical robot limitations:

- **Gazebo Harmonic**: Full-featured physics simulation environment with ROS 2 integration
- **Ignition Gazebo**: Next-generation simulation framework with improved performance
- **Webots**: Open-source robot simulator with extensive robot models
- **Mujoco**: Commercial physics simulator with excellent ROS 2 support (license required)

## Entry-Level Educational Robots

### TurtleBot 4
- **Platform**: ROS 2 native robot designed for education
- **Features**: Complete mobile robot with differential drive, IMU, camera, and LiDAR support
- **ROS 2 Support**: Full compatibility with ROS 2 Humble Hawksbill
- **Price Range**: $1,500-$3,000 depending on configuration
- **Strengths**: Well-documented, educational resources, community support

### Create® 3 Educational Robot
- **Platform**: Mobile robot platform from iRobot
- **Features**: Differential drive, multiple sensors, RGB camera
- **ROS 2 Support**: Native ROS 2 support with tutorials
- **Price Range**: $1,500
- **Strengths**: Real-world robot, educational curriculum available

## DIY Robot Platforms

### ROSbot
- **Platform**: Open-source mobile robot platform
- **Features**: Differential drive, 3D camera, LiDAR compatibility
- **ROS 2 Support**: Full ROS 2 compatibility with documentation
- **Price Range**: $600-$1,200
- **Strengths**: Open hardware, modular design, educational materials

### Donkey Car
- **Platform**: Autonomous RC car platform
- **Features**: Camera, IMU, motor control via Raspberry Pi
- **ROS 2 Support**: ROS 2 bridge available
- **Price Range**: $200-$400
- **Strengths**: Affordable, large community, good for computer vision

### TB3 Burger Waffle
- **Platform**: TurtleBot 3 variants
- **Features**: Compact mobile robot with 3D camera and LiDAR
- **ROS 2 Support**: Excellent ROS 2 support with tutorials
- **Price Range**: $700-$1,000
- **Strengths**: Compact, educational, wide range of tutorials

## Single Board Computers

### Raspberry Pi 4
- **Processor**: Quad-core ARM Cortex-A72 (1.5GHz)
- **RAM**: 2GB, 4GB, or 8GB options
- **ROS 2 Support**: Full ROS 2 support (Humble Hawksbill)
- **Price Range**: $75-$200
- **Use Case**: ROS 2 nodes, sensor interfaces, lightweight processing

### NVIDIA Jetson Nano
- **Processor**: Quad-core ARM Cortex-A57 (1.43GHz)
- **GPU**: 128-core Maxwell GPU
- **RAM**: 4GB
- **ROS 2 Support**: Full ROS 2 support with GPU acceleration
- **Price Range**: $99-$150
- **Use Case**: Computer vision, AI inference, sensor processing

### Jetson Orin Nano
- **Processor**: ARM Cortex-A78AE (2.2 GHz)
- **GPU**: 1024-core NVIDIA Ampere GPU
- **RAM**: 4GB or 8GB
- **ROS 2 Support**: ROS 2 support with significant acceleration capabilities
- **Price Range**: $200-$250
- **Use Case**: Advanced AI, complex sensor processing, real-time applications

## Sensors for Learning

### Camera Options
- **Intel RealSense D435**: RGB-D camera with ROS 2 driver
- **Logitech C920**: USB camera with good ROS 2 support
- **Raspberry Pi Camera Module**: Direct interface with Pi, ROS 2 support

### LiDAR Sensors
- **SLAMTEC RPLidar A1/A2**: 360° laser scanner with ROS 2 support
- **YDLIDAR X4**: Budget-friendly LiDAR with ROS 2 compatibility
- **SICK TIM551**: Industrial-grade LiDAR with ROS 2 drivers

### IMU Sensors
- **Adafruit BNO055**: 9-DOF sensor with orientation support
- **SparkFun 9DoF IMU**: ROS 2 compatible with sensor fusion

## Development Workstations

### Laptop Requirements
- **CPU**: Multi-core processor (Intel i5 or equivalent)
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: SSD recommended for better build performance
- **OS**: Ubuntu 22.04 LTS for native ROS 2 Humble development

### Desktop Considerations
- **CPU**: Multi-core processor for faster builds
- **RAM**: 16GB minimum for simulation environments
- **GPU**: Dedicated GPU helpful for Gazebo simulation and computer vision

## Budget Considerations

### Starter Budget ($200-$500)
- Raspberry Pi 4 with camera
- Basic mobile robot platform (like Donkey Car)
- Essential sensors (camera, basic IMU)

### Intermediate Budget ($500-$1,500)
- TB3 Burger/Waffle or ROSbot
- Additional sensors (LiDAR, more advanced camera)
- Better SBC (NVIDIA Jetson Nano)

### Advanced Budget ($1,500-$3,000)
- TurtleBot 4 or Create 3
- Multiple sensors and actuators
- High-performance SBC (NVIDIA Jetson Orin)
- Additional simulation licenses if needed

## Key Considerations

1. **Start with simulation**: Learn ROS 2 concepts in simulation before moving to hardware
2. **Choose active platforms**: Select platforms with active ROS 2 support and community
3. **Consider expandability**: Choose platforms that allow adding sensors and capabilities
4. **Factor in ongoing costs**: Include costs for power, maintenance, and potential replacements
5. **Check compatibility**: Ensure all components have ROS 2 support for your target distribution

## Conclusion

Hardware selection should align with learning objectives and budget constraints. For beginners, starting with simulation is highly recommended, followed by entry-level educational robots. The platforms listed here offer good ROS 2 support, educational resources, and community backing that will facilitate learning of ROS 2 concepts.