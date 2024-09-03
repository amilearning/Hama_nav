
# Unstructured Navigation Platform

## Overview

The Unstructured Navigation Platform is designed to facilitate autonomous navigation in unstructured environments.
It leverages ROS (Robot Operating System) on NVIDIA Jetson platforms to perform feature extraction, path planning, and control, allowing robots to navigate complex terrains.

## Table of Contents
- [Introduction](#introduction)
- [System Requirements](#system-requirements)
- [Getting Started](#getting-started)
   - [Clone the Repositories](#1-clone-the-repositories)
   - [Build the ROS Packages](#2-build-the-ros-packages)
   - [Run the Docker Container](#3-run-the-docker-container)
   - [Launch the ROS Nodes](#4-launch-the-ros-nodes)
- [Low-Level Control and Sensor Nodes](#low-level-control-and-sensor-nodes)
- [Training the Steago Model](#training-the-steago-model)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Contact](#contact)

# Introduction

This platform is built to enable autonomous navigation in environments that are typically challenging for robots to traverse. It includes feature extraction for understanding the environment, and path planning for safe and efficient navigation.

## System Requirements

- **Hardware:**
  - NVIDIA Jetson AGX Orin 
  - LiDAR sensor (Livox)
  - ZED Stereo Camera  

- **Software:**
  - Ubuntu 20.04 
  - Docker CE with NVIDIA runtime
  - ROS Noetic
  - Git

## Getting Started

### 1. Clone the Repositories into workspace 

```bash
git clone https://github.com/amilearning/unstruct_navigation.git -b orin2
git clone https://github.com/amilearning/unsup_seg_pkg.git -b orin
git clone https://github.com/amilearning/Hama_nav.git 
```
* Baseline Repository: Self-Supervised Segmentation " https://github.com/leggedrobotics/self_supervised_segmentation " 


### 2. Build the ROS Packages

After cloning the repositories, build the ROS workspace:

```bash
cd ~/workspace/
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 3. Run the Docker Container

Use the provided shell script `torch_docker2.sh` to start the Docker container:

```bash
chmod +x torch_docker2.sh
./torch_docker2.sh
```

Alternatively, you can run the Docker container manually (steogo_ws is the workspace directory in local env)
```bash
docker run --runtime nvidia -it --rm --network host \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /etc/enctune.conf:/etc/enctune.conf \
  --volume /etc/nv_tegra_release:/etc/nv_tegra_release \
  --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
  --volume /var/run/dbus:/var/run/dbus \
  --volume /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
  --volume /var/run/docker.sock:/var/run/docker.sock \
  --volume /home/hmcl/jetson-containers/data:/data \
  --device /dev/snd \
  --device /dev/bus/usb \
  --device /dev/video0 \
  --device /dev/video1 \
  --device /dev/i2c-0 --device /dev/i2c-1 --device /dev/i2c-2 \
  --device /dev/i2c-3 --device /dev/i2c-4 --device /dev/i2c-5 \
  --device /dev/i2c-6 --device /dev/i2c-7 --device /dev/i2c-8 \
  --device /dev/i2c-9 \
  -v /run/jtop.sock:/run/jtop.sock \
  -v /home/hmcl/steogo_ws:/home/stego_ws \
  amilearning/unstruct_nav_l4t:v1
```


### 4. Launch the ROS Nodes

1. **Start the Feature Extraction Node:**

   ```bash
   roslaunch unstruct_navigation_ros unstruct_visual_navigation.launch
   ```

2. **Start the Path Planning Node:**

   ```bash
   roslaunch unstruct_navigation_ros unstruct_planner.launch
   ```

## Low-Level Control and Sensor Nodes

The `Hama_nav` repository contains essential nodes for low-level control and additional sensor integration.

- **Repository URL:** [Hama_nav](https://github.com/amilearning/Hama_nav.git)

1. **Low-Level Control Initialization:**

   ```bash
   roslaunch lowevel_ctrl low_level.launch
   ```

   This command initializes the low-level controllers responsible for direct hardware interactions, such as motor control and sensor data processing.

2. **LiDAR, and state estimator initialization:**
check alias in bashrc.. 
    

3. **MAVROS Initialization:**

   ```bash
   rosrun mavros px4.launch 
   ```

   MAVROS is used to interface ROS with the robot's autopilot system, allowing for telemetry and control data exchange.

## PX4 Firmware

The PX4 firmware is designed for the Pixhawk flight controller and is essential for converting signals to control servo and motor PWM outputs. This firmware allows for precise control of actuators in robotic platforms, particularly for off-road and autonomous navigation applications.

- **Repository URL:** [offroad_px4autopilot](https://github.com/amilearning/offroad_px4autopilot.git)

This repository contains the custom PX4 firmware tailored for off-road autonomous vehicles. The firmware is adapted to manage the specific requirements of converting control signals to PWM outputs for servos and motors.

### Installation and Setup

1. **Clone the PX4 Firmware Repository:**

   ```bash
   git clone https://github.com/amilearning/offroad_px4autopilot.git ~/px4_firmware


### Training the Steago Model

To train the Steago model, navigate to the `unsup_seg_pkg` directory and run the training script:

## Troubleshooting
- **Docker Issues:** Ensure the NVIDIA runtime is correctly installed and that all volumes and devices are properly mapped.
- **ROS Nodes Not Communicating:** Verify that the ROS environment is correctly sourced and that all nodes are runnin (even inside of docker). 
