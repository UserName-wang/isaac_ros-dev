#!/bin/bash

# Proxy settings (adjust as needed)
export http_proxy=http://192.168.0.57:12346
export https_proxy=http://192.168.0.57:12346
export HTTP_PROXY=http://192.168.0.57:12346
export HTTPS_PROXY=http://192.168.0.57:12346

# Create a directory for downloaded packages
mkdir -p ~/isaac_ros_debs
cd ~/isaac_ros_debs

# Base URL for the packages
BASE_URL="https://isaac.download.nvidia.cn/isaac-ros/release-3/pool/release-3.2"

echo "Downloading Isaac ROS packages through proxy..."

# Download packages with proxy settings
# PeopleSemSeg models
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-peoplesemseg-models-install-arm64.deb

# PeopleNet models
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-peoplenet-models-install-arm64.deb

# Visual SLAM packages
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-visual-slam-interfaces-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-visual-slam-arm64.deb

# UNet packages
wget -c --proxy=on ${BASE_URL}/ros-humble-gxf-isaac-ros-unet-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-unet-arm64.deb

# TensorRT and related packages
wget -c --proxy=on ${BASE_URL}/ros-humble-gxf-isaac-tensor-rt-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-tensor-rt-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-tensor-proc-arm64.deb

# DNN Image Encoder
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-dnn-image-encoder-arm64.deb

# DetectNet packages
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-detectnet-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-gxf-isaac-detectnet-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-nitros-detection2-d-array-type-arm64.deb

# Triton packages
wget -c --proxy=on ${BASE_URL}/ros-humble-gxf-isaac-triton-arm64.deb
wget -c --proxy=on ${BASE_URL}/ros-humble-isaac-ros-triton-arm64.deb

echo "Download complete. You can now install these packages with:"
echo "sudo dpkg -i *.deb"
echo "If there are dependency errors, resolve them with:"
echo "sudo apt-get install -f"