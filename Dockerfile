# Use the official ROS Humble base image
FROM arm64v8/ros:humble-ros-base

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    net-tools \
    python3-rpi.gpio \
    rpi.gpio-common \
    gpiod \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install setuptools \
                 rpi_ws281x \
                 adafruit-circuitpython-neopixel

# Set up the ROS workspace
WORKDIR /ros_ws
