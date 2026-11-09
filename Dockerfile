# Official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Install Python, Pandas, Matplotlib, and ROS 2 point cloud dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pandas \
    python3-matplotlib \
    ros-jazzy-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app
COPY src/ /app/src/

# Environment variables for ROS network and Python output
ENV ROS_DOMAIN_ID=20
ENV PYTHONUNBUFFERED=1