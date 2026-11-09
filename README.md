# BGRacing LiDAR Autonomous Emergency Braking (AEB) Pipeline

An end-to-end containerized ROS 2 data processing pipeline simulating an Autonomous Emergency Braking (AEB) system using raw LiDAR point cloud data. 

## Architecture Overview

The system is built using a shared Docker image based on Ubuntu 24.04 and ROS 2 Jazzy. Two distinct containers communicate over a dedicated Docker bridge network (`aeb_net`).

1. **LiDAR Broadcaster Node**
   - Reads raw LiDAR sensor data from a dataset (`lidar.csv`) using Pandas.
   - Normalizes spatial coordinates from millimeters to metric standard units.
   - Converts the data into a standard ROS 2 `sensor_msgs/PointCloud2` message.
   - Publishes the point cloud to the `/lidar/points` topic.

2. **Emergency Braking Node**
   - Subscribes to the `/lidar/points` topic.
   - **Ground Filtering:** Applies a static Z-axis filter (ignoring points below 0.8m) to remove the road surface from the point cloud.
   - **ROI Detection:** Defines a 3D bounding box (0.5m to 10m forward, 3m lateral width) to detect obstacles in the ego vehicle's direct path.
   - **Decision Logic:** Calculates point density within the ROI to trigger an emergency `BRAKE!` command or a `PROCEED` status.
   - **Visualization:** Generates a real-time top-down scatter plot and a structured decision log.

## Example Output
*(The system successfully detected an obstacle at ~5.5m while filtering out the ground)*
![AEB System Result]

## Prerequisites
- Docker
- Docker Compose

## Build and Run Instructions

1. Place your raw point cloud dataset inside the `data/` directory and name it `lidar.csv`.
2. Deploy the pipeline:
   ```bash
   docker compose up --build
