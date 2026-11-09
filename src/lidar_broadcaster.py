import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import pandas as pd
import time


class LidarBroadcaster(Node):
    def __init__(self):
        super().__init__('lidar_broadcaster_node')
        # Create a Publisher for the point cloud topic
        self.pub = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.get_logger().info('LiDAR Broadcaster initialized. Waiting for network...')

        # Brief pause to allow ROS and Docker to establish network connections
        time.sleep(3)
        self.publish_frame(702)

    def publish_frame(self, target_frame):
        csv_path = '/data/lidar.csv'
        self.get_logger().info(f'Attempting to read frame {target_frame} from {csv_path}')

        try:
            # Read data and filter by target frame
            df = pd.read_csv(csv_path)
            frame_df = df[df['Frame'] == target_frame]

            if frame_df.empty:
                self.get_logger().error(f"Frame {target_frame} not found in the dataset.")
                return

            # Convert spatial coordinates from millimeters to meters
            # Dividing by 1000.0 aligns the data with standard ROS 2 metric units
            points = (frame_df[['x', 'y', 'z']] / 1000.0).values.tolist()

            # Create the standard ROS message header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'lidar_sensor_link'

            # Build the PointCloud2 message
            pc2_msg = point_cloud2.create_cloud_xyz32(header, points)

            # Publish the topic
            self.pub.publish(pc2_msg)
            self.get_logger().info(f'Successfully published {len(points)} points for Frame {target_frame}')

        except Exception as e:
            self.get_logger().error(f"Failed to process LiDAR data: {e}")


def main():
    rclpy.init()
    node = LidarBroadcaster()
    # Spin the node briefly to ensure the message is published before exit
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()