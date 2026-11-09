import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class EmergencyBrakingNode(Node):
    def __init__(self):
        super().__init__('emergency_braking_node')
        # Subscribe to the topic published by the broadcaster container
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            10)
        self.get_logger().info('AEB System initialized. Listening for LiDAR data...')

    def lidar_callback(self, msg):
        self.get_logger().info('Received LiDAR point cloud! Processing...')
        start_time = time.perf_counter()

        # Extract points from the standard ROS 2 message
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if not points:
            self.get_logger().warning('Point cloud is empty.')
            return

        # Define the Region of Interest (ROI) for braking
        # Ego vehicle travels forward along the X-axis, Y is lateral
        X_MIN, X_MAX = 0.5, 10.0  # 0.5 to 10 meters forward
        Y_MIN, Y_MAX = -1.5, 1.5  # 1.5 meters to each side (lane width)
        Z_MIN, Z_MAX = 0.8, 2.0  # Static ground filtering: Ignore everything below 80cm

        all_x, all_y = [], []
        danger_x, danger_y = [], []

        for p in points:
            x, y, z = p[0], p[1], p[2]
            all_x.append(x)
            all_y.append(y)

            # Check if the point falls inside our 3D braking bounding box
            if (X_MIN <= x <= X_MAX) and (Y_MIN <= y <= Y_MAX) and (Z_MIN <= z <= Z_MAX):
                danger_x.append(x)
                danger_y.append(y)

        process_time_ms = (time.perf_counter() - start_time) * 1000.0

        # Braking decision: require > 50 points to filter out sensor noise
        is_braking = len(danger_x) > 50
        status_msg = "EMERGENCY BRAKE ACTIVATED!" if is_braking else "PATH CLEAR"
        self.get_logger().info(f"Status: {status_msg} | Obstacle points: {len(danger_x)}")

        # Export visual results to the /results directory
        self.save_results(all_x, all_y, danger_x, danger_y, is_braking, process_time_ms)

        # Terminate after successfully processing the frame
        raise SystemExit

    def save_results(self, all_x, all_y, danger_x, danger_y, is_braking, p_time):
        os.makedirs('/results', exist_ok=True)

        # 1. Write a structured log file
        with open('/results/aeb_log.txt', 'w') as f:
            f.write("--- Autonomous Emergency Braking (AEB) Report ---\n")
            f.write(f"Processing Time: {p_time:.2f} ms\n")
            f.write(f"Points inside Danger Zone: {len(danger_x)}\n")
            f.write(f"System Decision: {'BRAKE!' if is_braking else 'PROCEED'}\n")

        # 2. Draw a top-down view graph
        plt.figure(figsize=(10, 8))

        # Plot Y (lateral) on the X-axis and X (forward) on the Y-axis
        plt.scatter(all_y, all_x, s=2, c='#1f77b4', alpha=0.4, label='LiDAR Point Cloud')

        if danger_x:
            plt.scatter(danger_y, danger_x, s=15, c='red', label='Obstacle (Danger Zone)')

        # Ego Vehicle position at the origin
        plt.plot(0, 0, marker='^', markersize=16, color='black', label='Ego Vehicle')

        # Draw the ROI bounding box (10m)
        roi_rect = patches.Rectangle((-1.5, 0.5), 3.0, 9.5, linewidth=2, edgecolor='orange', facecolor='none',
                                     linestyle='--', label='AEB ROI')
        plt.gca().add_patch(roi_rect)

        title_color = 'red' if is_braking else 'green'
        plt.title(f"AEB System Status: {'BRAKE!' if is_braking else 'CLEAR'}", color=title_color, fontsize=16,
                  weight='bold')
        plt.xlabel('Lateral Distance Y [m]')
        plt.ylabel('Forward Distance X [m]')
        plt.legend(loc='upper right')
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.axis('equal')

        # Fix plot limits to keep the view focused
        plt.xlim(-12, 12)
        plt.ylim(-5, 20)

        plt.savefig('/results/aeb_visual_result.png', dpi=200, bbox_inches='tight')
        plt.close()
        self.get_logger().info('Results successfully saved to /results folder.')


def main():
    rclpy.init()
    node = EmergencyBrakingNode()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()