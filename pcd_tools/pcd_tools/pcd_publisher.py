import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'points', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Load point cloud from .ply
        cloud = o3d.io.read_point_cloud("/home/ainhoaarnaiz/test.ply")
        self.points_np = np.asarray(cloud.points, dtype=np.float32)

        # Filter out NaN/Inf
        finite_mask = np.isfinite(self.points_np).all(axis=1)
        self.points_np = self.points_np[finite_mask]

        # # Filter out distant points (e.g., > 20 m)
        # dist = np.linalg.norm(self.points_np, axis=1)
        # self.points_np = self.points_np[dist < 20.0]
        
        # # Shift Z up by 2.5 meters ---------> DEPENDS ON .PLY
        # self.points_np[:, 2] += 2.5

        # Debug
        z_vals = self.points_np[:, 2]
        self.get_logger().info(f"Loaded {self.points_np.shape[0]} valid points")
        self.get_logger().info(f"Z range: {z_vals.min():.2f} to {z_vals.max():.2f}")

    def timer_callback(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, self.points_np)
        self.publisher.publish(cloud_msg)
        self.get_logger().info('Published filtered point cloud.')

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

