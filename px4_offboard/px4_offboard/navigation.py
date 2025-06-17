import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

class SimplePathChecker:
    def __init__(self, grid, resolution, origin):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin
        self.shape = grid.shape

    def index_from_world(self, point):
        indices = ((np.array(point[:2]) - self.origin[:2]) / self.resolution).astype(int)
        indices = np.clip(indices, [0, 0], [self.shape[0]-1, self.shape[1]-1])
        return tuple(indices)

    def is_path_clear(self, start_world, goal_world):
        """Check if direct line path is clear of obstacles using Bresenham's line algorithm"""
        start_idx = self.index_from_world(start_world)
        goal_idx = self.index_from_world(goal_world)
        
        # Get all points along the line using Bresenham's algorithm
        line_points = self._bresenham_line(start_idx[0], start_idx[1], goal_idx[0], goal_idx[1])
        
        # Check if any point along the line is an obstacle
        for point in line_points:
            if self.grid[point] != 0:  # Obstacle found
                return False
        return True
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get all grid points along a line"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return points

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # QoS Profile as specified
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Publishers and Subscribers
        self.position_pub = self.create_publisher(TrajectorySetpoint, '/position_goal', qos_profile)
        self.goal_sub = self.create_subscription(PointStamped, '/goal_point', self.goal_callback, qos_profile)
        self.map_sub = self.create_subscription(OccupancyGrid, '/projected_map', self.gridmap_callback, qos_map)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)

        # State variables
        self.grid = None
        self.resolution = 0.2
        self.origin = np.array([0.0, 0.0])
        self.path_checker = None
        self.current_position = np.array([0.0, 0.0, 1.0])

    def local_pos_callback(self, msg):
        self.current_position = np.array([
            msg.x,
            msg.y,
            msg.z
        ])

    def gridmap_callback(self, msg):
        self.get_logger().info("Received OccupancyGrid from /projected_map")
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])

        # Handle occupancy grid data
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Handle unknown cells (-1) by treating them as obstacles
        self.grid = np.where((data > 50) | (data == -1), 1, 0).astype(np.uint8)
        
        self.path_checker = SimplePathChecker(self.grid, self.resolution, self.origin)
        self.get_logger().info(f"Path checker initialized with map size: {width}x{height}, resolution: {self.resolution}")

    def goal_callback(self, msg):
        if self.path_checker is None:
            self.get_logger().warn("Path checker not initialized: grid map not yet received")
            return

        start = self.current_position.tolist()
        goal = [msg.point.x, msg.point.y, self.current_position[2]]

        self.get_logger().info(f"Checking direct path from {start[:2]} to {goal[:2]}")
        
        # Check if direct path is possible
        if self.path_checker.is_path_clear(start, goal):
            self.get_logger().info("Direct path is clear - sending goal directly")
            self.publish_position_goal(goal[0], goal[1], goal[2])
        else:
            self.get_logger().warn("Direct path blocked by obstacles - goal not sent")

    def publish_position_goal(self, x, y, z, yaw=None):
        """Publish position goal as specified"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float('nan') if yaw is None else yaw
        msg.yawspeed = 0.0
        
        self.position_pub.publish(msg)
        self.get_logger().info(f"Published Position Goal: {[x, y, z]}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()