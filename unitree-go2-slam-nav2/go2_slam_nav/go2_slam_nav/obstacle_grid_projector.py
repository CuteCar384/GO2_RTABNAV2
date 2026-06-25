"""Fuse RTAB-Map /map with /cloud_obstacles into a 2D grid for Nav2 and explore_lite."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def _cloud_xyz_arrays(cloud: np.ndarray):
    """Return x/y/z float arrays from structured or Nx3 point clouds."""
    if cloud.size == 0:
        return (
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )

    if cloud.dtype.names and all(name in cloud.dtype.names for name in ('x', 'y', 'z')):
        return (
            np.asarray(cloud['x'], dtype=np.float64),
            np.asarray(cloud['y'], dtype=np.float64),
            np.asarray(cloud['z'], dtype=np.float64),
        )

    arr = np.asarray(cloud, dtype=np.float64)
    if arr.ndim == 2 and arr.shape[1] >= 3:
        return arr[:, 0], arr[:, 1], arr[:, 2]

    raise ValueError(f'unsupported point cloud array shape/dtype: {arr.shape} {cloud.dtype}')


class ObstacleGridProjector(Node):
    """
    Build /map_obstacles from RTAB-Map /map (free/unknown) plus 3D obstacle points.

    Obstacle cells come from /cloud_obstacles projection so 2D matches the 3D view.
    """

    def __init__(self):
        super().__init__('obstacle_grid_projector')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cloud_obstacles_topic', '/cloud_obstacles')
        self.declare_parameter('output_topic', '/map_obstacles')
        self.declare_parameter('output_updates_topic', '/map_obstacles_updates')
        self.declare_parameter('min_obstacle_height', 0.04)
        self.declare_parameter('max_obstacle_height', 1.5)
        self.declare_parameter('obstacle_value', 100)
        self.declare_parameter('inflate_cells', 1)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('max_cloud_points', 25000)

        map_topic = self.get_parameter('map_topic').value
        cloud_topic = self.get_parameter('cloud_obstacles_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_updates_topic = self.get_parameter('output_updates_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self._last_cloud = None
        self._map_msg = None
        self._dirty = False

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, cloud_qos)
        update_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(OccupancyGrid, output_topic, out_qos)
        self._updates_pub = self.create_publisher(
            OccupancyGridUpdate, output_updates_topic, update_qos)

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.2
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Obstacle grid: {map_topic} + {cloud_topic} -> {output_topic} '
            f'(updates: {output_updates_topic})'
        )

    def _map_cb(self, msg: OccupancyGrid):
        self._map_msg = msg
        self._dirty = True
        # Publish map-only grid immediately so Nav2 static_layer can subscribe early.
        if self._last_cloud is None:
            self._publish(force=True)

    def _cloud_cb(self, msg: PointCloud2):
        self._last_cloud = msg
        self._dirty = True

    def _project_cloud(self, cells: np.ndarray, info, obs_val: int, inflate: int) -> int:
        if self._last_cloud is None:
            return 0

        min_z = float(self.get_parameter('min_obstacle_height').value)
        max_z = float(self.get_parameter('max_obstacle_height').value)
        max_points = int(self.get_parameter('max_cloud_points').value)

        try:
            cloud = pc2.read_points_numpy(
                self._last_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            cloud = np.array(
                list(pc2.read_points(
                    self._last_cloud, field_names=('x', 'y', 'z'), skip_nans=True)),
                dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')],
            )

        n = len(cloud)
        if n == 0:
            return 0

        if n > max_points > 0:
            idx = np.linspace(0, n - 1, max_points, dtype=np.int64)
            cloud = cloud[idx]

        xs, ys, zs = _cloud_xyz_arrays(cloud)
        mask = (zs >= min_z) & (zs <= max_z)
        if not np.any(mask):
            return 0

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        width = info.width
        height = info.height

        mx = ((xs[mask] - ox) / res).astype(np.int32)
        my = ((ys[mask] - oy) / res).astype(np.int32)
        valid = (mx >= 0) & (mx < width) & (my >= 0) & (my < height)
        if not np.any(valid):
            return 0

        mx = mx[valid]
        my = my[valid]
        flat = my * width + mx
        cells[flat] = obs_val

        if inflate > 0:
            for di in range(-inflate, inflate + 1):
                for dj in range(-inflate, inflate + 1):
                    if di == 0 and dj == 0:
                        continue
                    cx = mx + di
                    cy = my + dj
                    inb = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
                    cells[cy[inb] * width + cx[inb]] = obs_val

        return int(len(flat))

    def _publish(self, force: bool = False):
        if self._map_msg is None:
            return
        if not force and not self._dirty:
            return

        info = self._map_msg.info
        width = info.width
        height = info.height
        if width == 0 or height == 0:
            return

        obs_val = int(self.get_parameter('obstacle_value').value)
        inflate = max(0, int(self.get_parameter('inflate_cells').value))

        try:
            base = np.array(self._map_msg.data, dtype=np.int8, copy=True)
            cells = base.copy()
            cloud_count = self._project_cloud(cells, info, obs_val, inflate)
        except Exception as exc:
            self.get_logger().error(f'obstacle grid publish failed: {exc}')
            return

        stamp = self.get_clock().now().to_msg()

        out = OccupancyGrid()
        out.header = self._map_msg.header
        out.header.stamp = stamp
        out.info = info
        out.data = cells.tolist()
        self._pub.publish(out)

        changed = np.flatnonzero(cells != base)
        if changed.size > 0:
            update = OccupancyGridUpdate()
            update.header = out.header
            update.x = 0
            update.y = 0
            update.width = width
            update.height = height
            update.data = cells.reshape(-1).tolist()
            self._updates_pub.publish(update)

        self._dirty = False

        self.get_logger().info(
            f'Published /map_obstacles ({width}x{height}), '
            f'projected {cloud_count} cloud points',
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGridProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()