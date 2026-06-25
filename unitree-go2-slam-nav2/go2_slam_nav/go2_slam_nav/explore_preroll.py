"""Drive forward and rotate to prime SLAM map before explore_lite starts."""

import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class _Phase(Enum):
    WAIT_ODOM = auto()
    FORWARD = auto()
    ROTATE = auto()
    WAIT_NAV2 = auto()
    DONE = auto()


class ExplorePreroll(Node):
    def __init__(self):
        super().__init__('explore_preroll')

        self.declare_parameter('distance_m', 1.0)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('scan_rotation_rad', 6.28)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('odom_topic', '/go2/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_wait_timeout', 30.0)
        self.declare_parameter('max_duration', 60.0)
        self.declare_parameter('cmd_vel_rate', 20.0)
        self.declare_parameter('nav2_wait_timeout', 60.0)
        self.declare_parameter('nav2_action_name', 'navigate_to_pose')

        self._distance_m = float(self.get_parameter('distance_m').value)
        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._scan_rotation_rad = float(self.get_parameter('scan_rotation_rad').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._odom_topic = self.get_parameter('odom_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._odom_wait_timeout = float(self.get_parameter('odom_wait_timeout').value)
        self._max_duration = float(self.get_parameter('max_duration').value)
        self._nav2_wait_timeout = float(self.get_parameter('nav2_wait_timeout').value)
        nav2_action = str(self.get_parameter('nav2_action_name').value)
        cmd_rate = float(self.get_parameter('cmd_vel_rate').value)

        self._phase = _Phase.WAIT_ODOM
        self._start_x = None
        self._start_y = None
        self._start_yaw = None
        self._latest_odom = None
        self._move_start = None
        self._rotation_accum = 0.0
        self._prev_yaw = None
        self._odom_deadline = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=self._odom_wait_timeout)
        )
        self._nav2_deadline = None

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, odom_qos)
        self.create_timer(1.0 / cmd_rate, self._tick)

        try:
            from nav2_msgs.action import NavigateToPose
            self._nav_client = ActionClient(self, NavigateToPose, nav2_action)
        except ImportError:
            self._nav_client = None

        scan_part = (
            f' + rotate {math.degrees(self._scan_rotation_rad):.0f}°'
            if self._scan_rotation_rad > 0.1 else ''
        )
        self.get_logger().info(
            f'Explore preroll: drive {self._distance_m:.2f} m at '
            f'{self._linear_speed:.2f} m/s{scan_part}, then wait for Nav2'
        )

    @property
    def done(self) -> bool:
        return self._phase == _Phase.DONE

    @staticmethod
    def _yaw_from_odom(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg
        if self._start_x is not None:
            return
        self._start_x = msg.pose.pose.position.x
        self._start_y = msg.pose.pose.position.y
        self._start_yaw = self._yaw_from_odom(msg)
        self._prev_yaw = self._start_yaw
        self._move_start = self.get_clock().now()
        self._phase = _Phase.FORWARD
        self.get_logger().info(
            f'Odom baseline: ({self._start_x:.3f}, {self._start_y:.3f})'
        )

    def _distance_traveled(self) -> float:
        if self._latest_odom is None or self._start_x is None:
            return 0.0
        pose = self._latest_odom.pose.pose.position
        return math.hypot(pose.x - self._start_x, pose.y - self._start_y)

    def _publish_stop(self):
        stop = Twist()
        for _ in range(5):
            self._cmd_pub.publish(stop)

    def _finish(self, reason: str):
        if self.done:
            return
        self._phase = _Phase.DONE
        self._publish_stop()
        self.get_logger().info(reason)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _update_rotation(self) -> float:
        if self._latest_odom is None or self._prev_yaw is None:
            return 0.0
        yaw = self._yaw_from_odom(self._latest_odom)
        delta = self._normalize_angle(yaw - self._prev_yaw)
        self._prev_yaw = yaw
        self._rotation_accum += abs(delta)
        return self._rotation_accum

    def _enter_wait_nav2(self):
        self._publish_stop()
        self._phase = _Phase.WAIT_NAV2
        self._nav2_deadline = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=self._nav2_wait_timeout)
        )
        self.get_logger().info('Preroll motion done — waiting for Nav2 action server...')

    def _tick(self):
        if self.done:
            return

        now = self.get_clock().now()

        if self._move_start is not None:
            elapsed = (now - self._move_start).nanoseconds / 1e9
            if elapsed > self._max_duration:
                self._finish(
                    f'Max duration ({self._max_duration:.1f}s) reached — stopping preroll'
                )
                return

        if self._phase == _Phase.WAIT_ODOM:
            if now >= self._odom_deadline:
                self._finish('Odom timeout — skipping preroll')
            return

        if self._phase == _Phase.FORWARD:
            traveled = self._distance_traveled()
            if traveled >= self._distance_m:
                if self._scan_rotation_rad > 0.1:
                    self._rotation_accum = 0.0
                    self._prev_yaw = (
                        self._yaw_from_odom(self._latest_odom)
                        if self._latest_odom else self._start_yaw
                    )
                    self._phase = _Phase.ROTATE
                    self.get_logger().info(
                        f'Forward complete: {traveled:.2f} m — rotating to scan surroundings'
                    )
                else:
                    self._enter_wait_nav2()
                return

            cmd = Twist()
            cmd.linear.x = self._linear_speed
            self._cmd_pub.publish(cmd)
            return

        if self._phase == _Phase.ROTATE:
            rotated = self._update_rotation()
            if rotated >= self._scan_rotation_rad:
                self.get_logger().info(
                    f'Rotation scan complete: {math.degrees(rotated):.0f}°'
                )
                self._enter_wait_nav2()
                return

            cmd = Twist()
            cmd.angular.z = self._angular_speed
            self._cmd_pub.publish(cmd)
            return

        if self._phase == _Phase.WAIT_NAV2:
            if self._nav_client is None:
                self._finish('Nav2 action type unavailable — starting explore anyway')
                return
            if self._nav_client.wait_for_server(timeout_sec=0.0):
                self._finish(
                    'Preroll warmup complete — map primed and Nav2 ready for explore_lite'
                )
                return
            if self._nav2_deadline is not None and now >= self._nav2_deadline:
                self._finish(
                    f'Nav2 wait timeout ({self._nav2_wait_timeout:.0f}s) — starting explore anyway'
                )


def main(args=None):
    rclpy.init(args=args)
    node = ExplorePreroll()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()