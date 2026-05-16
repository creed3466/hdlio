#!/usr/bin/env python3
"""
nrx_odom_converter.py
 nav_ros_msgs/msg/Odometry (custom NRX CDR raw bytes from bag)
 -> nav_msgs/msg/Odometry

Approach: read raw CDR bytes directly from the bag SQLite DB,
parse the custom layout, and publish nav_msgs/msg/Odometry on /odom
synchronized with /clock (use_sim_time).

CDR layout of nav_ros_msgs/msg/Odometry (128 bytes, little-endian):
  [0:4]   CDR encapsulation header
  [4:8]   seq (uint32)
  [8:12]  padding (zeros)
  [12:20] stamp (uint64, nanoseconds absolute)
  [20:24] frame_id string len (uint32 = 5)
  [24:29] "odom\0"
  [29:32] padding
  [32:36] child_frame_id string len (uint32 = 10)
  [36:46] "base_link\0"
  [46:52] padding
  [52:76] pose: x, y, theta (3 x float64)
  [76:108] eular: yaw, pitch, roll, gyro_z (4 x float64)
  [104:128] twist: vx, vy, va (3 x float64)

Mapping to nav_msgs/msg/Odometry:
  pose.position.x/y  = pose.x / pose.y
  pose.orientation   = quaternion from eular.yaw
  twist.linear.x/y   = twist.vx / twist.vy
  twist.angular.z    = twist.va
"""

import struct
import math
import time
import sqlite3
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.clock import ClockType
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def parse_nrx_odom_cdr(data: bytes):
    """
    Parse raw CDR bytes of nav_ros_msgs/msg/Odometry (128 bytes).
    Returns (stamp_sec, stamp_nsec, px, py, yaw, vx, vy, va) or None.
    """
    if len(data) < 128:
        return None
    try:
        stamp_ns   = struct.unpack_from('<Q', data, 12)[0]
        stamp_sec  = int(stamp_ns // 1_000_000_000)
        stamp_nsec = int(stamp_ns %  1_000_000_000)
        px, py, _theta     = struct.unpack_from('<3d', data, 52)
        yaw, _p, _r, _gz   = struct.unpack_from('<4d', data, 76)
        vx, vy, va         = struct.unpack_from('<3d', data, 104)
        return stamp_sec, stamp_nsec, px, py, yaw, vx, vy, va
    except struct.error:
        return None


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class NrxOdomConverter(Node):
    """
    Reads /das_service/odometry_pb rows from bag SQLite DB,
    parses the custom CDR, and publishes /odom at the correct sim-time.
    Uses /clock to drive playback timing.
    """

    def __init__(self, pre_msgs=None):
        super().__init__('nrx_odom_converter')

        self.declare_parameter('bag_db', '/root/dataset/nrx/robot_data_19700101_000536/robot_data_19700101_000536_0.db3')
        bag_db = self.get_parameter('bag_db').get_parameter_value().string_value

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(Odometry, '/odom', best_effort_qos)

        # Use pre-loaded messages if available, otherwise load now
        if pre_msgs is not None:
            self._msgs = pre_msgs
            self.get_logger().info(
                f'nrx_odom_converter: using {len(self._msgs)} pre-loaded odometry msgs')
        else:
            self._msgs = self._load_bag(bag_db)
            self.get_logger().info(
                f'nrx_odom_converter: loaded {len(self._msgs)} odometry msgs from {bag_db}')

        self._idx  = 0
        self._count = 0

        # Subscribe to /clock to drive publishing
        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_cb, best_effort_qos)

        self.get_logger().info('nrx_odom_converter ready -> /odom')

    def _load_bag(self, db_path: str):
        """Load all odometry CDR messages sorted by timestamp."""
        msgs = []
        try:
            # Use normal connect (not URI mode) with check_same_thread disabled
            conn = sqlite3.connect(db_path, check_same_thread=False)
            # Get topic_id for /das_service/odometry_pb
            row = conn.execute(
                "SELECT id FROM topics WHERE name='/das_service/odometry_pb';"
            ).fetchone()
            if row is None:
                self.get_logger().error('Topic /das_service/odometry_pb not found in bag DB')
                conn.close()
                return []
            topic_id = row[0]
            rows = conn.execute(
                'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp;',
                (topic_id,)
            ).fetchall()
            conn.close()
            for ts, data in rows:
                parsed = parse_nrx_odom_cdr(bytes(data))
                if parsed:
                    msgs.append((ts, parsed))
        except Exception as e:
            self.get_logger().error(f'Failed to load bag: {e}')
        return msgs

    def _clock_cb(self, msg: Clock):
        """Publish all odometry messages whose timestamp <= current sim time."""
        if not self._msgs:
            return

        # current sim time in ns
        sim_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

        published = 0
        while self._idx < len(self._msgs):
            bag_ts_ns, parsed = self._msgs[self._idx]
            if bag_ts_ns > sim_ns:
                break

            stamp_sec, stamp_nsec, px, py, yaw, vx, vy, va = parsed

            out = Odometry()
            out.header.frame_id = 'odom'
            out.header.stamp.sec    = stamp_sec
            out.header.stamp.nanosec = stamp_nsec
            out.child_frame_id = 'base_link'
            out.pose.pose.position.x = px
            out.pose.pose.position.y = py
            out.pose.pose.position.z = 0.0
            out.pose.pose.orientation = yaw_to_quat(yaw)
            out.twist.twist.linear.x  = vx
            out.twist.twist.linear.y  = vy
            out.twist.twist.angular.z = va

            self._pub.publish(out)
            self._idx  += 1
            self._count += 1
            published   += 1

            if self._count == 1:
                self.get_logger().info(
                    f'[nrx_odom_converter] FIRST ODOM: '
                    f't={stamp_sec}.{stamp_nsec:09d} '
                    f'pos=({px:.4f},{py:.4f}) yaw={yaw:.4f} '
                    f'vel=({vx:.4f},{va:.4f})'
                )
            if self._count % 500 == 0:
                self.get_logger().info(
                    f'[nrx_odom_converter] published {self._count}/{len(self._msgs)}'
                )


def main():
    # Load bag data BEFORE rclpy.init() to avoid sqlite3 library conflicts
    # Use subprocess to isolate from ROS LD_LIBRARY_PATH sqlite3 override
    import sys
    import subprocess
    import json

    bag_db = '/root/dataset/nrx/robot_data_19700101_000536/robot_data_19700101_000536_0.db3'

    # Parse --ros-args -p bag_db:=... from argv before rclpy takes over
    for i, arg in enumerate(sys.argv):
        if arg == '-p' and i + 1 < len(sys.argv) and sys.argv[i+1].startswith('bag_db:='):
            bag_db = sys.argv[i+1].split(':=', 1)[1]
            break

    print(f'[nrx_odom_converter] Pre-loading bag via subprocess: {bag_db}', flush=True)

    loader_code = f"""
import sqlite3, struct, json, sys
db = "{bag_db}"
try:
    conn = sqlite3.connect(db, check_same_thread=False)
    row = conn.execute("SELECT id FROM topics WHERE name='/das_service/odometry_pb';").fetchone()
    topic_id = row[0]
    cursor = conn.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp;', (topic_id,))
    out = []
    try:
        for r in cursor:
            ts, data = r
            d = bytes(data)
            if len(d) < 128: continue
            stamp_ns = struct.unpack_from('<Q', d, 12)[0]
            px, py = struct.unpack_from('<2d', d, 52)
            yaw = struct.unpack_from('<d', d, 76)[0]
            vx, vy, va = struct.unpack_from('<3d', d, 104)
            out.append([ts, int(stamp_ns//1_000_000_000), int(stamp_ns%1_000_000_000), px, py, yaw, vx, vy, va])
    except Exception as e:
        sys.stderr.write(f"Stopped at {{len(out)}}: {{e}}\\n")
    conn.close()
    print(json.dumps(out))
except Exception as e:
    sys.stderr.write(f"ERR: {{e}}\\n")
    print("[]")
"""

    result = subprocess.run(
        ['python3', '-c', loader_code],
        capture_output=True, text=True, timeout=60,
        env={k: v for k, v in __import__('os').environ.items()
             if k not in ('LD_LIBRARY_PATH', 'LD_PRELOAD')}
    )
    if result.stderr:
        print(f'[nrx_odom_converter] loader stderr: {result.stderr.strip()}', flush=True)

    pre_msgs = []
    try:
        raw = json.loads(result.stdout.strip())
        for row in raw:
            ts, sec, nsec, px, py, yaw, vx, vy, va = row
            pre_msgs.append((int(ts), (int(sec), int(nsec), px, py, yaw, vx, vy, va)))
        print(f'[nrx_odom_converter] Pre-loaded {len(pre_msgs)} msgs', flush=True)
    except Exception as e:
        print(f'[nrx_odom_converter] Parse failed: {e}', flush=True)

    rclpy.init()
    node = NrxOdomConverter(pre_msgs=pre_msgs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'[nrx_odom_converter] total published: {node._count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
