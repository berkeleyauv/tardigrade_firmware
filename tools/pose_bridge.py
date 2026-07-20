#!/usr/bin/env python3
"""Standalone Jetson -> ESP pose bridge (no webapp).

Subscribes to the robot_localization EKF's filtered odometry and streams Pose
frames to the ESP over serial. Use this for autonomy or bench runs where no
operator dashboard is involved.

If you ALSO want the remote dashboard, do NOT run this — run gcs_server.py --ros
instead, which injects pose AND serves the webapp from one process. The serial
port can only be owned once, so the two are mutually exclusive.

  source /opt/ros/humble/setup.bash
  python3 pose_bridge.py --serial /dev/ttyUSB0

Dependencies:  ROS 2 (rclpy, nav_msgs) + pyserial.
"""

import argparse
import sys

import tardigrade_protocol as tp

try:
    import serial
except ImportError:
    serial = None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--odom-topic", default="/tardigrade/state/odometry/filtered")
    args = ap.parse_args()

    if serial is None:
        print("pyserial required:  pip install pyserial", file=sys.stderr)
        return 1
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
    except ImportError:
        print("ROS 2 not found — source your setup.bash first.", file=sys.stderr)
        return 1

    ser = serial.Serial(args.serial, args.baud, write_timeout=0.2)

    class PoseBridge(Node):
        def __init__(self):
            super().__init__("tardigrade_pose_bridge")
            self.seq = 0
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
            self.get_logger().info(
                f"{args.odom_topic} -> {args.serial} @ {args.baud}")

        def on_odom(self, msg):
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            lv = msg.twist.twist.linear
            av = msg.twist.twist.angular
            # firmware quaternion order is (w, x, y, z)
            ser.write(tp.encode_pose(
                self.seq, (p.x, p.y, p.z), (o.w, o.x, o.y, o.z),
                (lv.x, lv.y, lv.z), (av.x, av.y, av.z)))
            self.seq = (self.seq + 1) & 0xFFFF

    rclpy.init(args=None)
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
