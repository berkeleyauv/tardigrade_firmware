#!/usr/bin/env python3
"""Tardigrade remote ground-station backend.

Runs on the machine the ESP is plugged into — typically the Jetson, reached
headlessly over SSH. It owns the one serial port and lets any number of remote
browsers drive the vehicle:

  browser  <--WebSocket-->  gcs_server  <--serial-->  ESP32

The server is a near-transparent BYTE PIPE. Command frames from a browser are
written to the serial port unmodified; bytes from the ESP are broadcast to every
connected browser. The frontend therefore reuses the exact same wire parser it
uses for a direct USB (WebSerial) connection — the only thing that changes is
where the bytes come from.

On the robosub it also injects pose: with --ros it subscribes to the EKF's
filtered odometry and writes Pose frames onto the same serial link, so the ESP's
ExternalEstimator is fed without a second process fighting for the port.

  # bench (no ROS):
  python3 gcs_server.py --serial /dev/ttyUSB0

  # robosub on the Jetson, injecting pose from the EKF:
  python3 gcs_server.py --serial /dev/ttyUSB0 --ros

Then, from ANY machine on the network:
  http://<jetson-hostname>:8080/

Dependencies:  pip install aiohttp pyserial     (+ a sourced ROS 2 for --ros)
"""

import argparse
import asyncio
import os
import sys
import threading

import tardigrade_protocol as tp

try:
    import serial  # pyserial
except ImportError:
    serial = None

try:
    from aiohttp import web, WSMsgType
except ImportError:
    web = None


HERE = os.path.dirname(os.path.abspath(__file__))


class SerialHub:
    """Owns the serial port. One writer lock, many broadcast subscribers."""

    def __init__(self, port, baud):
        if serial is None:
            raise RuntimeError("pyserial required:  pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2)
        self.write_lock = threading.Lock()
        self.clients = set()          # asyncio.Queue per browser
        self.loop = None              # set once the event loop is running
        self.parser = tp.Parser()     # server-side, only to count CRC errors
        self._stop = threading.Event()

    def write(self, data: bytes):
        # Serialised so a pose injection and an operator command can never
        # interleave mid-frame on the wire.
        with self.write_lock:
            try:
                self.ser.write(data)
            except Exception as exc:  # noqa: BLE001 - report, don't crash
                print(f"[serial] write failed: {exc}", file=sys.stderr)

    def reader_thread(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:  # noqa: BLE001
                print(f"[serial] read failed: {exc}", file=sys.stderr)
                continue
            if not data:
                continue
            for _ in self.parser.feed(data):
                pass  # advancing the parser is what tallies crc_errors
            if self.loop is not None:
                # Hand the raw bytes to the asyncio side for broadcast.
                self.loop.call_soon_threadsafe(self._fanout, data)

    def _fanout(self, data: bytes):
        for q in list(self.clients):
            if q.full():
                # A browser that cannot keep up must not stall telemetry for
                # everyone else. Drop its oldest frame.
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(data)

    def stop(self):
        self._stop.set()


async def ws_handler(request):
    hub = request.app["hub"]
    ws = web.WebSocketResponse(heartbeat=30)
    await ws.prepare(request)

    q = asyncio.Queue(maxsize=256)
    hub.clients.add(q)
    peer = request.remote
    print(f"[ws] browser connected: {peer} ({len(hub.clients)} total)")

    async def pump_to_browser():
        while True:
            data = await q.get()
            await ws.send_bytes(data)

    pump = asyncio.ensure_future(pump_to_browser())
    try:
        async for msg in ws:
            if msg.type == WSMsgType.BINARY:
                hub.write(msg.data)        # browser -> ESP, verbatim
            elif msg.type == WSMsgType.TEXT and msg.data == "ping":
                await ws.send_str("pong")
    finally:
        pump.cancel()
        hub.clients.discard(q)
        print(f"[ws] browser gone: {peer} ({len(hub.clients)} total)")
    return ws


async def index_handler(request):
    return web.FileResponse(os.path.join(HERE, "dashboard.html"))


async def status_handler(request):
    hub = request.app["hub"]
    return web.json_response({
        "browsers": len(hub.clients),
        "serial_crc_errors": hub.parser.crc_errors,
        "ros_pose_injection": request.app.get("ros", False),
    })


def start_ros_injection(hub, odom_topic):
    """Optional: subscribe to the EKF odometry and stream Pose frames.

    Runs rclpy in its own thread; the callback packs a frame and writes it
    through the same locked serial port the browsers use.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
    except ImportError:
        print("[ros] rclpy / nav_msgs not found — did you source ROS 2? "
              "Running WITHOUT pose injection.", file=sys.stderr)
        return

    class PoseBridge(Node):
        def __init__(self):
            super().__init__("tardigrade_pose_bridge")
            self.seq = 0
            self.count = 0
            self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
            self.get_logger().info(f"pose bridge: {odom_topic} -> ESP serial")

        def on_odom(self, msg):
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            lv = msg.twist.twist.linear
            av = msg.twist.twist.angular
            frame = tp.encode_pose(
                self.seq,
                (p.x, p.y, p.z),
                (o.w, o.x, o.y, o.z),   # firmware order is w,x,y,z
                (lv.x, lv.y, lv.z),
                (av.x, av.y, av.z),
            )
            hub.write(frame)
            self.seq = (self.seq + 1) & 0xFFFF
            self.count += 1
            if self.count % 60 == 0:  # ~2 s at 30 Hz
                self.get_logger().info(f"forwarded {self.count} poses")

    def spin():
        rclpy.init(args=None)
        node = PoseBridge()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    threading.Thread(target=spin, daemon=True).start()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", required=True, help="ESP serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0", help="0.0.0.0 = reachable from the network")
    ap.add_argument("--ros", action="store_true", help="inject pose from the EKF odometry topic")
    ap.add_argument("--odom-topic", default="/tardigrade/state/odometry/filtered")
    args = ap.parse_args()

    if web is None:
        print("aiohttp required:  pip install aiohttp", file=sys.stderr)
        return 1

    hub = SerialHub(args.serial, args.baud)
    threading.Thread(target=hub.reader_thread, daemon=True).start()
    if args.ros:
        start_ros_injection(hub, args.odom_topic)

    app = web.Application()
    app["hub"] = hub
    app["ros"] = args.ros
    app.router.add_get("/", index_handler)
    app.router.add_get("/status", status_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static", HERE)

    async def on_startup(app):
        hub.loop = asyncio.get_running_loop()

    app.on_startup.append(on_startup)

    print(f"[http] serving on http://{args.bind}:{args.http_port}/  "
          f"(serial {args.serial} @ {args.baud}"
          f"{', ROS pose injection ON' if args.ros else ''})")
    web.run_app(app, host=args.bind, port=args.http_port, print=None)
    hub.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
