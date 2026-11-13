"""Bayani-6 mission master: patrol -> detect -> halt -> pin -> sample -> map -> resume.

* Drives a looping waypoint patrol with Nav2 (nav2_simple_commander).
* Watches /bayani/anomaly_score. Above the trigger threshold it cancels the Nav2 task,
  looks up the robot pose (map -> base_link) through tf2 and pins a hotspot.
* Asks arm_sampler for a dip sample on /arm/command and waits for "done" on /arm/status.
* Appends the hotspot to a CSV, publishes it on /bayani/hotspot and resumes the patrol
  from the waypoint it was heading to.
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MasterIO(Node):
    """Topic + tf2 side of the master. Navigation runs through BasicNavigator in main()."""

    def __init__(self):
        super().__init__("bayani_master")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("threshold", 0.8)
        self.declare_parameter("cooldown_s", 3.0)
        self.declare_parameter("dedupe_radius_m", 1.5)
        self.declare_parameter("arm_timeout_s", 30.0)
        self.declare_parameter("csv_path", os.path.expanduser("~/.bayani/hotspots.csv"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.score = 0.0
        self.peak = 0.0
        self.triggered = False
        self.last_trigger = 0.0
        self.arm_status = ""

        self.create_subscription(Float32, "/bayani/anomaly_score", self.on_score, 10)
        self.create_subscription(String, "/arm/status", self.on_arm_status, 10)
        self.pub_arm = self.create_publisher(String, "/arm/command", 10)
        self.pub_hotspot = self.create_publisher(PointStamped, "/bayani/hotspot", 10)

    def on_score(self, msg):
        self.score = msg.data
        thr = self.get_parameter("threshold").value
        cooldown = self.get_parameter("cooldown_s").value
        if self.triggered:
            self.peak = max(self.peak, msg.data)
        elif msg.data > thr and time.monotonic() - self.last_trigger > cooldown:
            self.triggered = True
            self.peak = msg.data
            self.get_logger().warn(f"anomaly_score {msg.data:.2f} > {thr:.2f}: halting")

    def on_arm_status(self, msg):
        self.arm_status = msg.data

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value, self.get_parameter("base_frame").value, Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as e:
            self.get_logger().error(f"tf2 lookup failed: {e}")
            return None

    def clear_trigger(self):
        self.triggered = False
        self.last_trigger = time.monotonic()


class HotspotLog:
    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.points = []
        if os.path.exists(path):
            with open(path) as f:
                for row in csv.DictReader(f):
                    self.points.append((float(row["x_m"]), float(row["y_m"])))
        else:
            with open(path, "w", newline="") as f:
                csv.writer(f).writerow(["id", "timestamp", "x_m", "y_m", "score", "sample"])

    def near(self, x, y, r):
        return any(math.hypot(x - px, y - py) < r for px, py in self.points)

    def add(self, x, y, score, sampled):
        hid = f"H{len(self.points) + 1}"
        with open(self.path, "a", newline="") as f:
            csv.writer(f).writerow([hid, datetime.now().isoformat(timespec="seconds"), f"{x:.3f}", f"{y:.3f}", f"{score:.3f}", "ok" if sampled else "failed"])
        self.points.append((x, y))
        return hid


def load_waypoints(nav, path, frame):
    with open(path) as f:
        data = yaml.safe_load(f)
    poses = []
    for wp in data["waypoints"]:
        p = PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(wp["x"]), float(wp["y"])
        q = yaw_to_quat(float(wp.get("yaw", 0.0)))
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        poses.append(p)
    return poses


def wait_for_arm(io, timeout):
    io.arm_status = ""
    io.pub_arm.publish(String(data="dip_sample"))
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < timeout:
        rclpy.spin_once(io, timeout_sec=0.1)
        if io.arm_status in ("done", "error"):
            return io.arm_status == "done"
    io.get_logger().error("arm_sampler did not answer in time")
    return False


def main():
    rclpy.init()
    io = MasterIO()
    nav = BasicNavigator()
    nav.waitUntilNav2Active(localizer="bt_navigator")  # Cartographer provides map->odom, no AMCL

    poses = load_waypoints(nav, io.get_parameter("waypoints_file").value, io.get_parameter("map_frame").value)
    log = HotspotLog(io.get_parameter("csv_path").value)
    io.get_logger().info(f"Patrol loaded: {len(poses)} waypoints. Hotspots -> {log.path}")

    start = 0
    nav.followWaypoints(poses[start:])
    try:
        while rclpy.ok():
            rclpy.spin_once(io, timeout_sec=0.05)
            fb = nav.getFeedback()
            current = start + (fb.current_waypoint if fb else 0)

            if nav.isTaskComplete() and not io.triggered:
                start = 0  # loop the patrol
                nav.followWaypoints(poses)
                continue

            if not io.triggered:
                continue

            nav.cancelTask()
            xy = io.robot_xy()
            if xy is None:
                io.clear_trigger()
            elif log.near(*xy, io.get_parameter("dedupe_radius_m").value):
                io.get_logger().info(f"Known hotspot near ({xy[0]:.2f}, {xy[1]:.2f}): continuing")
                io.clear_trigger()
            else:
                io.get_logger().info(f"Pinned hotspot at ({xy[0]:.2f}, {xy[1]:.2f}), sampling")
                ok = wait_for_arm(io, io.get_parameter("arm_timeout_s").value)
                hid = log.add(xy[0], xy[1], io.peak, ok)
                pt = PointStamped()
                pt.header.frame_id = io.get_parameter("map_frame").value
                pt.header.stamp = io.get_clock().now().to_msg()
                pt.point.x, pt.point.y = xy
                io.pub_hotspot.publish(pt)
                io.get_logger().info(f"{hid} logged (score {io.peak:.2f}, sample {'ok' if ok else 'FAILED'})")
                io.clear_trigger()

            start = min(current, len(poses) - 1)
            nav.followWaypoints(poses[start:])
    except KeyboardInterrupt:
        pass
    finally:
        nav.cancelTask()
        io.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
