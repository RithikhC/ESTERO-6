"""Bayani-6 arm sampler: runs the myCobot 280 dip-sampling sequence on request.

Listens on /arm/command for "dip_sample", moves through pre-taught joint poses with
pymycobot, holds the gripper at the water line for `hold_s` seconds, stows the arm and
replies "done" (or "error") on /arm/status.

Joint poses are in degrees and should be re-taught for your mounting and creek bank height.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

STOW = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
REACH = [-90.0, -30.0, -40.0, -20.0, 0.0, 0.0]
DIP = [-90.0, -58.0, -72.0, -40.0, 0.0, 12.0]


class ArmSampler(Node):
    def __init__(self):
        super().__init__("arm_sampler")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("speed", 40)
        self.declare_parameter("hold_s", 3.0)
        self.declare_parameter("dry_run", False)

        self.arm = None
        if not self.get_parameter("dry_run").value:
            from pymycobot.mycobot import MyCobot
            self.arm = MyCobot(self.get_parameter("port").value, self.get_parameter("baud").value)
            self.move(STOW)
        self.busy = False
        self.pub = self.create_publisher(String, "/arm/status", 10)
        self.create_subscription(String, "/arm/command", self.on_command, 10)
        self.get_logger().info("arm_sampler ready" + (" (dry run)" if self.arm is None else ""))

    def move(self, pose, settle=2.5):
        if self.arm is not None:
            self.arm.send_angles(pose, self.get_parameter("speed").value)
        time.sleep(settle)

    def on_command(self, msg):
        if msg.data != "dip_sample" or self.busy:
            return
        self.busy = True
        try:
            self.get_logger().info("dip_sample: reach")
            self.move(REACH)
            self.get_logger().info("dip_sample: descend to water line")
            self.move(DIP)
            time.sleep(self.get_parameter("hold_s").value)
            self.get_logger().info("dip_sample: stow")
            self.move(REACH, settle=1.5)
            self.move(STOW)
            self.pub.publish(String(data="done"))
        except Exception as e:  # serial errors, etc.
            self.get_logger().error(f"dip_sample failed: {e}")
            self.pub.publish(String(data="error"))
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = ArmSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
