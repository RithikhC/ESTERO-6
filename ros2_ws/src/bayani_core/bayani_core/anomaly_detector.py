"""Unsupervised water-surface anomaly detector for Bayani-6.

PaDiM-style approach with a diagonal Gaussian per feature location:
  1. Calibrate on clean water only (a ~60 s video or the live camera).
  2. For every new frame, measure how far each image patch is from "normal".
  3. Publish a 0..1 score on /bayani/anomaly_score (0.5 ~ the noisiest clean frame).

No labelled data is needed. Anything that does not look like the calibration water
(oil sheen, sludge, foam, weed mats) produces a high score.
"""
import os
import time

import cv2
import numpy as np
import rclpy
import torch
import torchvision
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

IMG = 224
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def load_resnet18():
    try:
        return torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.DEFAULT)
    except AttributeError:  # older torchvision on JetPack
        return torchvision.models.resnet18(pretrained=True)


class PatchFeatures(torch.nn.Module):
    """ResNet-18 layer2 + layer3 features at 1/8 resolution (28 x 28 patches for 224 px input)."""

    def __init__(self, n_channels=100, seed=0):
        super().__init__()
        m = load_resnet18()
        self.stem = torch.nn.Sequential(m.conv1, m.bn1, m.relu, m.maxpool, m.layer1)
        self.l2, self.l3 = m.layer2, m.layer3
        g = torch.Generator().manual_seed(seed)
        self.register_buffer("idx", torch.randperm(128 + 256, generator=g)[:n_channels])

    @torch.no_grad()
    def forward(self, x):
        f2 = self.l2(self.stem(x))
        f3 = torch.nn.functional.interpolate(self.l3(f2), size=f2.shape[-2:], mode="bilinear", align_corners=False)
        return torch.cat([f2, f3], 1)[:, self.idx]


class AnomalyDetector(Node):
    def __init__(self):
        super().__init__("anomaly_detector")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("stats_path", os.path.expanduser("~/.bayani/water_stats.npz"))
        self.declare_parameter("calibration_video", "")
        self.declare_parameter("calibrate_seconds", 60.0)
        self.declare_parameter("calibration_fps", 5.0)
        self.declare_parameter("publish_heatmap", True)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = PatchFeatures().to(self.device).eval()
        if self.device.type == "cuda":
            self.model = self.model.half()
        self.bridge = CvBridge()
        self.stats = None
        self.calib_frames = []
        self.calib_start = None
        self.last_calib = 0.0

        self.pub_score = self.create_publisher(Float32, "/bayani/anomaly_score", 10)
        self.pub_heat = self.create_publisher(Image, "/bayani/anomaly_heatmap", 2)

        path = self.get_parameter("stats_path").value
        video = self.get_parameter("calibration_video").value
        if os.path.exists(path):
            self.load_stats(path)
        elif video:
            self.calibrate_from_video(video)
        else:
            self.get_logger().info("No baseline found: calibrating from the live camera. Point it at CLEAN water.")

        self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos_profile_sensor_data)

    # ---------- feature helpers ----------
    def to_tensor(self, bgr):
        rgb = cv2.cvtColor(cv2.resize(bgr, (IMG, IMG), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        x = (rgb.astype(np.float32) / 255.0 - MEAN) / STD
        t = torch.from_numpy(x.transpose(2, 0, 1)).unsqueeze(0).to(self.device)
        return t.half() if self.device.type == "cuda" else t

    def features(self, bgr):
        return self.model(self.to_tensor(bgr)).float()[0]  # C x H x W

    def score_map(self, feat):
        z2 = (feat - self.stats["mean"]) ** 2 / self.stats["var"]
        m = z2.mean(0).cpu().numpy()
        return cv2.GaussianBlur(m, (0, 0), sigmaX=2.0)

    # ---------- calibration ----------
    def build_stats(self, frames):
        self.get_logger().info(f"Building clean-water baseline from {len(frames)} frames...")
        n, mean, m2 = 0, None, None
        for f in frames:  # Welford running mean / variance per location
            x = self.features(f)
            n += 1
            if mean is None:
                mean, m2 = x.clone(), torch.zeros_like(x)
            else:
                d = x - mean
                mean += d / n
                m2 += d * (x - mean)
        var = m2 / max(1, n - 1) + 0.01
        self.stats = {"mean": mean, "var": var}
        raw = [float(self.score_map(self.features(f)).max()) for f in frames]
        self.stats["p99"] = float(np.percentile(raw, 99))
        path = self.get_parameter("stats_path").value
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.savez(path, mean=mean.cpu().numpy(), var=var.cpu().numpy(), p99=self.stats["p99"])
        self.get_logger().info(f"Baseline saved to {path} (p99 raw score {self.stats['p99']:.3f})")

    def load_stats(self, path):
        d = np.load(path)
        self.stats = {
            "mean": torch.from_numpy(d["mean"]).to(self.device),
            "var": torch.from_numpy(d["var"]).to(self.device),
            "p99": float(d["p99"]),
        }
        self.get_logger().info(f"Loaded clean-water baseline from {path}")

    def calibrate_from_video(self, video):
        cap = cv2.VideoCapture(video)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        every = max(1, int(round(fps / self.get_parameter("calibration_fps").value)))
        frames, i = [], 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            if i % every == 0:
                frames.append(f)
            i += 1
        cap.release()
        if not frames:
            raise RuntimeError(f"Could not read frames from {video}")
        self.build_stats(frames)

    # ---------- runtime ----------
    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        now = time.monotonic()
        if self.stats is None:
            if self.calib_start is None:
                self.calib_start = now
            if now - self.last_calib >= 1.0 / self.get_parameter("calibration_fps").value:
                self.calib_frames.append(frame.copy())
                self.last_calib = now
            if now - self.calib_start >= self.get_parameter("calibrate_seconds").value:
                self.build_stats(self.calib_frames)
                self.calib_frames = []
            return

        smap = self.score_map(self.features(frame))
        score = float(np.clip(0.5 * smap.max() / self.stats["p99"], 0.0, 1.0))
        self.pub_score.publish(Float32(data=score))

        if self.get_parameter("publish_heatmap").value and self.pub_heat.get_subscription_count() > 0:
            heat = np.clip(0.5 * smap / self.stats["p99"], 0, 1)
            heat = cv2.resize((heat * 255).astype(np.uint8), (frame.shape[1], frame.shape[0]))
            overlay = cv2.addWeighted(frame, 0.6, cv2.applyColorMap(heat, cv2.COLORMAP_JET), 0.4, 0)
            out = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out.header = msg.header
            self.pub_heat.publish(out)


def main():
    rclpy.init()
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
