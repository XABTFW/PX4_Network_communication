#!/usr/bin/env python3
"""Detect a red target in ROS2 images and send DYT telemetry frames to PX4."""

import math
import os
import struct
import sys
import time
import tty

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


FRAME_LEN = 32
SYNC_1 = 0xEE
SYNC_2_TELEMETRY = 0x16

TRACKING_STATE_SEARCH = "SEARCH"
TRACKING_STATE_LOCKED = "LOCKED"

LOS_SCALE_DEG = 0.05
GIMBAL_SCALE_DEG = 0.01
BBOX_SCALE_PX = 4.0


def angle_to_raw(angle_deg, scale_deg):
    raw = int(round(angle_deg / scale_deg))
    return max(-32768, min(32767, raw))


def bbox_to_raw(size_px):
    raw = int(round(size_px / BBOX_SCALE_PX))
    return max(0, min(255, raw))


def put_s16_le(frame, offset, value):
    struct.pack_into("<h", frame, offset, value)


def checksum(frame_without_checksum):
    return sum(frame_without_checksum) & 0xFF


def target_valid_for_state(tracking_state):
    return tracking_state == TRACKING_STATE_LOCKED


def build_frame(los_x_rad, los_y_rad, bbox_width_px, bbox_height_px, tracking_state, frame_counter):
    frame = bytearray(FRAME_LEN)
    frame[0] = SYNC_1
    frame[1] = SYNC_2_TELEMETRY

    if tracking_state == TRACKING_STATE_LOCKED:
        frame[2] = 1 << 2

    put_s16_le(frame, 6, angle_to_raw(math.degrees(los_x_rad), LOS_SCALE_DEG))
    put_s16_le(frame, 8, angle_to_raw(math.degrees(los_y_rad), LOS_SCALE_DEG))

    put_s16_le(frame, 10, angle_to_raw(0.0, GIMBAL_SCALE_DEG))
    put_s16_le(frame, 12, angle_to_raw(0.0, GIMBAL_SCALE_DEG))
    put_s16_le(frame, 14, angle_to_raw(0.0, GIMBAL_SCALE_DEG))

    if tracking_state == TRACKING_STATE_LOCKED:
        frame[16] = bbox_to_raw(bbox_width_px)
        frame[17] = bbox_to_raw(bbox_height_px)

    frame[28] = 1 << 7
    frame[29] = frame_counter & 0xFF
    frame[30] = (frame_counter >> 8) & 0xFF
    frame[31] = checksum(frame[:-1])
    return bytes(frame)


def open_port(path):
    if not os.path.exists(path):
        print(
            f"{path} does not exist. Start the virtual serial pair first, for example:\n"
            "  socat -d -d pty,raw,echo=0,link=/tmp/dyt_cam_0 "
            "pty,raw,echo=0,link=/tmp/dyt_px4_0",
            file=sys.stderr,
        )
        raise SystemExit(1)

    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    except OSError as exc:
        print(f"failed to open {path}: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    tty.setraw(fd)
    return fd


def write_frame_once(fd, data):
    written = os.write(fd, data)

    if written != len(data):
        raise RuntimeError(f"short serial write: {written}/{len(data)} bytes")


class RosImageToDytSender(Node):
    def __init__(self):
        super().__init__("ros_image_to_dyt_sender")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("port", "/tmp/dyt_cam_0")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx0", -1.0)
        self.declare_parameter("cy0", -1.0)
        self.declare_parameter("min_area", 100.0)

        self._image_topic = self.get_parameter("image_topic").value
        self._port = self.get_parameter("port").value
        self._rate = float(self.get_parameter("rate").value)
        self._fx = float(self.get_parameter("fx").value)
        self._fy = float(self.get_parameter("fy").value)
        self._cx0 = float(self.get_parameter("cx0").value)
        self._cy0 = float(self.get_parameter("cy0").value)
        self._min_area = float(self.get_parameter("min_area").value)

        if self._rate <= 0.0:
            raise ValueError("rate must be greater than 0")

        self._fd = open_port(self._port)
        self._bridge = CvBridge()
        self._frame_counter = 0
        self._last_send_s = None
        self._tracking_state = TRACKING_STATE_SEARCH
        self._los_x_rad = 0.0
        self._los_y_rad = 0.0
        self._bbox_width_px = 0.0
        self._bbox_height_px = 0.0
        self._bbox_area = 0.0

        self.create_subscription(Image, self._image_topic, self._image_callback, 10)
        self.create_timer(1.0 / self._rate, self._timer_callback)

        self.get_logger().info(
            f"sending DYT frames to {self._port} at {self._rate:.1f}Hz, image_topic={self._image_topic}"
        )

    def destroy_node(self):
        if getattr(self, "_fd", None) is not None:
            os.close(self._fd)
            self._fd = None

        super().destroy_node()

    def _image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            self._set_search()
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red_1 = np.array([0, 100, 80], dtype=np.uint8)
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_red_2 = np.array([170, 100, 80], dtype=np.uint8)
        upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_red_1, upper_red_1) | cv2.inRange(hsv, lower_red_2, upper_red_2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self._set_search()
            return

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))

        if area <= self._min_area:
            self._set_search(area)
            return

        x, y, w, h = cv2.boundingRect(contour)
        target_cx = x + w * 0.5
        target_cy = y + h * 0.5
        fx = self._fx if self._fx > 0.0 else float(image.shape[1])
        fy = self._fy if self._fy > 0.0 else float(image.shape[0])
        cx0 = self._cx0 if self._cx0 >= 0.0 else float(image.shape[1]) * 0.5
        cy0 = self._cy0 if self._cy0 >= 0.0 else float(image.shape[0]) * 0.5

        self._tracking_state = TRACKING_STATE_LOCKED
        self._los_x_rad = math.atan((target_cx - cx0) / fx)
        self._los_y_rad = math.atan((target_cy - cy0) / fy)
        self._bbox_width_px = float(w)
        self._bbox_height_px = float(h)
        self._bbox_area = area

    def _set_search(self, area=0.0):
        self._tracking_state = TRACKING_STATE_SEARCH
        self._los_x_rad = 0.0
        self._los_y_rad = 0.0
        self._bbox_width_px = 0.0
        self._bbox_height_px = 0.0
        self._bbox_area = float(area)

    def _timer_callback(self):
        self._frame_counter = (self._frame_counter + 1) & 0xFFFFFFFF
        target_valid = target_valid_for_state(self._tracking_state)
        frame = build_frame(
            self._los_x_rad,
            self._los_y_rad,
            self._bbox_width_px,
            self._bbox_height_px,
            self._tracking_state,
            self._frame_counter,
        )

        send_time_s = time.monotonic()
        write_frame_once(self._fd, frame)
        send_dt_s = send_time_s - self._last_send_s if self._last_send_s is not None else float("nan")
        self._last_send_s = send_time_s

        print(
            f"target_valid={target_valid} tracking_state={self._tracking_state} "
            f"los_x_deg={math.degrees(self._los_x_rad):+.2f} "
            f"los_y_deg={math.degrees(self._los_y_rad):+.2f} "
            f"bbox_area={self._bbox_area:.1f} frame_counter={self._frame_counter} "
            f"send_dt_s={send_dt_s:.4f}",
            flush=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RosImageToDytSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
