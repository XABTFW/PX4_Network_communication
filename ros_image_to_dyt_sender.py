#!/usr/bin/env python3
"""Detect a red target in ROS2 images and send DYT telemetry frames to PX4."""

import json
import math
import os
import socket
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
        self.declare_parameter("h1_low", 0)
        self.declare_parameter("h1_high", 15)
        self.declare_parameter("h2_low", 165)
        self.declare_parameter("h2_high", 180)
        self.declare_parameter("s_min", 70)
        self.declare_parameter("v_min", 50)
        self.declare_parameter("image_timeout_s", 0.3)
        self.declare_parameter("enable_gz_gimbal_udp", False)
        self.declare_parameter("udp_target_host", "127.0.0.1")
        self.declare_parameter("udp_target_port", 15200)

        self._image_topic = self.get_parameter("image_topic").value
        self._port = self.get_parameter("port").value
        self._rate = float(self.get_parameter("rate").value)
        self._fx = float(self.get_parameter("fx").value)
        self._fy = float(self.get_parameter("fy").value)
        self._cx0 = float(self.get_parameter("cx0").value)
        self._cy0 = float(self.get_parameter("cy0").value)
        self._min_area = float(self.get_parameter("min_area").value)
        self._h1_low = int(self.get_parameter("h1_low").value)
        self._h1_high = int(self.get_parameter("h1_high").value)
        self._h2_low = int(self.get_parameter("h2_low").value)
        self._h2_high = int(self.get_parameter("h2_high").value)
        self._s_min = int(self.get_parameter("s_min").value)
        self._v_min = int(self.get_parameter("v_min").value)
        self._image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self._enable_gz_gimbal_udp = bool(self.get_parameter("enable_gz_gimbal_udp").value)
        self._udp_target_host = self.get_parameter("udp_target_host").value
        self._udp_target_port = int(self.get_parameter("udp_target_port").value)

        if self._rate <= 0.0:
            raise ValueError("rate must be greater than 0")

        self._fd = open_port(self._port)
        self._bridge = CvBridge()
        self._frame_counter = 0
        self._last_send_s = None
        self._last_image_s = None
        self._tracking_state = TRACKING_STATE_SEARCH
        self._los_x_rad = 0.0
        self._los_y_rad = 0.0
        self._bbox_width_px = 0.0
        self._bbox_height_px = 0.0
        self._bbox_area = 0.0
        self._contour_count = 0
        self._max_area = 0.0
        self._gz_gimbal_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self._enable_gz_gimbal_udp else None
        self._gz_gimbal_udp_addr = (self._udp_target_host, self._udp_target_port)
        self._gz_gimbal_udp_warned = False

        self.create_subscription(Image, self._image_topic, self._image_callback, 10)
        self._mask_pub = self.create_publisher(Image, "/dyt_debug/mask", 10)
        self._overlay_pub = self.create_publisher(Image, "/dyt_debug/overlay", 10)
        self.create_timer(1.0 / self._rate, self._timer_callback)

        self.get_logger().info(
            f"sending DYT frames to {self._port} at {self._rate:.1f}Hz, image_topic={self._image_topic}"
        )
        if self._enable_gz_gimbal_udp:
            self.get_logger().info(
                f"sending Gazebo gimbal target UDP to udp://{self._udp_target_host}:{self._udp_target_port}"
            )

    def destroy_node(self):
        if getattr(self, "_fd", None) is not None:
            os.close(self._fd)
            self._fd = None

        if getattr(self, "_gz_gimbal_udp_sock", None) is not None:
            self._gz_gimbal_udp_sock.close()
            self._gz_gimbal_udp_sock = None

        super().destroy_node()

    def _image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            self._set_search()
            return

        self._last_image_s = time.monotonic()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red_1 = np.array([self._h1_low, self._s_min, self._v_min], dtype=np.uint8)
        upper_red_1 = np.array([self._h1_high, 255, 255], dtype=np.uint8)
        lower_red_2 = np.array([self._h2_low, self._s_min, self._v_min], dtype=np.uint8)
        upper_red_2 = np.array([self._h2_high, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_red_1, upper_red_1) | cv2.inRange(hsv, lower_red_2, upper_red_2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self._contour_count = len(contours)
        overlay = image.copy()

        if not contours:
            self._set_search()
            self._draw_overlay(overlay, None, None, None, None, False)
            self._publish_debug_images(mask, overlay, msg.header)
            return

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        self._max_area = area

        if area <= self._min_area:
            x, y, w, h = cv2.boundingRect(contour)
            self._set_search(area)
            self._draw_overlay(overlay, x, y, w, h, False)
            self._publish_debug_images(mask, overlay, msg.header)
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
        self._draw_overlay(overlay, x, y, w, h, True)
        self._publish_debug_images(mask, overlay, msg.header)

    def _set_search(self, area=0.0):
        self._tracking_state = TRACKING_STATE_SEARCH
        self._los_x_rad = 0.0
        self._los_y_rad = 0.0
        self._bbox_width_px = 0.0
        self._bbox_height_px = 0.0
        self._bbox_area = float(area)

        if area == 0.0:
            self._max_area = 0.0

    def _draw_overlay(self, overlay, x, y, w, h, target_valid):
        image_h, image_w = overlay.shape[:2]
        cx0 = self._cx0 if self._cx0 >= 0.0 else float(image_w) * 0.5
        cy0 = self._cy0 if self._cy0 >= 0.0 else float(image_h) * 0.5
        image_cx = int(round(cx0))
        image_cy = int(round(cy0))
        color = (0, 255, 0) if target_valid else (0, 165, 255)

        if x is not None and y is not None and w is not None and h is not None:
            target_cx = int(round(x + w * 0.5))
            target_cy = int(round(y + h * 0.5))
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 2)
            cv2.circle(overlay, (target_cx, target_cy), 4, color, -1)

        cv2.drawMarker(overlay, (image_cx, image_cy), (255, 0, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(
            overlay,
            f"target_valid={target_valid}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            f"bbox_area={self._bbox_area:.1f} los_x_deg={math.degrees(self._los_x_rad):+.2f} "
            f"los_y_deg={math.degrees(self._los_y_rad):+.2f}",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

    def _publish_debug_images(self, mask, overlay, header):
        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding="mono8")
        overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        mask_msg.header = header
        overlay_msg.header = header
        self._mask_pub.publish(mask_msg)
        self._overlay_pub.publish(overlay_msg)

    def _image_timed_out(self):
        return self._last_image_s is None or time.monotonic() - self._last_image_s > self._image_timeout_s

    def _send_gz_gimbal_udp(self, target_valid, los_x_rad, los_y_rad):
        if self._gz_gimbal_udp_sock is None:
            return

        msg = {
            "valid": bool(target_valid),
            "los_x_rad": float(los_x_rad),
            "los_y_rad": float(los_y_rad),
        }

        try:
            self._gz_gimbal_udp_sock.sendto(
                json.dumps(msg, separators=(",", ":")).encode("utf-8"),
                self._gz_gimbal_udp_addr,
            )
        except OSError as exc:
            if not self._gz_gimbal_udp_warned:
                self.get_logger().warn(f"Gazebo gimbal UDP send failed: {exc}")
                self._gz_gimbal_udp_warned = True

    def _timer_callback(self):
        self._frame_counter = (self._frame_counter + 1) & 0xFFFFFFFF
        image_timed_out = self._image_timed_out()
        tracking_state = TRACKING_STATE_SEARCH if image_timed_out else self._tracking_state
        los_x_rad = 0.0 if image_timed_out else self._los_x_rad
        los_y_rad = 0.0 if image_timed_out else self._los_y_rad
        bbox_width_px = 0.0 if image_timed_out else self._bbox_width_px
        bbox_height_px = 0.0 if image_timed_out else self._bbox_height_px
        target_valid = target_valid_for_state(tracking_state)
        frame = build_frame(
            los_x_rad,
            los_y_rad,
            bbox_width_px,
            bbox_height_px,
            tracking_state,
            self._frame_counter,
        )

        send_time_s = time.monotonic()
        write_frame_once(self._fd, frame)
        send_dt_s = send_time_s - self._last_send_s if self._last_send_s is not None else float("nan")
        self._last_send_s = send_time_s
        self._send_gz_gimbal_udp(target_valid, los_x_rad, los_y_rad)

        print(
            f"target_valid={target_valid} tracking_state={tracking_state} "
            f"contour_count={self._contour_count} max_area={self._max_area:.1f} "
            f"bbox_width={bbox_width_px:.1f} bbox_height={bbox_height_px:.1f} "
            f"los_x_deg={math.degrees(los_x_rad):+.2f} "
            f"los_y_deg={math.degrees(los_y_rad):+.2f} "
            f"frame_counter={self._frame_counter} send_dt_s={send_dt_s:.4f}",
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
