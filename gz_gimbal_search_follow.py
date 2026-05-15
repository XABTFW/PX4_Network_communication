#!/usr/bin/env python3
"""Sweep a Gazebo x500 gimbal and follow target LOS offsets when available.

The script publishes directly to Gazebo's gimbal joint command topics. During
search it scans yaw in rows. When a detector sends a fresh target packet over
UDP, it adjusts yaw/pitch to center the target.

Accepted UDP JSON examples:
  {"valid": true, "los_x_rad": 0.03, "los_y_rad": -0.01}
  {"target_valid": true, "los_x_deg": 2.0, "los_y_deg": -1.0}

Accepted UDP text example:
  1 0.03 -0.01
"""

import argparse
import json
import math
import socket
import subprocess
import sys
import time


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="x500_gimbal", help="Gazebo model name, or 'auto'")
    parser.add_argument("--gz-bin", default="gz", help="Gazebo CLI executable")
    parser.add_argument("--rate", type=float, default=10.0, help="command publish rate in Hz")
    parser.add_argument("--yaw-min", type=float, default=-70.0, help="search yaw min in deg")
    parser.add_argument("--yaw-max", type=float, default=70.0, help="search yaw max in deg")
    parser.add_argument("--pitch-top", type=float, default=20.0, help="search top pitch in deg")
    parser.add_argument("--pitch-bottom", type=float, default=-45.0, help="search bottom pitch in deg")
    parser.add_argument("--pitch-step", type=float, default=15.0, help="pitch row step in deg")
    parser.add_argument("--yaw-speed", type=float, default=25.0, help="search yaw speed in deg/s")
    parser.add_argument("--udp-bind", default="127.0.0.1", help="target packet bind address")
    parser.add_argument("--udp-port", type=int, default=15200, help="target packet UDP port")
    parser.add_argument("--no-target-udp", action="store_true", help="disable UDP target input and search forever")
    parser.add_argument("--target-timeout", type=float, default=0.35, help="target freshness timeout in seconds")
    parser.add_argument("--follow-gain", type=float, default=0.8, help="LOS-to-gimbal proportional gain")
    parser.add_argument("--max-follow-step", type=float, default=4.0, help="max follow correction per tick in deg")
    parser.add_argument("--yaw-sign", type=float, default=1.0, help="set -1 if horizontal tracking moves the wrong way")
    parser.add_argument("--pitch-sign", type=float, default=-1.0, help="set 1 if vertical tracking moves the wrong way")
    parser.add_argument("--dry-run", action="store_true", help="print commands without publishing to Gazebo")
    return parser.parse_args()


def list_gz_topics(gz_bin):
    try:
        result = subprocess.run(
            [gz_bin, "topic", "-l"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"failed to list Gazebo topics with '{gz_bin} topic -l': {exc}") from exc

    return result.stdout.splitlines()


def resolve_topics(gz_bin, model):
    if model != "auto":
        return (
            f"/model/{model}/command/gimbal_yaw",
            f"/model/{model}/command/gimbal_pitch",
        )

    yaw_topic = None
    pitch_topic = None

    for topic in list_gz_topics(gz_bin):
        if topic.endswith("/command/gimbal_yaw"):
            yaw_topic = topic

        elif topic.endswith("/command/gimbal_pitch"):
            pitch_topic = topic

    if yaw_topic is None or pitch_topic is None:
        raise RuntimeError("could not auto-detect gimbal command topics; pass --model x500_gimbal")

    return yaw_topic, pitch_topic


class GzPublisher:
    def __init__(self, gz_bin, yaw_topic, pitch_topic, dry_run):
        self._gz_bin = gz_bin
        self._yaw_topic = yaw_topic
        self._pitch_topic = pitch_topic
        self._dry_run = dry_run
        self._warned = False

    def publish(self, yaw_deg, pitch_deg):
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)

        if self._dry_run:
            print(f"yaw={yaw_deg:+7.2f}deg pitch={pitch_deg:+7.2f}deg", flush=True)
            return

        self._publish_one(self._yaw_topic, yaw_rad)
        self._publish_one(self._pitch_topic, pitch_rad)

    def _publish_one(self, topic, value_rad):
        payload = f"data: {value_rad:.7f}"

        try:
            subprocess.run(
                [self._gz_bin, "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", payload],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            if not self._warned:
                print(f"Gazebo publish failed on {topic}: {exc}", file=sys.stderr)
                self._warned = True


class TargetReceiver:
    def __init__(self, bind_addr, port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_addr, port))
        self._sock.setblocking(False)
        self.valid = False
        self.los_x_rad = 0.0
        self.los_y_rad = 0.0
        self.last_update = 0.0

    def poll(self):
        while True:
            try:
                data, _addr = self._sock.recvfrom(2048)
            except BlockingIOError:
                return

            self._parse(data.decode("utf-8", errors="replace").strip())

    def fresh(self, now, timeout_s):
        return self.valid and (now - self.last_update) <= timeout_s

    def _parse(self, text):
        try:
            msg = json.loads(text)
            valid = bool(msg.get("valid", msg.get("target_valid", False)))

            if "los_x_rad" in msg:
                los_x = float(msg["los_x_rad"])
            else:
                los_x = math.radians(float(msg.get("los_x_deg", 0.0)))

            if "los_y_rad" in msg:
                los_y = float(msg["los_y_rad"])
            else:
                los_y = math.radians(float(msg.get("los_y_deg", 0.0)))

        except (ValueError, TypeError, json.JSONDecodeError):
            parts = text.replace(",", " ").split()

            if len(parts) < 3:
                return

            try:
                valid = bool(int(float(parts[0])))
                los_x = float(parts[1])
                los_y = float(parts[2])
            except ValueError:
                return

        self.valid = valid
        self.los_x_rad = los_x
        self.los_y_rad = los_y
        self.last_update = time.monotonic()


class SearchPattern:
    def __init__(self, args):
        self._yaw_min = args.yaw_min
        self._yaw_max = args.yaw_max
        self._pitch_top = args.pitch_top
        self._pitch_bottom = args.pitch_bottom
        self._pitch_step = max(1.0, args.pitch_step)
        self._yaw_speed = max(1.0, args.yaw_speed)
        self.yaw = self._yaw_min
        self.pitch = self._pitch_top
        self._direction = 1.0

    def update(self, dt):
        self.yaw += self._direction * self._yaw_speed * dt

        if self.yaw >= self._yaw_max:
            self.yaw = self._yaw_max
            self._next_row()

        elif self.yaw <= self._yaw_min:
            self.yaw = self._yaw_min
            self._next_row()

        return self.yaw, self.pitch

    def _next_row(self):
        self._direction *= -1.0
        self.pitch -= self._pitch_step

        if self.pitch < self._pitch_bottom:
            self.pitch = self._pitch_top

    def set_pose(self, yaw, pitch):
        self.yaw = clamp(yaw, self._yaw_min, self._yaw_max)
        self.pitch = clamp(pitch, self._pitch_bottom, self._pitch_top)


def main():
    args = parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be greater than zero")

    try:
        yaw_topic, pitch_topic = resolve_topics(args.gz_bin, args.model)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(1) from exc

    print(f"publishing gimbal commands: yaw={yaw_topic}, pitch={pitch_topic}", flush=True)
    if args.no_target_udp:
        print("target UDP input disabled; search mode only", flush=True)

    else:
        print(f"listening for target offsets on udp://{args.udp_bind}:{args.udp_port}", flush=True)

    publisher = GzPublisher(args.gz_bin, yaw_topic, pitch_topic, args.dry_run)
    target = None if args.no_target_udp else TargetReceiver(args.udp_bind, args.udp_port)
    search = SearchPattern(args)

    period = 1.0 / args.rate
    last_time = time.monotonic()
    last_log = 0.0

    while True:
        now = time.monotonic()
        dt = max(0.0, now - last_time)
        last_time = now
        if target is not None:
            target.poll()

        if target is not None and target.fresh(now, args.target_timeout):
            yaw_correction = args.yaw_sign * math.degrees(target.los_x_rad) * args.follow_gain
            pitch_correction = args.pitch_sign * math.degrees(target.los_y_rad) * args.follow_gain
            yaw_correction = clamp(yaw_correction, -args.max_follow_step, args.max_follow_step)
            pitch_correction = clamp(pitch_correction, -args.max_follow_step, args.max_follow_step)
            yaw = search.yaw + yaw_correction
            pitch = search.pitch + pitch_correction
            search.set_pose(yaw, pitch)
            mode = "follow"

        else:
            yaw, pitch = search.update(dt)
            mode = "search"

        publisher.publish(search.yaw, search.pitch)

        if now - last_log > 1.0:
            print(f"{mode:6s} yaw={search.yaw:+7.2f}deg pitch={search.pitch:+7.2f}deg", flush=True)
            last_log = now

        sleep_s = period - (time.monotonic() - now)

        if sleep_s > 0.0:
            time.sleep(sleep_s)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
