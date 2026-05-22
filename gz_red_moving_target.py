#!/usr/bin/env python3
"""Spawn a smooth random red cube target in Gazebo / GZ Sim.

The target is meant for seeker / gimbal tracking tests. It moves inside a
bounded square, keeps a mostly constant speed, and changes direction smoothly.
Each planned heading change is kept below 90 deg so the path does not make
right-angle turns, U-turns, or short back-and-forth movements.

By default the motion runs inside Gazebo through RedMovingTargetController.
Use --external-pose only for the older service-based pose update mode.
"""

import argparse
import math
import random
import subprocess
import sys
import time


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="default", help="GZ world name")
    parser.add_argument("--model-name", default="red_moving_target", help="target model name")
    parser.add_argument("--gz-bin", default="gz", help="Gazebo CLI executable")
    parser.add_argument("--range", type=float, default=200.0, help="square movement range in meters")
    parser.add_argument("--center-x", type=float, default=0.0, help="movement square center x in meters")
    parser.add_argument("--center-y", type=float, default=0.0, help="movement square center y in meters")
    parser.add_argument("--z", type=float, default=2.0, help="target cube center height in meters")
    parser.add_argument("--size", type=float, default=2.0, help="cube side length in meters")
    parser.add_argument("--speed", type=float, default=8.0, help="target speed in m/s")
    parser.add_argument("--rate", type=float, default=30.0, help="pose update rate in Hz")
    parser.add_argument("--turn-rate", type=float, default=14.0, help="maximum smooth turn rate in deg/s")
    parser.add_argument("--turn-accel", type=float, default=18.0, help="maximum turn-rate acceleration in deg/s^2")
    parser.add_argument("--heading-response", type=float, default=0.8, help="heading correction response gain")
    parser.add_argument("--max-heading-change", type=float, default=55.0, help="planned heading change limit in deg")
    parser.add_argument("--min-heading-change", type=float, default=15.0, help="minimum random heading change in deg")
    parser.add_argument("--min-leg-time", type=float, default=6.0, help="minimum time before planning another turn")
    parser.add_argument("--max-leg-time", type=float, default=12.0, help="maximum time before planning another turn")
    parser.add_argument("--boundary-margin", type=float, default=25.0, help="start turning inward this far from edge")
    parser.add_argument("--seed", type=int, default=None, help="random seed for repeatable motion")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds; 0 runs forever")
    parser.add_argument("--dry-run", action="store_true", help="print poses without calling Gazebo")
    parser.add_argument("--no-spawn", action="store_true", help="do not create the cube, only update its pose")
    parser.add_argument("--external-pose", action="store_true", help="use old external /set_pose service updates")
    return parser.parse_args()


def make_cube_sdf(model_name, args, plugin_motion):
    size = args.size
    plugin = ""

    if plugin_motion:
        seed = args.seed if args.seed is not None else 0
        plugin = f"""
    <plugin filename="libRedMovingTargetController.so" name="custom::RedMovingTargetController">
      <center_x>{args.center_x:.6f}</center_x>
      <center_y>{args.center_y:.6f}</center_y>
      <z>{args.z:.6f}</z>
      <range>{args.range:.6f}</range>
      <speed>{args.speed:.6f}</speed>
      <turn_rate>{args.turn_rate:.6f}</turn_rate>
      <turn_accel>{args.turn_accel:.6f}</turn_accel>
      <heading_response>{args.heading_response:.6f}</heading_response>
      <max_heading_change>{args.max_heading_change:.6f}</max_heading_change>
      <min_heading_change>{args.min_heading_change:.6f}</min_heading_change>
      <min_leg_time>{args.min_leg_time:.6f}</min_leg_time>
      <max_leg_time>{args.max_leg_time:.6f}</max_leg_time>
      <boundary_margin>{args.boundary_margin:.6f}</boundary_margin>
      <seed>{seed}</seed>
    </plugin>"""

    return f"""<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="red_cube_visual">
        <geometry>
          <box>
            <size>{size:.3f} {size:.3f} {size:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.2 0.05 0.05 1</specular>
          <emissive>0.6 0 0 1</emissive>
        </material>
      </visual>
      <collision name="red_cube_collision">
        <geometry>
          <box>
            <size>{size:.3f} {size:.3f} {size:.3f}</size>
          </box>
        </geometry>
      </collision>
    </link>
{plugin}
  </model>
</sdf>"""


class GzTarget:
    def __init__(self, args):
        self._gz_bin = args.gz_bin
        self._world = args.world
        self._model_name = args.model_name
        self._args = args
        self._dry_run = args.dry_run
        self._warned = False

    def spawn(self, x, y, z, yaw, plugin_motion):
        if self._dry_run:
            print(f"spawn {self._model_name} at x={x:.2f} y={y:.2f} z={z:.2f}", flush=True)
            print(make_cube_sdf(self._model_name, self._args, plugin_motion), flush=True)
            return

        sdf = make_cube_sdf(self._model_name, self._args, plugin_motion)
        req = (
            f'name: "{self._model_name}", allow_renaming: false, '
            f'pose: {{ position: {{ x: {x:.3f}, y: {y:.3f}, z: {z:.3f} }}, '
            f'orientation: {{ w: {math.cos(yaw * 0.5):.7f}, z: {math.sin(yaw * 0.5):.7f} }} }}, '
            f'sdf: {sdf!r}'
        )

        result = subprocess.run(
            [
                self._gz_bin,
                "service",
                "-s",
                f"/world/{self._world}/create",
                "--reqtype",
                "gz.msgs.EntityFactory",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                req,
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip() or f"exit code {result.returncode}"
            print(f"Gazebo spawn warning: {message}", file=sys.stderr)
            print("Continuing with pose updates; use --model-name to avoid an existing-name collision.", file=sys.stderr)

    def set_pose(self, x, y, z, yaw):
        if self._dry_run:
            print(f"x={x:+8.2f} y={y:+8.2f} z={z:+5.2f} yaw={math.degrees(yaw):+7.2f}deg", flush=True)
            return

        req = (
            f'name: "{self._model_name}", '
            f'position: {{ x: {x:.3f}, y: {y:.3f}, z: {z:.3f} }}, '
            f'orientation: {{ w: {math.cos(yaw * 0.5):.7f}, z: {math.sin(yaw * 0.5):.7f} }}'
        )

        try:
            process = subprocess.Popen(
                [
                    self._gz_bin,
                    "service",
                    "-s",
                    f"/world/{self._world}/set_pose",
                    "--reqtype",
                    "gz.msgs.Pose",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "1000",
                    "--req",
                    req,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as exc:
            if not self._warned:
                print(f"Gazebo pose update failed: {exc}", file=sys.stderr)
                self._warned = True
            return

        _stdout, stderr = process.communicate()

        if process.returncode != 0 and not self._warned:
            message = stderr.strip() if stderr else f"exit code {process.returncode}"
            print(f"Gazebo pose update failed: {message}", file=sys.stderr)
            self._warned = True


class SmoothSnakeMotion:
    def __init__(self, args):
        self._cx = args.center_x
        self._cy = args.center_y
        self._half_range = max(args.range * 0.5, 1.0)
        self._margin = clamp(args.boundary_margin, 1.0, self._half_range * 0.45)
        self._speed = max(args.speed, 0.1)
        self._turn_rate = math.radians(max(args.turn_rate, 1.0))
        self._turn_accel = math.radians(max(args.turn_accel, 1.0))
        self._heading_response = max(args.heading_response, 0.1)
        max_heading_change_deg = clamp(args.max_heading_change, 1.0, 89.0)
        self._max_heading_change = math.radians(max_heading_change_deg)
        self._min_heading_change = math.radians(clamp(args.min_heading_change, 0.0, max_heading_change_deg))
        self._min_leg_time = max(args.min_leg_time, 0.5)
        self._max_leg_time = max(args.max_leg_time, self._min_leg_time)
        self.x = self._cx
        self.y = self._cy
        self.heading = random.uniform(-math.pi, math.pi)
        self.target_heading = self.heading
        self.heading_rate = 0.0
        self._next_turn_time = 0.0

    def update(self, now, dt):
        if now >= self._next_turn_time or self._near_boundary():
            self._plan_next_heading(now)

        heading_error = wrap_pi(self.target_heading - self.heading)
        desired_rate = clamp(heading_error * self._heading_response, -self._turn_rate, self._turn_rate)
        rate_error = desired_rate - self.heading_rate
        rate_step = self._turn_accel * dt
        self.heading_rate += clamp(rate_error, -rate_step, rate_step)
        self.heading = wrap_pi(self.heading + self.heading_rate * dt)
        self.x += math.cos(self.heading) * self._speed * dt
        self.y += math.sin(self.heading) * self._speed * dt

        x_min = self._cx - self._half_range
        x_max = self._cx + self._half_range
        y_min = self._cy - self._half_range
        y_max = self._cy + self._half_range
        self.x = clamp(self.x, x_min, x_max)
        self.y = clamp(self.y, y_min, y_max)

        return self.x, self.y, self.heading

    def _near_boundary(self):
        x_min = self._cx - self._half_range + self._margin
        x_max = self._cx + self._half_range - self._margin
        y_min = self._cy - self._half_range + self._margin
        y_max = self._cy + self._half_range - self._margin
        return self.x < x_min or self.x > x_max or self.y < y_min or self.y > y_max

    def _plan_next_heading(self, now):
        if self._near_boundary():
            inward = math.atan2(self._cy - self.y, self._cx - self.x)
            jitter = random.uniform(-self._max_heading_change, self._max_heading_change)
            desired = wrap_pi(inward + jitter)
            delta = clamp(wrap_pi(desired - self.heading), -self._max_heading_change, self._max_heading_change)
        else:
            sign = random.choice((-1.0, 1.0))
            amount = random.uniform(self._min_heading_change, self._max_heading_change)
            delta = sign * amount

        self.target_heading = wrap_pi(self.heading + delta)
        self._next_turn_time = now + random.uniform(self._min_leg_time, self._max_leg_time)


def main():
    args = parse_args()
    random.seed(args.seed)

    rate = max(args.rate, 1.0)
    dt_limit = 1.0 / rate
    motion = SmoothSnakeMotion(args)
    target = GzTarget(args)

    if not args.no_spawn:
        target.spawn(motion.x, motion.y, args.z, motion.heading, plugin_motion=not args.external_pose)

    if not args.external_pose:
        if args.duration > 0.0:
            time.sleep(args.duration)

        return

    start = time.monotonic()
    last = start

    try:
        while True:
            now = time.monotonic()

            if args.duration > 0.0 and now - start >= args.duration:
                break

            dt = min(now - last, 0.2)
            last = now
            x, y, yaw = motion.update(now, dt)
            target.set_pose(x, y, args.z, yaw)

            elapsed = time.monotonic() - now
            time.sleep(max(0.0, dt_limit - elapsed))

    except KeyboardInterrupt:
        print("\nStopped red moving target.", file=sys.stderr)


if __name__ == "__main__":
    main()
