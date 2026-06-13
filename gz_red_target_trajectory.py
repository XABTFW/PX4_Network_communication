#!/usr/bin/env python3
"""Record and live-plot a Gazebo model trajectory from /world/<world>/pose/info."""

import argparse
import csv
import html
import math
import os
import re
import subprocess
import sys
import threading
import time


NAME_RE = re.compile(r'\bname:\s*"([^"]+)"')
POSITION_RE = re.compile(r"\bposition\s*\{([^{}]*)\}", re.DOTALL)
FIELD_RE = re.compile(r"\b([xyz]):\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="default", help="Gazebo world name")
    parser.add_argument("--model-name", default="red_moving_target", help="model name to record")
    parser.add_argument("--gz-bin", default="gz", help="Gazebo CLI executable")
    parser.add_argument("--topic", default=None, help="pose topic; default is /world/<world>/pose/info")
    parser.add_argument("--duration", type=float, default=60.0, help="record seconds; 0 records until Ctrl-C")
    parser.add_argument("--sample-rate", type=float, default=30.0, help="maximum saved samples per second; 0 saves all")
    parser.add_argument("--refresh-rate", type=float, default=10.0, help="live plot refresh rate in Hz")
    parser.add_argument("--no-live", action="store_true", help="record without opening a live plot window")
    parser.add_argument("--csv", default="red_moving_target_trajectory.csv", help="output CSV path")
    parser.add_argument("--svg", default="red_moving_target_trajectory.svg", help="output SVG path")
    return parser.parse_args()


def pose_blocks(lines):
    block = []
    depth = 0
    recording = False

    for line in lines:
        if not recording and re.match(r"\s*pose\s*\{", line):
            recording = True

        if recording:
            block.append(line)
            depth += line.count("{") - line.count("}")

            if depth == 0:
                yield "".join(block)
                block = []
                recording = False


def parse_pose(block, model_name):
    name_match = NAME_RE.search(block)

    if not name_match or name_match.group(1) != model_name:
        return None

    position_match = POSITION_RE.search(block)

    if not position_match:
        return None

    fields = {match.group(1): float(match.group(2)) for match in FIELD_RE.finditer(position_match.group(1))}

    if not {"x", "y", "z"}.issubset(fields):
        return None

    return fields["x"], fields["y"], fields["z"]


class TrajectoryRecorder:
    def __init__(self, args):
        self._args = args
        self._samples = []
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._process = None
        self.error = None

    @property
    def stopped(self):
        return self._stop_event.is_set()

    def stop(self):
        self._stop_event.set()

        if self._process is not None and self._process.poll() is None:
            self._process.terminate()

    def samples(self):
        with self._lock:
            return list(self._samples)

    def run(self):
        topic = self._args.topic or f"/world/{self._args.world}/pose/info"
        command = [self._args.gz_bin, "topic", "-e", "-t", topic]
        min_sample_dt = 0.0 if self._args.sample_rate <= 0.0 else 1.0 / self._args.sample_rate
        last_sample_time = None
        start = time.monotonic()

        try:
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self.error = f"failed to start {' '.join(command)!r}: {exc}"
            self.stop()
            return

        try:
            assert self._process.stdout is not None

            for block in pose_blocks(self._process.stdout):
                if self.stopped:
                    break

                now = time.monotonic()

                if self._args.duration > 0.0 and now - start >= self._args.duration:
                    break

                pose = parse_pose(block, self._args.model_name)

                if pose is None:
                    continue

                if last_sample_time is not None and now - last_sample_time < min_sample_dt:
                    continue

                last_sample_time = now

                with self._lock:
                    self._samples.append((now - start, *pose))

        finally:
            self.stop()

            if self._process is not None:
                try:
                    self._process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=2.0)


def record_trajectory(args):
    recorder = TrajectoryRecorder(args)

    try:
        recorder.run()
    except KeyboardInterrupt:
        recorder.stop()

    return recorder.samples(), recorder.error


def run_live_plot(args):
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

    import matplotlib.pyplot as plt

    recorder = TrajectoryRecorder(args)
    thread = threading.Thread(target=recorder.run, daemon=True)
    thread.start()

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], color="#d32f2f", linewidth=2.0)
    current_point, = ax.plot([], [], "o", color="#1565c0", markersize=6)
    start_point, = ax.plot([], [], "o", color="#2e7d32", markersize=6)
    ax.set_title(f"{args.model_name} live trajectory")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, color="#e0e0e0")
    ax.set_aspect("equal", adjustable="datalim")

    refresh_dt = 1.0 / max(args.refresh_rate, 1.0)
    last_count = 0

    try:
        while thread.is_alive() and plt.fignum_exists(fig.number):
            samples = recorder.samples()

            if len(samples) != last_count:
                xs = [sample[1] for sample in samples]
                ys = [sample[2] for sample in samples]
                line.set_data(xs, ys)

                if samples:
                    start_point.set_data([xs[0]], [ys[0]])
                    current_point.set_data([xs[-1]], [ys[-1]])

                if len(samples) >= 2:
                    ax.relim()
                    ax.autoscale_view()

                ax.set_title(f"{args.model_name} live trajectory | samples: {len(samples)}")
                fig.canvas.draw_idle()
                last_count = len(samples)

            plt.pause(refresh_dt)

    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        thread.join(timeout=3.0)
        plt.ioff()

    return recorder.samples(), recorder.error


def write_csv(path, samples):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(("time_s", "x_m", "y_m", "z_m"))
        writer.writerows(samples)


def nice_bounds(values):
    lower = min(values)
    upper = max(values)

    if math.isclose(lower, upper):
        lower -= 1.0
        upper += 1.0

    margin = (upper - lower) * 0.05
    return lower - margin, upper + margin


def point_to_svg(x, y, bounds, geometry):
    x_min, x_max, y_min, y_max = bounds
    width, height, pad = geometry
    plot_w = width - 2.0 * pad
    plot_h = height - 2.0 * pad
    scale = min(plot_w / (x_max - x_min), plot_h / (y_max - y_min))
    used_w = (x_max - x_min) * scale
    used_h = (y_max - y_min) * scale
    origin_x = pad + (plot_w - used_w) * 0.5
    origin_y = pad + (plot_h - used_h) * 0.5
    return origin_x + (x - x_min) * scale, origin_y + used_h - (y - y_min) * scale


def render_svg(path, samples, model_name):
    width = 1000
    height = 800
    pad = 72
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    x_min, x_max = nice_bounds(xs)
    y_min, y_max = nice_bounds(ys)
    bounds = (x_min, x_max, y_min, y_max)
    geometry = (width, height, pad)
    points = [point_to_svg(x, y, bounds, geometry) for _t, x, y, _z in samples]
    polyline = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    start_x, start_y = points[0]
    end_x, end_y = points[-1]
    title = html.escape(f"{model_name} trajectory")

    grid_lines = []
    tick_labels = []

    for i in range(6):
        ratio = i / 5.0
        x_value = x_min + (x_max - x_min) * ratio
        y_value = y_min + (y_max - y_min) * ratio
        x_tick, y_bottom = point_to_svg(x_value, y_min, bounds, geometry)
        x_left, y_tick = point_to_svg(x_min, y_value, bounds, geometry)
        _x_right, y_tick_end = point_to_svg(x_max, y_value, bounds, geometry)
        x_tick_end, _y_top = point_to_svg(x_value, y_max, bounds, geometry)

        grid_lines.append(f'<line x1="{x_tick:.2f}" y1="{y_bottom:.2f}" x2="{x_tick_end:.2f}" y2="{_y_top:.2f}" />')
        grid_lines.append(f'<line x1="{x_left:.2f}" y1="{y_tick:.2f}" x2="{_x_right:.2f}" y2="{y_tick_end:.2f}" />')
        tick_labels.append(f'<text x="{x_tick:.2f}" y="{height - 24}" text-anchor="middle">{x_value:.1f}</text>')
        tick_labels.append(f'<text x="48" y="{y_tick + 4:.2f}" text-anchor="end">{y_value:.1f}</text>')

    duration = samples[-1][0] - samples[0][0]
    distance = sum(
        math.hypot(samples[i][1] - samples[i - 1][1], samples[i][2] - samples[i - 1][2])
        for i in range(1, len(samples))
    )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#ffffff"/>
  <style>
    text {{ font-family: Arial, sans-serif; fill: #202124; font-size: 16px; }}
    .small {{ fill: #5f6368; font-size: 13px; }}
    .grid line {{ stroke: #e0e0e0; stroke-width: 1; }}
    .axis {{ stroke: #8a8f98; stroke-width: 1.5; fill: none; }}
  </style>
  <text x="{width / 2:.0f}" y="34" text-anchor="middle" font-size="24">{title}</text>
  <text x="{width / 2:.0f}" y="58" text-anchor="middle" class="small">samples: {len(samples)} | duration: {duration:.1f}s | xy distance: {distance:.1f}m</text>
  <g class="grid">
    {"".join(grid_lines)}
  </g>
  <rect x="{pad}" y="{pad}" width="{width - 2 * pad}" height="{height - 2 * pad}" class="axis"/>
  <polyline points="{polyline}" fill="none" stroke="#d32f2f" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>
  <circle cx="{start_x:.2f}" cy="{start_y:.2f}" r="6" fill="#2e7d32"/>
  <circle cx="{end_x:.2f}" cy="{end_y:.2f}" r="6" fill="#1565c0"/>
  <text x="{start_x + 10:.2f}" y="{start_y - 10:.2f}" class="small">start</text>
  <text x="{end_x + 10:.2f}" y="{end_y - 10:.2f}" class="small">end</text>
  {"".join(tick_labels)}
  <text x="{width / 2:.0f}" y="{height - 4}" text-anchor="middle" class="small">x / m</text>
  <text x="18" y="{height / 2:.0f}" text-anchor="middle" class="small" transform="rotate(-90 18 {height / 2:.0f})">y / m</text>
</svg>
"""

    with open(path, "w", encoding="utf-8") as svg_file:
        svg_file.write(svg)


def main():
    args = parse_args()

    if args.no_live:
        samples, error = record_trajectory(args)
    else:
        samples, error = run_live_plot(args)

    if error is not None:
        print(error, file=sys.stderr)
        return 1

    if len(samples) < 2:
        print(
            f"Only recorded {len(samples)} sample(s). Check that Gazebo is running and model '{args.model_name}' exists.",
            file=sys.stderr,
        )
        return 1

    write_csv(args.csv, samples)
    render_svg(args.svg, samples, args.model_name)
    print(f"Wrote {len(samples)} samples to {args.csv}")
    print(f"Wrote trajectory plot to {args.svg}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
