#!/usr/bin/env python3
"""Send fake DYT telemetry frames to the PX4 dyt_gimbal driver."""

import argparse
import math
import os
import struct
import sys
import time
import tty


FRAME_LEN = 32
SYNC_1 = 0xEE
SYNC_2_TELEMETRY = 0x16

TRACKING_STATE_SEARCH = "SEARCH"
TRACKING_STATE_LOCKED = "LOCKED"

LOS_SCALE_DEG = 0.05
GIMBAL_SCALE_DEG = 0.01
SWEEP_AMPLITUDE_DEG = 5.0
SWEEP_PERIOD_S = 4.0
LOST_RELOCK_LOST_START_S = 5.0
LOST_RELOCK_RELOCK_START_S = 30.0
LOST_RELOCK_CYCLE_LOCKED_S = 5.0
LOST_RELOCK_CYCLE_LOST_S = 10.0
LOST_RELOCK_CYCLE_RELOCKED_S = 10.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send fake DYT table-2 telemetry frames to a PX4 virtual serial port."
    )
    parser.add_argument("--port", default="/tmp/dyt_cam_0", help="serial port to write")
    parser.add_argument("--rate", type=float, default=20.0, help="send rate in Hz")
    parser.add_argument(
        "--lost-start",
        type=float,
        default=LOST_RELOCK_LOST_START_S,
        help="lost_relock locked-to-lost transition time in seconds",
    )
    parser.add_argument(
        "--relock-start",
        type=float,
        default=LOST_RELOCK_RELOCK_START_S,
        help="lost_relock lost-to-relocked transition time in seconds",
    )
    parser.add_argument(
        "--cycle",
        action="store_true",
        help="cycle lost_relock through locked/lost/relocked phases",
    )
    parser.add_argument(
        "--mode",
        choices=("center", "right", "left", "up", "down", "lost", "sweep", "lost_relock"),
        default="center",
        help="fake target mode",
    )
    return parser.parse_args()


def angle_to_raw(angle_deg, scale_deg):
    raw = int(round(angle_deg / scale_deg))
    return max(-32768, min(32767, raw))


def put_s16_le(frame, offset, value):
    struct.pack_into("<h", frame, offset, value)


def checksum(frame_without_checksum):
    return sum(frame_without_checksum) & 0xFF


def target_valid_for_state(tracking_state):
    return tracking_state == TRACKING_STATE_LOCKED


def lost_relock_phase(elapsed_s, lost_start_s, relock_start_s, cycle):
    if cycle:
        cycle_s = LOST_RELOCK_CYCLE_LOCKED_S + LOST_RELOCK_CYCLE_LOST_S + LOST_RELOCK_CYCLE_RELOCKED_S
        cycle_elapsed_s = elapsed_s % cycle_s

        if cycle_elapsed_s < LOST_RELOCK_CYCLE_LOCKED_S:
            return "locked"

        if cycle_elapsed_s < LOST_RELOCK_CYCLE_LOCKED_S + LOST_RELOCK_CYCLE_LOST_S:
            return "lost"

        return "relocked"

    if elapsed_s < lost_start_s:
        return "locked"

    if elapsed_s < relock_start_s:
        return "lost"

    return "relocked"


def mode_phase(mode, elapsed_s, lost_start_s, relock_start_s, cycle):
    if mode == "lost_relock":
        return lost_relock_phase(elapsed_s, lost_start_s, relock_start_s, cycle)

    return mode


def mode_sample(mode, elapsed_s, lost_start_s, relock_start_s, cycle):
    if mode == "center":
        return 0.0, 0.0, TRACKING_STATE_LOCKED

    if mode == "right":
        return 5.0, 0.0, TRACKING_STATE_LOCKED

    if mode == "left":
        return -5.0, 0.0, TRACKING_STATE_LOCKED

    if mode == "up":
        return 0.0, 5.0, TRACKING_STATE_LOCKED

    if mode == "down":
        return 0.0, -5.0, TRACKING_STATE_LOCKED

    if mode == "lost":
        return 0.0, 0.0, TRACKING_STATE_SEARCH

    if mode == "lost_relock":
        if lost_relock_phase(elapsed_s, lost_start_s, relock_start_s, cycle) == "lost":
            return 0.0, 0.0, TRACKING_STATE_SEARCH

        return 0.0, 0.0, TRACKING_STATE_LOCKED

    phase = (elapsed_s % SWEEP_PERIOD_S) / SWEEP_PERIOD_S
    triangle = 4.0 * abs(phase - 0.5) - 1.0
    return SWEEP_AMPLITUDE_DEG * triangle, 0.0, TRACKING_STATE_LOCKED


def build_frame(los_x_deg, los_y_deg, tracking_state, frame_counter):
    frame = bytearray(FRAME_LEN)
    frame[0] = SYNC_1
    frame[1] = SYNC_2_TELEMETRY

    if tracking_state == TRACKING_STATE_LOCKED:
        frame[2] = 1 << 2

    put_s16_le(frame, 6, angle_to_raw(los_x_deg, LOS_SCALE_DEG))
    put_s16_le(frame, 8, angle_to_raw(los_y_deg, LOS_SCALE_DEG))

    put_s16_le(frame, 10, angle_to_raw(0.0, GIMBAL_SCALE_DEG))
    put_s16_le(frame, 12, angle_to_raw(0.0, GIMBAL_SCALE_DEG))
    put_s16_le(frame, 14, angle_to_raw(0.0, GIMBAL_SCALE_DEG))

    if tracking_state == TRACKING_STATE_LOCKED:
        frame[16] = 20
        frame[17] = 20

    frame[28] = 1 << 7

    # The current PX4 parser counts accepted telemetry frames internally. These
    # bytes are not decoded today, but make the fake frame stream easy to inspect.
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


def main():
    args = parse_args()

    if args.rate <= 0.0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 1

    if args.lost_start < 0.0:
        print("--lost-start must be greater than or equal to 0", file=sys.stderr)
        return 1

    if args.relock_start <= args.lost_start:
        print("--relock-start must be greater than --lost-start", file=sys.stderr)
        return 1

    fd = open_port(args.port)
    period_s = 1.0 / args.rate
    start_s = time.monotonic()
    next_send_s = start_s
    last_send_s = None
    frame_counter = 0

    try:
        while True:
            now_s = time.monotonic()

            if now_s < next_send_s:
                time.sleep(next_send_s - now_s)

            elapsed_s = time.monotonic() - start_s
            frame_counter = (frame_counter + 1) & 0xFFFFFFFF
            los_x_deg, los_y_deg, tracking_state = mode_sample(
                args.mode, elapsed_s, args.lost_start, args.relock_start, args.cycle
            )
            phase = mode_phase(args.mode, elapsed_s, args.lost_start, args.relock_start, args.cycle)
            target_valid = target_valid_for_state(tracking_state)
            frame = build_frame(los_x_deg, los_y_deg, tracking_state, frame_counter)
            send_time_s = time.monotonic()
            write_frame_once(fd, frame)
            send_dt_s = send_time_s - last_send_s if last_send_s is not None else float("nan")
            last_send_s = send_time_s

            print(
                f"elapsed_s={elapsed_s:.3f} mode={args.mode} phase={phase} frame_counter={frame_counter} "
                f"target_valid={target_valid} tracking_state={tracking_state} "
                f"send_dt_s={send_dt_s:.4f} period_s={period_s:.4f} "
                f"los_x_deg={los_x_deg:+.2f} los_y_deg={los_y_deg:+.2f}",
                flush=True,
            )

            next_send_s += period_s

            if next_send_s < time.monotonic() - period_s:
                next_send_s = time.monotonic()

    except KeyboardInterrupt:
        print("\nstopped")

    finally:
        os.close(fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
