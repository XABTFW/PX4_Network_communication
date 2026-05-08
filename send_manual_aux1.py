#!/usr/bin/env python3
"""Send MAVLink GCS heartbeat and MANUAL_CONTROL aux1 to PX4 SITL on 14540."""

import argparse
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


DEFAULT_CONNECT = "udp:127.0.0.1:14540"
FALLBACK_CONNECT = "udpout:127.0.0.1:14540"
RATE_HZ = 5.0
HEARTBEAT_RATE_HZ = 1.0
HEARTBEAT_TIMEOUT_S = 2.0
SOURCE_SYSTEM = 246
SOURCE_COMPONENT = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
TARGET_SYSTEM = 1
THROTTLE_CENTER = 500
AUX1_FULL_SCALE = 1000
AUX1_EXTENSION_BIT = 1 << 2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send GCS heartbeat and MANUAL_CONTROL aux1 to PX4 SITL."
    )
    parser.add_argument(
        "--connect",
        default=DEFAULT_CONNECT,
        help="MAVLink output endpoint",
    )
    parser.add_argument("--rate", type=float, default=RATE_HZ, help="send rate in Hz")
    parser.add_argument(
        "--aux1",
        type=int,
        choices=(0, 1),
        default=1,
        help="AUX1 switch value to send",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=HEARTBEAT_TIMEOUT_S,
        help="seconds to wait for PX4 heartbeat before trying udpout fallback",
    )
    return parser.parse_args()


def open_connection(connect):
    try:
        return mavutil.mavlink_connection(
            connect,
            source_system=SOURCE_SYSTEM,
            source_component=SOURCE_COMPONENT,
        )
    except OSError as exc:
        raise RuntimeError(f"failed to open {connect}: {exc}") from exc


def wait_for_px4_heartbeat(master, timeout_s):
    deadline_s = time.monotonic() + timeout_s

    while time.monotonic() < deadline_s:
        send_heartbeat(master)
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)

        if msg is None:
            continue

        if msg.get_srcSystem() == SOURCE_SYSTEM:
            continue

        if getattr(msg, "type", None) == mavutil.mavlink.MAV_TYPE_GCS:
            continue

        return msg.get_srcSystem()

    return 0


def open_default_connection(connect, heartbeat_timeout_s):
    try:
        master = open_connection(connect)
    except RuntimeError as exc:
        if connect != DEFAULT_CONNECT:
            print(str(exc), file=sys.stderr)
            raise SystemExit(1) from exc

        print(f"{exc}; trying {FALLBACK_CONNECT}", file=sys.stderr)
        master = None

    if master is not None and connect.startswith("udp:"):
        target_system = wait_for_px4_heartbeat(master, heartbeat_timeout_s)

        if target_system > 0:
            return master, connect, target_system

        print(f"no PX4 heartbeat on {connect}; trying {FALLBACK_CONNECT}", file=sys.stderr)
        master.close()

    elif master is not None:
        return master, connect, TARGET_SYSTEM

    try:
        return open_connection(FALLBACK_CONNECT), FALLBACK_CONNECT, TARGET_SYSTEM
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from exc


def send_manual_control(master, target_system, aux1_value):
    master.mav.manual_control_send(
        target_system,
        0,
        0,
        THROTTLE_CENTER,
        0,
        0,
        0,
        AUX1_EXTENSION_BIT,
        0,
        0,
        aux1_value,
        0,
        0,
        0,
        0,
        0,
        force_mavlink1=False,
    )


def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def main():
    args = parse_args()

    if args.rate <= 0.0:
        print("--rate must be greater than 0", file=sys.stderr)
        return 1

    master, active_connect, target_system = open_default_connection(args.connect, args.heartbeat_timeout)
    aux1_mavlink = AUX1_FULL_SCALE if args.aux1 == 1 else 0
    period_s = 1.0 / args.rate
    heartbeat_period_s = 1.0 / HEARTBEAT_RATE_HZ
    next_send_s = time.monotonic()
    next_heartbeat_s = next_send_s
    sent_count = 0
    heartbeat_count = 0

    print(
        f"connect={active_connect} "
        f"from sysid={SOURCE_SYSTEM} compid={SOURCE_COMPONENT} "
        f"target_system={target_system} rate={args.rate:.1f}Hz",
        flush=True,
    )
    print(
        f"x=0 y=0 z={THROTTLE_CENTER} r=0 aux1={aux1_mavlink} "
        f"enabled_extensions=0x{AUX1_EXTENSION_BIT:02x}",
        flush=True,
    )

    try:
        while True:
            now_s = time.monotonic()

            if now_s >= next_heartbeat_s:
                send_heartbeat(master)
                heartbeat_count += 1
                next_heartbeat_s += heartbeat_period_s

            if now_s >= next_send_s:
                send_manual_control(master, target_system, aux1_mavlink)
                sent_count += 1
                print(
                    f"sent_count={sent_count} aux1={args.aux1} "
                    f"enabled_extensions=0x{AUX1_EXTENSION_BIT:02x} heartbeat_count={heartbeat_count}",
                    flush=True,
                )
                next_send_s += period_s

                if next_send_s < time.monotonic() - period_s:
                    next_send_s = time.monotonic()

            sleep_until_s = min(next_send_s, next_heartbeat_s)

            if sleep_until_s > time.monotonic():
                time.sleep(sleep_until_s - time.monotonic())

    except KeyboardInterrupt:
        print("\nstopped", flush=True)

    finally:
        master.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
