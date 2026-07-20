#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
radar_to_mavlink_bridge.py

运行在"本机 A"(跑 QGC + PX4 的电脑)上的转换桥, 模拟将来 QGC 侧要做的事:

  雷达 --(网线, UDP, 反无雷达协议帧)--> [本脚本] 解码 --> 编码成 MAVLink UAV_INFO
       --> 发给 PX4 的 MAVLink 端口 --> PX4 handle_message_uav_info
       --> follower_info --> target_frame_udp `mavdump on` 打印协议十六进制

即: 雷达发的是"反无雷达"协议帧(不是 MAVLink), 转成 MAVLink 的工作在本机完成。

数据流:
  [电脑B 雷达/模拟]  target_frame_sim.py 发协议帧
        │ UDP  --listen-ip:--listen-port
        ▼
  [本机A 本脚本]  解码协议帧 -> 每个目标编码成 UAV_INFO
        │ UDP  --px4-ip:--px4-port  (PX4 绑定的 MAVLink 端口, mavlink status 可查)
        ▼
  [本机A PX4]  target_frame_udp mavdump on -> MAV HEX / MAV DEC

零第三方依赖, 只用标准库。

用法示例:
  # 本机A, 假设雷达/模拟端发到本机 50000, PX4 GCS 链路端口 18570
  python3 radar_to_mavlink_bridge.py --listen-port 50000 --px4-ip 127.0.0.1 --px4-port 18570

  # 电脑B 用第一版脚本模拟雷达发协议帧到本机A:
  python3 target_frame_sim.py --target-ip <本机A的IP> --target-port 50000 --local-port 50000
"""

import argparse
import math
import socket
import struct
import sys
import threading

# ===========================================================================
# 第一部分: 反无雷达协议解码 (与 target_frame_codec / target_frame_sim.py 一致)
# ===========================================================================
FRAME_HEADER = 0xFD
PAYLOAD_LENGTH_FIELD = 0x10
MESSAGE_TYPE = 0x01
RESERVED = 0x01
IDENT = (0xC5, 0xCE, 0xC2)
HEADER_SIZE = 15
TARGET_SIZE = 50
CRC_SIZE = 2
MAX_DATAGRAM_SIZE = 1472
MAX_TARGETS = (MAX_DATAGRAM_SIZE - HEADER_SIZE - CRC_SIZE) // TARGET_SIZE

# 50 字节目标体: 3xint32, 2xuint16, 5xfloat, 2xuint8, 3xfloat (小端)
_TARGET_STRUCT = struct.Struct("<iiiHHfffffBBfff")


def _crc_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _crc_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def _crc_x25(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return (crc ^ 0xFFFF) & 0xFFFF


_CRC = {"none": lambda d: 0, "modbus": _crc_modbus,
        "ccitt": _crc_ccitt_false, "ccitt-false": _crc_ccitt_false, "x25": _crc_x25}


def frame_size(n):
    return HEADER_SIZE + n * TARGET_SIZE + CRC_SIZE


def decode_radar_frame(buf, crc_mode):
    """解码一个协议帧, 返回 [target_dict, ...]。校验失败抛 ValueError。"""
    length = len(buf)
    if length < frame_size(1):
        raise ValueError("帧太短")
    if buf[0] != FRAME_HEADER or buf[3] != MESSAGE_TYPE or buf[4] != RESERVED:
        raise ValueError("帧头错误")
    if (buf[5], buf[6], buf[7]) != IDENT:
        raise ValueError("标识符错误")
    if buf[1] != PAYLOAD_LENGTH_FIELD:
        raise ValueError("载荷长度字段错误")
    count = buf[14]
    if count == 0 or count > MAX_TARGETS:
        raise ValueError("目标数量错误")
    expected = frame_size(count)
    if length != expected or struct.unpack_from("<H", buf, 12)[0] != expected:
        raise ValueError("长度不匹配")
    if crc_mode != "none":
        got = struct.unpack_from("<H", buf, length - CRC_SIZE)[0]
        calc = _CRC[crc_mode](buf[:length - CRC_SIZE])
        if got != calc:
            raise ValueError("CRC 错误 (帧内=0x%04X 计算=0x%04X)" % (got, calc))

    targets = []
    off = HEADER_SIZE
    for _ in range(count):
        v = _TARGET_STRUCT.unpack_from(buf, off)
        targets.append({
            "longitude_e7": v[0], "latitude_e7": v[1], "altitude_mm": v[2],
            "target_id": v[3], "delete_flag": v[4], "speed_m_s": v[5],
            "altitude_m": v[9], "track_type": v[10], "target_attribute": v[11],
            "vx_m_s": v[12], "vy_m_s": v[13], "vz_m_s": v[14],
        })
        off += TARGET_SIZE
    return targets


# ===========================================================================
# 第二部分: MAVLink v2 UAV_INFO 编码 (来自 common.xml id=12921, crc_extra=100)
# 载荷线序: mavid(u32) group_id(u32) lat(f) lon(f) yaw(f) yaw_speed(f)
#          rel_alt(f) vx(f) vy(f) vz(f) land(u32) is_leader(u8)  = 45 字节
# ===========================================================================
MSG_ID_UAV_INFO = 12921
CRC_EXTRA_UAV_INFO = 100
_UAV_INFO_STRUCT = struct.Struct("<IIffffffffIB")


def _mav_crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def _mavlink_crc(data, crc_extra):
    crc = 0xFFFF
    for b in data:
        crc = _mav_crc_accumulate(b, crc)
    return _mav_crc_accumulate(crc_extra, crc)


def build_uav_info_frame(seq, sysid, compid, mavid, group_id, is_leader,
                         lat, lon, yaw, yaw_speed, rel_alt, vx, vy, vz, land):
    payload = _UAV_INFO_STRUCT.pack(
        mavid & 0xFFFFFFFF, group_id & 0xFFFFFFFF,
        lat, lon, yaw, yaw_speed, rel_alt, vx, vy, vz,
        land & 0xFFFFFFFF, is_leader & 0xFF)
    core = struct.pack("<BBBBBB", len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF)
    core += struct.pack("<I", MSG_ID_UAV_INFO)[:3]
    core += payload
    crc = _mavlink_crc(core, CRC_EXTRA_UAV_INFO)
    return b"\xFD" + core + struct.pack("<H", crc)


# ===========================================================================
# 打印
# ===========================================================================
_print_lock = threading.Lock()


def log(*args):
    with _print_lock:
        print(*args)
        sys.stdout.flush()


def hexdump(tag, buf):
    lines = ["%s len=%d" % (tag, len(buf))]
    for off in range(0, len(buf), 16):
        chunk = buf[off:off + 16]
        lines.append("%s +%03d: %s" % (tag, off, " ".join("%02X" % b for b in chunk)))
    return "\n".join(lines)


# ===========================================================================
# 桥主体
# ===========================================================================
def main():
    p = argparse.ArgumentParser(
        description="雷达协议帧 -> MAVLink UAV_INFO 转换桥 (运行在 QGC+PX4 那台电脑上)")
    p.add_argument("--listen-ip", default="0.0.0.0", help="监听地址 (默认 0.0.0.0, 收所有网卡)")
    p.add_argument("--listen-port", type=int, default=50000, help="监听雷达协议帧的 UDP 端口 (默认 50000)")
    p.add_argument("--px4-ip", default="127.0.0.1", help="PX4 所在 IP (默认本机 127.0.0.1)")
    p.add_argument("--px4-port", type=int, default=18570,
                   help="PX4 绑定的 MAVLink UDP 端口 (SITL 实例0 GCS=18570, Offboard=14580; mavlink status 可查)")
    p.add_argument("--crc", default="modbus",
                   choices=["none", "modbus", "ccitt", "ccitt-false", "x25"],
                   help="雷达协议 CRC 模式, 必须与雷达/发送端一致 (默认 modbus)")
    p.add_argument("--sysid", type=int, default=200, help="转出的 MAVLink 系统 ID (默认 200)")
    p.add_argument("--compid", type=int, default=190, help="转出的组件 ID (默认 190)")
    p.add_argument("--is-leader", type=int, choices=[0, 1], default=0,
                   help="UAV_INFO 的 is_leader (默认 0=从机, 使 follower_info.mavid=雷达目标id)")
    p.add_argument("--group-id", type=int, default=0, help="group_id 字段 (默认 0)")
    p.add_argument("--verbose", action="store_true", help="打印收发帧的十六进制")
    args = p.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        rx.bind((args.listen_ip, args.listen_port))
    except OSError as exc:
        log("绑定监听端口 %d 失败: %s" % (args.listen_port, exc))
        sys.exit(1)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    px4_addr = (args.px4_ip, args.px4_port)

    log("radar_to_mavlink_bridge 已启动")
    log("  监听雷达协议帧: %s:%d  (CRC=%s)" % (args.listen_ip, args.listen_port, args.crc))
    log("  转发 UAV_INFO 到 PX4: %s:%d  (sysid=%d compid=%d is_leader=%d)" % (
        px4_addr[0], px4_addr[1], args.sysid, args.compid, args.is_leader))
    log("  在 PX4 shell 执行 `target_frame_udp mavdump on` 查看编码出的协议十六进制。\n")

    seq = 0
    rx_frames = 0
    tx_msgs = 0
    dropped = 0

    while True:
        try:
            data, src = rx.recvfrom(MAX_DATAGRAM_SIZE)
        except KeyboardInterrupt:
            log("\n退出")
            break
        except OSError as exc:
            log("接收错误: %s" % exc)
            break

        if not data:
            continue

        try:
            targets = decode_radar_frame(data, args.crc)
        except ValueError as exc:
            dropped += 1
            log("丢弃来自 %s:%d 的帧: %s" % (src[0], src[1], exc))
            continue

        rx_frames += 1
        log("---- 收到雷达帧 from %s:%d, 目标数=%d ----" % (src[0], src[1], len(targets)))
        if args.verbose:
            log(hexdump("RADAR RX", data))

        for t in targets:
            if t["delete_flag"] != 0:
                log("  目标 id=%d 为删除记录, 跳过" % t["target_id"])
                continue

            lat = t["latitude_e7"] * 1e-7
            lon = t["longitude_e7"] * 1e-7
            alt = t["altitude_m"]
            vx, vy, vz = t["vx_m_s"], t["vy_m_s"], t["vz_m_s"]

            frame = build_uav_info_frame(
                seq, args.sysid, args.compid,
                mavid=t["target_id"], group_id=args.group_id, is_leader=args.is_leader,
                lat=lat, lon=lon, yaw=0.0, yaw_speed=0.0, rel_alt=alt,
                vx=vx, vy=vy, vz=vz, land=0)
            seq = (seq + 1) & 0xFF

            try:
                tx.sendto(frame, px4_addr)
                tx_msgs += 1
            except OSError as exc:
                log("  发送到 PX4 失败: %s" % exc)
                continue

            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            log("  -> UAV_INFO id=%d lon=%.7f lat=%.7f alt=%.3f speed=%.3f vx=%.3f vy=%.3f vz=%.3f" % (
                t["target_id"], lon, lat, alt, speed, vx, vy, vz))
            if args.verbose:
                log(hexdump("  MAVLINK TX", frame))

        log("  统计: rx_frames=%d tx_msgs=%d dropped=%d\n" % (rx_frames, tx_msgs, dropped))

    rx.close()
    tx.close()


if __name__ == "__main__":
    main()
