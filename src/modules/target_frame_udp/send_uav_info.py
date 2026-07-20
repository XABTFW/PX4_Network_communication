#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_uav_info.py

方案A 的发送端脚本 (零第三方依赖, 只用 Python 标准库)。

用途:
  手动输入十进制目标数据 (格式与 target_frame_udp 的 senddec 一致),
  脚本把它编码成 MAVLink v2 的 UAV_INFO 消息, 通过网线 (UDP) 发出。

数据链路:
  [本脚本] 输入十进制 -> 编码成 MAVLink UAV_INFO
        | UDP (网线)
        v
  [本机 QGC] 开启 MAVLink Forwarding, 透传 UAV_INFO (不做转换, 只转发)
        v
  [本机 PX4] handle_message_uav_info() -> 发布 follower_info
        v
  [target_frame_udp] mavdump on -> 编码成协议帧 -> 打印十六进制 (MAV HEX / MAV DEC)

关键说明:
  * UAV_INFO 落到 follower_info 的条件 (见 mavlink_receiver.cpp handle_message_uav_info):
      - is_leader=0 (从机)          -> follower_info.mavid = mavid
      - is_leader=1 且 mavid>=100   -> follower_info.mavid = mavid-100
    本脚本默认 is_leader=0, 直接用输入的 id 作为 mavid, 最简单直观。
  * 目标 id 不能等于本机 PX4 的 MAV_SYS_ID, 也不能为 0 (模块会跳过)。
  * UAV_INFO 的 lat/lon/rel_alt/vx/vy/vz 都是 32 位 float, 因此经纬度低位
    会有约 1e-5 度 (~1m) 的精度损失, 与直接 senddec(double) 不会逐字节一致。

拓扑 (对面电脑只跑本脚本, 本机跑 QGC+PX4):
  [对面电脑 B] send_uav_info.py  --(网线 UDP)-->  [本机 A] PX4 的 MAVLink 端口
  QGC 在本机 A 照常运行做监控, 但不在数据入口链路上 (见下方说明)。

为什么直接发给 PX4, 而不是经 QGC 转发:
  QGC 的 MAVLink Forwarding 方向是 "飞机 -> 外部", 不会把外部收到的消息中继
  给飞控; 且 QGC 只转发能通过 CRC 校验的消息, 而本 QGC 未必带 UAV_INFO(12921)
  的 dialect。因此脚本应直接发到 PX4 绑定的 MAVLink UDP 端口。

目标端口 (PX4 SITL 实例0, 用 `mavlink status` 可确认):
  --target-port 18570   GCS 链路 (PX4 绑定的本地端口), 推荐
  --target-port 14580   Offboard/API 链路 (PX4 绑定的本地端口), 亦可
  注意: 14540 是 PX4 "往外发" 的对端端口, 发到它到不了 PX4。

用法示例:
  python3 send_uav_info.py --target-ip 192.168.50.11 --target-port 18570
  然后在提示符输入:  100 116.3912345 39.9071234 123.45 1.25 -2.5 0.75
"""

import argparse
import socket
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# UAV_INFO 消息常量 (来自 common.xml id=12921, mavgen 计算的 crc_extra=100)
# 载荷字段的 MAVLink 线序 (按类型大小降序, 同大小按声明序):
#   mavid(u32) group_id(u32) lat(f) lon(f) yaw(f) yaw_speed(f) rel_alt(f)
#   vx(f) vy(f) vz(f) land(u32) is_leader(u8)   共 45 字节
# ---------------------------------------------------------------------------
MSG_ID_UAV_INFO = 12921
CRC_EXTRA_UAV_INFO = 100
_PAYLOAD_STRUCT = struct.Struct("<IIffffffffIB")


# ---------------------------------------------------------------------------
# MAVLink CRC (CRC-16/MCRF4XX: init 0xFFFF, 反射多项式 0x8408, 无最终异或)
# ---------------------------------------------------------------------------
def _crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def mavlink_crc(data, crc_extra):
    crc = 0xFFFF
    for b in data:
        crc = _crc_accumulate(b, crc)
    return _crc_accumulate(crc_extra, crc)


def build_uav_info_frame(seq, sysid, compid, mavid, group_id, is_leader,
                         lat, lon, yaw, yaw_speed, rel_alt, vx, vy, vz, land):
    """构造一条完整的 MAVLink v2 UAV_INFO 帧 (与 pymavlink 输出逐字节一致)。"""
    payload = _PAYLOAD_STRUCT.pack(
        mavid & 0xFFFFFFFF, group_id & 0xFFFFFFFF,
        lat, lon, yaw, yaw_speed, rel_alt, vx, vy, vz,
        land & 0xFFFFFFFF, is_leader & 0xFF)

    # v2 头 (不含 STX): len, incompat_flags, compat_flags, seq, sysid, compid, msgid(3B)
    core = struct.pack("<BBBBBB", len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF)
    core += struct.pack("<I", MSG_ID_UAV_INFO)[:3]
    core += payload

    crc = mavlink_crc(core, CRC_EXTRA_UAV_INFO)
    return b"\xFD" + core + struct.pack("<H", crc)


# ---------------------------------------------------------------------------
# 打印
# ---------------------------------------------------------------------------
_print_lock = threading.Lock()


def log(*args):
    with _print_lock:
        print(*args)
        sys.stdout.flush()


def dump_hex(tag, frame):
    lines = ["%s MAVLINK len=%d" % (tag, len(frame))]
    for off in range(0, len(frame), 16):
        chunk = frame[off:off + 16]
        lines.append("%s +%03d: %s" % (tag, off, " ".join("%02X" % b for b in chunk)))
    log("\n".join(lines))


HELP_TEXT = """\
可用命令:
  <id> <lon> <lat> <alt> <vx> <vy> <vz>    发送一条 UAV_INFO (等同 senddec 输入)
  send <id> <lon> <lat> <alt> <vx> <vy> <vz>
  auto <hz>                                以指定频率自动重发上一条
  auto off                                 停止自动重发
  hex on|off                               是否打印发出的 MAVLink 帧十六进制
  status                                   显示统计
  help                                     显示帮助
  quit / exit                              退出

字段: id=目标编号(!=0 且 != 本机MAV_SYS_ID)  lon=经度(deg)  lat=纬度(deg)
      alt=高度(m)  vx/vy/vz=NED速度(m/s, vz正向下)
示例:
  100 116.3912345 39.9071234 123.45 1.25 -2.5 0.75
"""


class Sender:
    def __init__(self, sock, target_addr, sysid, compid, is_leader, group_id, show_hex):
        self._sock = sock
        self._target = target_addr
        self._sysid = sysid
        self._compid = compid
        self._is_leader = is_leader
        self._group_id = group_id
        self._show_hex = show_hex
        self._seq = 0
        self.tx = 0
        self._last = None
        self._auto_thread = None
        self._auto_stop = threading.Event()

    def _next_seq(self):
        s = self._seq & 0xFF
        self._seq = (self._seq + 1) & 0xFF
        return s

    def _send(self, mavid, lon, lat, alt, vx, vy, vz):
        seq = self._next_seq()
        frame = build_uav_info_frame(
            seq, self._sysid, self._compid,
            mavid=mavid, group_id=self._group_id, is_leader=self._is_leader,
            lat=lat, lon=lon, yaw=0.0, yaw_speed=0.0, rel_alt=alt,
            vx=vx, vy=vy, vz=vz, land=0)
        self._sock.sendto(frame, self._target)
        self.tx += 1
        speed = (vx * vx + vy * vy + vz * vz) ** 0.5
        log("\n---- TX UAV_INFO -> %s:%d (seq=%d) ----" % (self._target[0], self._target[1], seq))
        log("TX DEC id=%d lon=%.7f lat=%.7f alt=%.3f speed=%.3f vx=%.3f vy=%.3f vz=%.3f is_leader=%d" % (
            mavid, lon, lat, alt, speed, vx, vy, vz, self._is_leader))
        if self._show_hex:
            dump_hex("TX", frame)
        self._last = (mavid, lon, lat, alt, vx, vy, vz)

    def send_decimal(self, fields):
        if len(fields) != 7:
            raise ValueError("需要 7 个值: <id> <lon> <lat> <alt> <vx> <vy> <vz>")
        mavid = int(fields[0], 0)
        if mavid == 0:
            raise ValueError("id 不能为 0")
        lon = float(fields[1]); lat = float(fields[2]); alt = float(fields[3])
        vx = float(fields[4]); vy = float(fields[5]); vz = float(fields[6])
        if not (-180.0 <= lon <= 180.0):
            raise ValueError("经度必须在 -180..180")
        if not (-90.0 <= lat <= 90.0):
            raise ValueError("纬度必须在 -90..90")
        self._send(mavid, lon, lat, alt, vx, vy, vz)

    def set_hex(self, on):
        self._show_hex = on

    def start_auto(self, hz):
        self.stop_auto()
        if self._last is None:
            raise ValueError("还没有可重发的数据, 先手动发送一次")
        hz = max(0.1, min(50.0, hz))
        interval = 1.0 / hz
        self._auto_stop.clear()

        def loop():
            while not self._auto_stop.wait(interval):
                try:
                    self._send(*self._last)
                except OSError as exc:
                    log("自动发送失败: %s" % exc)
                    break

        self._auto_thread = threading.Thread(target=loop, daemon=True)
        self._auto_thread.start()
        log("自动重发已开启: %.1f Hz" % hz)

    def stop_auto(self):
        if self._auto_thread and self._auto_thread.is_alive():
            self._auto_stop.set()
            self._auto_thread.join(timeout=1.0)
        self._auto_thread = None


def run_repl(sender, args):
    log("send_uav_info 已启动")
    log("  目标: %s:%d   sysid=%d compid=%d is_leader=%d group_id=%d" % (
        args.target_ip, args.target_port, args.sysid, args.compid, args.is_leader, args.group_id))
    log("  提示: 目标 id 不要等于本机 PX4 的 MAV_SYS_ID; 端口需是 PX4 绑定的端口 "
        "(mavlink status 查看); 在 PX4 shell 执行 `target_frame_udp mavdump on` 看十六进制。")
    log("")
    log(HELP_TEXT)

    while True:
        try:
            line = input("uav> ").strip()
        except (EOFError, KeyboardInterrupt):
            log("\n退出")
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd in ("quit", "exit"):
                break
            elif cmd == "help":
                log(HELP_TEXT)
            elif cmd == "status":
                log("TX=%d" % sender.tx)
            elif cmd == "send":
                sender.send_decimal(parts[1:])
            elif cmd == "hex":
                if len(parts) >= 2 and parts[1].lower() in ("on", "off"):
                    sender.set_hex(parts[1].lower() == "on")
                    log("hex 打印: %s" % parts[1].lower())
                else:
                    log("用法: hex on|off")
            elif cmd == "auto":
                if len(parts) >= 2 and parts[1].lower() == "off":
                    sender.stop_auto(); log("自动重发已停止")
                elif len(parts) >= 2:
                    sender.start_auto(float(parts[1]))
                else:
                    log("用法: auto <hz> | auto off")
            elif len(parts) == 7:
                sender.send_decimal(parts)
            else:
                log("无法识别的命令, 输入 help 查看用法")
        except (ValueError, OSError) as exc:
            log("错误: %s" % exc)

    sender.stop_auto()


def main():
    p = argparse.ArgumentParser(description="发送 MAVLink UAV_INFO (方案A 发送端, 零依赖)")
    p.add_argument("--target-ip", required=True, help="接收方 IP (本机 QGC 或 PX4 的地址)")
    p.add_argument("--target-port", type=int, default=18570,
                   help="PX4 绑定的 MAVLink UDP 端口 (SITL 实例0: GCS=18570, Offboard=14580; 用 mavlink status 确认)")
    p.add_argument("--sysid", type=int, default=200, help="本脚本模拟的 MAVLink 系统 ID (默认 200)")
    p.add_argument("--compid", type=int, default=190, help="本脚本模拟的组件 ID (默认 190)")
    p.add_argument("--is-leader", type=int, choices=[0, 1], default=0,
                   help="UAV_INFO 的 is_leader 字段 (默认 0=从机, 使 follower_info.mavid=id)")
    p.add_argument("--group-id", type=int, default=0, help="group_id 字段 (默认 0)")
    p.add_argument("--bind-port", type=int, default=0,
                   help="本地发送源端口 (默认 0=系统随机; 需要固定源端口时指定)")
    p.add_argument("--hex", action="store_true", help="启动即打印发出的 MAVLink 帧十六进制")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if args.bind_port:
        sock.bind(("0.0.0.0", args.bind_port))

    sender = Sender(sock, (args.target_ip, args.target_port),
                    args.sysid, args.compid, args.is_leader, args.group_id, args.hex)
    try:
        run_repl(sender, args)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
