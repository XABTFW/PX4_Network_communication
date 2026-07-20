#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
target_frame_sim.py

在“没有第二台 PX4/QGC”的电脑上运行，用来替代对端 PX4 的 target_frame_udp 模块。

功能:
  1. 手动输入十进制目标数据 (格式与模块的 senddec 完全一致):
         <id> <lon> <lat> <alt> <vx> <vy> <vz>
  2. 按照 反无雷达 协议编码成 71 字节的目标帧 (与模块 target_frame_codec 完全一致, 8字节时间),
     通过网线 (UDP) 发送给运行 PX4 的本机。
  3. 后台线程实时监听并解码收到的帧, 打印出与 PX4 `dump on` 一致的
     HEX + 十进制 内容, 方便对照校验。

关键网络约束 (与 target_frame_udp.cpp receive_frames 的过滤逻辑对应):
  PX4 模块只接收 “源IP == 它配置的 peer_ip” 且 “源端口 == 它配置的 remote_port(默认50000)”
  的 UDP 包。因此本脚本必须:
    - 绑定本地端口 = PX4 模块的 remote_port (默认 50000), 使发出的源端口匹配;
    - 从 PX4 模块 peer_ip 指向的这台机器的 IP 发出 (即本机的有线网卡 IP)。

对应关系 (参考 README 双机示例):
  PX4(A) 192.168.50.11:  target_frame_udp start -t 192.168.50.12 -l 50000 -o 50000 ...
  本脚本(替代 B) 跑在 192.168.50.12, 发往 192.168.50.11:50000, 本地绑定 50000。

  用法:  python3 target_frame_sim.py --target-ip 192.168.50.11
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------
# 协议常量 (与 target_frame_codec.hpp 保持一致)
# ---------------------------------------------------------------------------
FRAME_HEADER = 0xFD
PAYLOAD_LENGTH_FIELD = 0x10
MESSAGE_TYPE = 0x01
RESERVED = 0x01
IDENT = (0xC5, 0xCE, 0xC2)
HEADER_SIZE = 19  # 8-byte (u64) time field
TARGET_SIZE = 50
CRC_SIZE = 2
MAX_DATAGRAM_SIZE = 1472
MAX_TARGETS = (MAX_DATAGRAM_SIZE - HEADER_SIZE - CRC_SIZE) // TARGET_SIZE

DEFAULT_TRACK_TYPE = 0x20
DEFAULT_TARGET_ATTRIBUTE = 0x22


# ---------------------------------------------------------------------------
# CRC-16 (与 target_frame_codec.cpp 的四种模式逐位对应)
# ---------------------------------------------------------------------------
def crc_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def crc_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def crc_x25(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return (crc ^ 0xFFFF) & 0xFFFF


CRC_FUNCS = {
    "none": lambda d: 0,
    "modbus": crc_modbus,
    "ccitt": crc_ccitt_false,
    "ccitt-false": crc_ccitt_false,
    "x25": crc_x25,
}


def crc16(mode, data):
    return CRC_FUNCS[mode](data)


# ---------------------------------------------------------------------------
# 目标结构 (字段顺序与 codec 一致)
# ---------------------------------------------------------------------------
class Target:
    __slots__ = (
        "longitude_e7", "latitude_e7", "altitude_mm", "target_id", "delete_flag",
        "speed_m_s", "distance_m", "azimuth_rad", "elevation_rad", "altitude_m",
        "track_type", "target_attribute", "vx_m_s", "vy_m_s", "vz_m_s",
    )

    def __init__(self):
        self.longitude_e7 = 0
        self.latitude_e7 = 0
        self.altitude_mm = 0
        self.target_id = 0
        self.delete_flag = 0
        self.speed_m_s = 0.0
        self.distance_m = 0.0
        self.azimuth_rad = 0.0
        self.elevation_rad = 0.0
        self.altitude_m = 0.0
        self.track_type = DEFAULT_TRACK_TYPE
        self.target_attribute = DEFAULT_TARGET_ATTRIBUTE
        self.vx_m_s = 0.0
        self.vy_m_s = 0.0
        self.vz_m_s = 0.0


# 50 字节目标体: 3xint32, 2xuint16, 5xfloat, 2xuint8, 3xfloat  (小端)
_TARGET_STRUCT = struct.Struct("<iiiHHfffffBBfff")


def frame_size(target_count):
    return HEADER_SIZE + target_count * TARGET_SIZE + CRC_SIZE


def encode(targets, sequence, time_ms, crc_mode):
    """把目标列表编码成协议帧 (与 codec::encode 一致)。"""
    count = len(targets)
    if count == 0 or count > MAX_TARGETS:
        raise ValueError("目标数量非法: %d" % count)

    total = frame_size(count)
    buf = bytearray(total)

    buf[0] = FRAME_HEADER
    buf[1] = PAYLOAD_LENGTH_FIELD
    buf[2] = sequence & 0xFF
    buf[3] = MESSAGE_TYPE
    buf[4] = RESERVED
    buf[5], buf[6], buf[7] = IDENT
    struct.pack_into("<Q", buf, 8, time_ms & 0xFFFFFFFFFFFFFFFF)
    struct.pack_into("<H", buf, 16, total)
    buf[18] = count

    off = HEADER_SIZE
    for t in targets:
        _TARGET_STRUCT.pack_into(
            buf, off,
            t.longitude_e7, t.latitude_e7, t.altitude_mm,
            t.target_id, t.delete_flag,
            t.speed_m_s, t.distance_m, t.azimuth_rad, t.elevation_rad, t.altitude_m,
            t.track_type, t.target_attribute,
            t.vx_m_s, t.vy_m_s, t.vz_m_s,
        )
        off += TARGET_SIZE

    struct.pack_into("<H", buf, total - CRC_SIZE, crc16(crc_mode, buf[:total - CRC_SIZE]))
    return bytes(buf)


def decode(buf, crc_mode):
    """解码协议帧, 返回 (frame_info_dict, [Target...])。校验失败抛 ValueError。"""
    length = len(buf)
    if length < frame_size(1):
        raise ValueError("帧太短")
    if buf[0] != FRAME_HEADER or buf[3] != MESSAGE_TYPE or buf[4] != RESERVED:
        raise ValueError("帧头错误")
    if (buf[5], buf[6], buf[7]) != IDENT:
        raise ValueError("标识符错误")
    if buf[1] != PAYLOAD_LENGTH_FIELD:
        raise ValueError("载荷长度字段错误")

    count = buf[18]
    if count == 0 or count > MAX_TARGETS:
        raise ValueError("目标数量错误")

    expected = frame_size(count)
    if length != expected or struct.unpack_from("<H", buf, 16)[0] != expected:
        raise ValueError("长度不匹配")

    if crc_mode != "none":
        got = struct.unpack_from("<H", buf, length - CRC_SIZE)[0]
        calc = crc16(crc_mode, buf[:length - CRC_SIZE])
        if got != calc:
            raise ValueError("CRC 错误 (帧内=0x%04X 计算=0x%04X)" % (got, calc))

    info = {
        "sequence": buf[2],
        "time_ms": struct.unpack_from("<Q", buf, 8)[0],
        "target_count": count,
    }

    targets = []
    off = HEADER_SIZE
    for _ in range(count):
        vals = _TARGET_STRUCT.unpack_from(buf, off)
        t = Target()
        (t.longitude_e7, t.latitude_e7, t.altitude_mm, t.target_id, t.delete_flag,
         t.speed_m_s, t.distance_m, t.azimuth_rad, t.elevation_rad, t.altitude_m,
         t.track_type, t.target_attribute, t.vx_m_s, t.vy_m_s, t.vz_m_s) = vals
        targets.append(t)
        off += TARGET_SIZE

    return info, targets


# ---------------------------------------------------------------------------
# 十进制 -> Target (与 command_send_decimal 一致)
# ---------------------------------------------------------------------------
def build_target_from_decimal(target_id, lon, lat, alt, vx, vy, vz,
                              track_type, target_attribute):
    if not (1 <= target_id <= 0xFFFF):
        raise ValueError("id 必须在 1..65535")
    if not (-180.0 <= lon <= 180.0):
        raise ValueError("经度必须在 -180..180")
    if not (-90.0 <= lat <= 90.0):
        raise ValueError("纬度必须在 -90..90")

    altitude_mm = alt * 1000.0
    if altitude_mm < -2147483648.0 or altitude_mm > 2147483647.0:
        raise ValueError("高度超出 int32 毫米范围")

    t = Target()
    t.target_id = target_id
    t.longitude_e7 = int(round(lon * 1e7))
    t.latitude_e7 = int(round(lat * 1e7))
    t.altitude_mm = int(round(altitude_mm))
    t.altitude_m = alt
    t.vx_m_s = vx
    t.vy_m_s = vy
    t.vz_m_s = vz
    t.speed_m_s = math.sqrt(vx * vx + vy * vy + vz * vz)
    t.track_type = track_type & 0xFF
    t.target_attribute = target_attribute & 0xFF
    return t


# ---------------------------------------------------------------------------
# 打印 (风格与模块 dump_frame / dump_target 对齐)
# ---------------------------------------------------------------------------
_print_lock = threading.Lock()


def log(*args):
    with _print_lock:
        print(*args)
        sys.stdout.flush()


def dump_frame(direction, buf):
    lines = ["%s HEX len=%d" % (direction, len(buf))]
    for offset in range(0, len(buf), 16):
        chunk = buf[offset:offset + 16]
        hexs = " ".join("%02X" % b for b in chunk)
        lines.append("%s HEX +%03d: %s" % (direction, offset, hexs))
    log("\n".join(lines))


def dump_target(direction, t):
    log("%s DEC id=%d lon=%.7f lat=%.7f alt=%.3f speed=%.3f" % (
        direction, t.target_id,
        t.longitude_e7 * 1e-7, t.latitude_e7 * 1e-7,
        t.altitude_m, t.speed_m_s))
    log("%s DEC vx=%.3f vy=%.3f vz=%.3f del=%d type=0x%02x attr=0x%02x" % (
        direction, t.vx_m_s, t.vy_m_s, t.vz_m_s,
        t.delete_flag, t.track_type, t.target_attribute))


# ---------------------------------------------------------------------------
# 接收线程: 实时解码显示收到的帧
# ---------------------------------------------------------------------------
class Receiver(threading.Thread):
    def __init__(self, sock, crc_mode):
        super().__init__(daemon=True)
        self._sock = sock
        self._crc_mode = crc_mode
        self._running = True
        self.rx_packets = 0
        self.rx_dropped = 0

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            try:
                data, src = self._sock.recvfrom(MAX_DATAGRAM_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            if not data:
                continue

            log("\n---- RX from %s:%d ----" % (src[0], src[1]))
            dump_frame("RX", data)
            try:
                info, targets = decode(data, self._crc_mode)
            except ValueError as exc:
                self.rx_dropped += 1
                log("RX 解码失败: %s" % exc)
                continue

            self.rx_packets += 1
            log("RX INFO seq=%d time_ms=%d count=%d" % (
                info["sequence"], info["time_ms"], info["target_count"]))
            for t in targets:
                dump_target("RX", t)


# ---------------------------------------------------------------------------
# 交互解析 / 发送
# ---------------------------------------------------------------------------
def parse_hex_input(text):
    cleaned = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c in " \t:-_,":
            i += 1
            continue
        if c == '0' and i + 1 < n and text[i + 1] in "xX":
            i += 2
            continue
        cleaned.append(c)
        i += 1
    hexstr = "".join(cleaned)
    if len(hexstr) == 0 or len(hexstr) % 2 != 0:
        raise ValueError("hex 输入必须是完整的字节")
    return bytes(int(hexstr[j:j + 2], 16) for j in range(0, len(hexstr), 2))


HELP_TEXT = """\
可用命令:
  <id> <lon> <lat> <alt> <vx> <vy> <vz>    发送一个十进制目标帧 (等同 senddec)
  senddec <id> <lon> <lat> <alt> <vx> <vy> <vz>
  sendhex <FD 10 ... CRC>                   直接发送/校验一个原始 hex 帧
  auto <hz>                                 以指定频率自动重发上一个目标帧
  auto off                                  停止自动重发
  status                                    显示统计信息
  help                                      显示本帮助
  quit / exit                               退出

示例:
  100 8.5462000 47.3980000 20.5 1.0 2.0 -0.5
"""


class Sender:
    def __init__(self, sock, target_addr, crc_mode, track_type, target_attribute):
        self._sock = sock
        self._target = target_addr
        self._crc_mode = crc_mode
        self._track_type = track_type
        self._target_attribute = target_attribute
        self._seq = 0
        self.tx_packets = 0
        self._last_targets = None
        self._auto_thread = None
        self._auto_stop = threading.Event()

    def _next_seq(self):
        s = self._seq & 0xFF
        self._seq = (self._seq + 1) & 0xFF
        return s

    def _send_targets(self, targets):
        seq = self._next_seq()
        time_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        frame = encode(targets, seq, time_ms, self._crc_mode)
        log("\n---- TX to %s:%d ----" % (self._target[0], self._target[1]))
        dump_frame("TX", frame)
        for t in targets:
            dump_target("TX", t)
        self._sock.sendto(frame, self._target)
        self.tx_packets += 1
        self._last_targets = targets

    def send_decimal(self, fields):
        if len(fields) != 7:
            raise ValueError("需要 7 个值: <id> <lon> <lat> <alt> <vx> <vy> <vz>")
        target_id = int(fields[0], 0)
        lon = float(fields[1])
        lat = float(fields[2])
        alt = float(fields[3])
        vx = float(fields[4])
        vy = float(fields[5])
        vz = float(fields[6])
        t = build_target_from_decimal(target_id, lon, lat, alt, vx, vy, vz,
                                      self._track_type, self._target_attribute)
        self._send_targets([t])

    def send_hex(self, text):
        frame = parse_hex_input(text)
        # 本地校验后再发, 与模块 sendhex 行为一致
        info, targets = decode(frame, self._crc_mode)
        log("\n---- TX (hex) to %s:%d ----" % (self._target[0], self._target[1]))
        dump_frame("TX", frame)
        for t in targets:
            dump_target("TX", t)
        self._sock.sendto(frame, self._target)
        self.tx_packets += 1
        self._seq = (info["sequence"] + 1) & 0xFF
        self._last_targets = targets

    def start_auto(self, hz):
        self.stop_auto()
        if self._last_targets is None:
            raise ValueError("还没有可重发的目标, 先手动发送一次")
        hz = max(0.1, min(50.0, hz))
        interval = 1.0 / hz
        self._auto_stop.clear()

        def loop():
            while not self._auto_stop.wait(interval):
                try:
                    self._send_targets(self._last_targets)
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


def run_repl(sender, receiver, args):
    log("target_frame_sim 已启动")
    log("  发送目标: %s:%d  (CRC=%s)" % (args.target_ip, args.target_port, args.crc))
    log("  本地绑定: 0.0.0.0:%d  (源端口需等于 PX4 的 -o remote_port)" % args.local_port)
    log("  track_type=0x%02x target_attribute=0x%02x" % (args.track_type, args.target_attribute))
    log("")
    log(HELP_TEXT)

    while True:
        try:
            line = input("frame> ").strip()
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
                log("TX=%d  RX=%d  RX_dropped=%d" % (
                    sender.tx_packets, receiver.rx_packets, receiver.rx_dropped))
            elif cmd == "senddec":
                sender.send_decimal(parts[1:])
            elif cmd == "sendhex":
                sender.send_hex(" ".join(parts[1:]))
            elif cmd == "auto":
                if len(parts) >= 2 and parts[1].lower() == "off":
                    sender.stop_auto()
                    log("自动重发已停止")
                elif len(parts) >= 2:
                    sender.start_auto(float(parts[1]))
                else:
                    log("用法: auto <hz> | auto off")
            elif len(parts) == 7:
                # 裸的 7 个数值 => 等同 senddec
                sender.send_decimal(parts)
            else:
                log("无法识别的命令, 输入 help 查看用法")
        except (ValueError, OSError) as exc:
            log("错误: %s" % exc)

    sender.stop_auto()
    receiver.stop()


def main():
    parser = argparse.ArgumentParser(
        description="target_frame_udp 协议模拟发送/接收脚本 (替代对端 PX4)")
    parser.add_argument("--target-ip", required=True,
                        help="运行 PX4 的本机有线网卡 IP (对端 PX4 的地址)")
    parser.add_argument("--target-port", type=int, default=50000,
                        help="PX4 模块的 local_port (-l), 默认 50000")
    parser.add_argument("--local-port", type=int, default=50000,
                        help="本脚本绑定的本地端口, 必须等于 PX4 模块的 remote_port (-o), 默认 50000")
    parser.add_argument("--crc", default="modbus",
                        choices=["none", "modbus", "ccitt", "ccitt-false", "x25"],
                        help="CRC-16 模式, 必须与 PX4 的 -c 一致, 默认 modbus")
    parser.add_argument("--track-type", type=lambda x: int(x, 0), default=DEFAULT_TRACK_TYPE,
                        help="track_type 字节 (默认 0x20)")
    parser.add_argument("--target-attribute", type=lambda x: int(x, 0), default=DEFAULT_TARGET_ATTRIBUTE,
                        help="target_attribute 字节 (默认 0x22)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", args.local_port))
    except OSError as exc:
        print("绑定本地端口 %d 失败: %s" % (args.local_port, exc), file=sys.stderr)
        print("提示: 若与本机 PX4 SITL 抢占同一端口, 请在两台真实机器上运行, "
              "或用不同端口并相应调整 PX4 的 -l/-o。", file=sys.stderr)
        sys.exit(1)
    sock.settimeout(0.5)

    target_addr = (args.target_ip, args.target_port)

    receiver = Receiver(sock, args.crc)
    receiver.start()
    sender = Sender(sock, target_addr, args.crc, args.track_type, args.target_attribute)

    try:
        run_repl(sender, receiver, args)
    finally:
        receiver.stop()
        sock.close()


if __name__ == "__main__":
    main()
