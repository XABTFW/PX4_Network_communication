# Target-frame UDP module

`target_frame_udp` implements the target-frame protocol from
`/home/btfw_1/反无雷达-协议20260715.docx` directly inside PX4. It sends this
vehicle's global position over UDP and publishes received peer targets to the
existing `follower_info` uORB topic.

## Protocol assumptions

- All multibyte fields are little-endian.
- A one-target frame is 67 bytes: 15-byte header, 50-byte target, 2-byte CRC.
- The documented payload-length byte remains the fixed value `0x10`.
- Longitude/latitude retain the documented 32-bit wire layout but are treated as
  signed E7 values so western/southern coordinates work.
- Integer altitude is millimetres; float altitude is metres AMSL.
- The 32-bit millisecond time is PX4 time-since-boot and naturally wraps.
- Vx/Vy/Vz use PX4 NED velocity, so positive Vz is down.
- Distance, azimuth and elevation are zero because the sender does not know the
  receiver's position. Consumers use the global coordinates instead.
- The document does not define its CRC-16 profile. `modbus` is the temporary
  default; `none`, `ccitt` (CCITT-FALSE), and `x25` are also supported.
- CRC covers every byte except the final two CRC bytes, and the CRC itself is
  sent low byte first.

Confirm these assumptions with the protocol owner before interoperability tests.

## Two-computer SITL example

Set static wired addresses `192.168.50.11/24` and `192.168.50.12/24`, allow UDP
port 50000, and give the PX4 instances different `MAV_SYS_ID` values.

Computer A, PX4 shell:

```sh
param set MAV_SYS_ID 1
target_frame_udp start -t 192.168.50.12 -l 50000 -o 50000 -r 10 -c modbus -y 0x20 -a 0x22
```

Computer B, PX4 shell:

```sh
param set MAV_SYS_ID 2
target_frame_udp start -t 192.168.50.11 -l 50000 -o 50000 -r 10 -c modbus -y 0x20 -a 0x22
```

Verify operation:

```sh
target_frame_udp test
target_frame_udp status
listener follower_info
listener vehicle_global_position
```

## Manual decimal/hexadecimal test

Pause the periodic ownship frames before a manual test so that their output is
not mixed with the test frame. Enable live dumps on both computers:

```sh
target_frame_udp auto off
target_frame_udp dump on
```

Send decimal target fields from either PX4 shell. Longitude and latitude are in
degrees, altitude is in metres, and `vx/vy/vz` are NED velocities in m/s
(positive `vz` is down):

```sh
target_frame_udp senddec 100 8.5462000 47.3980000 20.5 1.0 2.0 -0.5
```

The sender prints the encoded packet as offset-labelled `TX HEX` lines (16 bytes
per line) and its decimal fields as `TX DEC`. Concatenate the byte groups in
offset order to obtain the complete frame. The peer prints the same packet as
`RX HEX`, followed by `RX DEC`, and publishes it to `follower_info` when the
target ID differs from its own `MAV_SYS_ID`.

To send hexadecimal input, provide one complete protocol packet, including its
two CRC bytes. The packet must match the CRC mode selected by `-c`. Spaces,
colons and a contiguous hexadecimal string are accepted:

```sh
target_frame_udp sendhex FD 10 ... <CRC-low> <CRC-high>
target_frame_udp sendhex FD10...<CRC-low><CRC-high>
```

An easy way to obtain a valid packet is to copy the `TX HEX` output produced by
`senddec` and submit those bytes to `sendhex`. The module validates and decodes
the packet locally before transmission. Restore normal periodic transmission
after testing:

```sh
target_frame_udp dump off
target_frame_udp auto on
```

Host-side packet capture:

```sh
sudo tcpdump -ni <wired-interface> udp port 50000 -XX
```

For two SITL instances on one computer, use crossed ports instead of binding the
same port twice: A uses `-l 50000 -o 50001`, B uses `-l 50001 -o 50000`.

Received deletion records are validated but not published. Existing consumers
already expire `follower_info` samples by timeout, preventing a deletion record
from being interpreted as a valid position.
