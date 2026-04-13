/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file dj_mk.cpp
 *
 * Driver for ASMG-MD series servo connected over CAN bus.
 *
 * Hardware:
 * - Flight Controller: Pixhawk 6C
 * - Servo: ASMG-MD series
 * - CAN Port: CAN2 (default, mapped to can1 interface)
 * - Baudrate: 250kbps
 * - Protocol: Extended CAN frame (EXT)
 * - Frame ID: 0x18EF0201
 * - Data Length: 8 bytes
 * - Format: FF FF 01 01 POSH POSL SPEEDH SPEEDL
 *
 * Note:
 * - Hardware CAN1 port -> can0 interface (used by dm_imu_l1)
 * - Hardware CAN2 port -> can1 interface (used by dj_mk, default)
 */

#include "dj_mk.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <drivers/drv_hrt.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ioctl.h>
#include <px4_platform_common/getopt.h>

#ifndef IFF_UP
#define IFF_UP 0x1
#endif

#ifndef IFF_RUNNING
#define IFF_RUNNING 0x40
#endif

DjMk::DjMk(const char *can_dev) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_can_dev(can_dev)
{
	// 直接使用默认值，不依赖参数系统
	_servo_id = 1;
	_servo_speed = 2;  // 默认速度改为2 (0x0002)，可通过参数或命令调整

	PX4_INFO("DJ_MK driver created for %s (Hardware CAN%s)",
		_can_dev,
		(strcmp(_can_dev, "can0") == 0) ? "1" : "2");
}

DjMk::~DjMk()
{
	ScheduleClear();

	if (_can_fd >= 0) {
		::close(_can_fd);
		_can_fd = -1;
	}
}

int DjMk::init()
{
	_state = STATE_INITIALIZED;
	PX4_INFO("DJ_MK driver initialized, socket will be opened in Run()");
	return 0;
}

int DjMk::open_can_socket()
{
	// 首先检查网络设备状态
	FAR struct net_driver_s *dev = netdev_findbyname(_can_dev);
	if (dev == NULL) {
		PX4_ERR("CAN device '%s' not found", _can_dev);
		return -1;
	}

	PX4_INFO("%s flags before config: 0x%04lx (UP=%d, RUNNING=%d)",
		_can_dev, (unsigned long)dev->d_flags,
		(dev->d_flags & IFF_UP) ? 1 : 0,
		(dev->d_flags & IFF_RUNNING) ? 1 : 0);

	// 临时注释掉 bitrate 设置，因为它会导致 FDCAN IE=0（中断未启用）
	// 问题：SIOCSCANBITRATE 失败时会清除 IE 寄存器，导致中断链路断开
	// 解决方案：使用 defconfig 中预设的 bitrate（FDCAN2_ARBI_BITRATE=250000）
	/*
	// 设置波特率为 250kbps（仅对 can1/FDCAN2 设置）
	// Note: Bitrate should ideally be set when interface is down, but we skip
	// bringing it down since netdev_ifdown() is not available in this build
	if (strcmp(_can_dev, "can1") == 0) {
		// 先创建一个临时 socket 用于 ioctl
		int tmp_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (tmp_fd >= 0) {
			struct ifreq ifr_bitrate = {};
			snprintf(ifr_bitrate.ifr_name, IFNAMSIZ, "%s", _can_dev);

			// 设置波特率为 250kbps (注意：单位是 kbit/s)
			ifr_bitrate.ifr_ifru.ifru_can_data.arbi_bitrate = 250;

			if (ioctl(tmp_fd, SIOCSCANBITRATE, (unsigned long)&ifr_bitrate) < 0) {
				PX4_WARN("Failed to set bitrate to 250kbps: errno=%d (%s)", errno, strerror(errno));
				PX4_INFO("Will use existing bitrate configuration");
			} else {
				PX4_INFO("Bitrate set to 250kbps for %s", _can_dev);
			}

			// 确保接口 UP
			if ((dev->d_flags & IFF_UP) == 0) {
				PX4_INFO("Bringing %s up...", _can_dev);
				netdev_ifup(dev);
				px4_usleep(100000);
			}

			// 验证波特率
			memset(&ifr_bitrate, 0, sizeof(ifr_bitrate));
			snprintf(ifr_bitrate.ifr_name, IFNAMSIZ, "%s", _can_dev);
			if (ioctl(tmp_fd, SIOCGCANBITRATE, (unsigned long)&ifr_bitrate) == 0) {
				PX4_INFO("Current bitrate: %u kbps", ifr_bitrate.ifr_ifru.ifru_can_data.arbi_bitrate);
			}

			::close(tmp_fd);

		} else {
			PX4_WARN("Failed to create temp socket for bitrate config");
		}

	} else {
		// 非 can1，确保接口 UP
		if ((dev->d_flags & IFF_UP) == 0) {
			PX4_INFO("Bringing %s up...", _can_dev);
			netdev_ifup(dev);
			px4_usleep(100000);
		}
	}
	*/

	// 确保接口 UP
	if ((dev->d_flags & IFF_UP) == 0) {
		PX4_INFO("Bringing %s up...", _can_dev);
		netdev_ifup(dev);
		px4_usleep(100000);
	}

	// 创建 socket
	_can_fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
	if (_can_fd < 0) {
		PX4_ERR("socket() failed: errno=%d (%s)", errno, strerror(errno));
		return -1;
	}

	// 检查接口是否存在
	unsigned ifindex = if_nametoindex(_can_dev);
	if (ifindex == 0) {
		PX4_ERR("if_nametoindex() failed: errno=%d (%s)", errno, strerror(errno));
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}
	PX4_INFO("CAN interface '%s' found, ifindex=%u", _can_dev, ifindex);

	// 设置 CAN socket 选项
	int loopback = 0;
	if (setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
		PX4_WARN("Failed to disable CAN_RAW_LOOPBACK: errno=%d", errno);
	} else {
		PX4_INFO("CAN_RAW_LOOPBACK disabled");
	}

	int recv_own_msgs = 0;
	if (setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
		PX4_WARN("Failed to disable CAN_RAW_RECV_OWN_MSGS: errno=%d", errno);
	} else {
		PX4_INFO("CAN_RAW_RECV_OWN_MSGS disabled");
	}

	// 设置 CAN 过滤器：接收所有帧（包括扩展帧）
	// 注意：必须显式调用 setsockopt(CAN_RAW_FILTER)，否则 NuttX 默认过滤器
	// 可能不接收扩展帧（29-bit ID），导致收不到舵机回复（0x18EF0102）
	struct can_filter filter = {};
	filter.can_id = 0;
	filter.can_mask = 0;  // mask=0 表示接收所有帧（标准帧和扩展帧）

	if (setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
		PX4_ERR("setsockopt(CAN_RAW_FILTER) failed: errno=%d (%s)", errno, strerror(errno));
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}

	PX4_INFO("CAN filter set to receive ALL frames (including extended)");

	// 绑定到CAN接口
	struct sockaddr_can addr = {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifindex;

	if (bind(_can_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind() failed: errno=%d (%s)", errno, strerror(errno));
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}

	PX4_INFO("CAN socket opened and bound to %s successfully (fd=%d)", _can_dev, _can_fd);
	return 0;
}

void DjMk::Run()
{
	static uint32_t run_count = 0;
	run_count++;

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// 如果 socket 还没打开，在这里打开（在 work queue 上下文中）
	if (_can_fd < 0 && _state == STATE_INITIALIZED) {
		if (open_can_socket() < 0) {
			PX4_ERR("Failed to open CAN socket in Run()");
			_state = STATE_ERROR;
			return;
		}
		_socket_open_time = hrt_absolute_time();
		_state = STATE_RUNNING;
		PX4_INFO("CAN socket opened successfully, fd=%d", _can_fd);
	}

	// socket 应该在上面已经打开
	if (_can_fd < 0) {
		return;
	}

	// 正常运行状态
	if (_state != STATE_RUNNING) {
		return;
	}

	// 处理来自 custom_command 的待执行命令
	if (_cmd_pending) {
		_cmd_pending = false;

		if (_query_mode) {
			// 查询模式：发送查询 PID 命令（根据你的调试工具验证的格式）
			_query_mode = false;
			_query_start_time = hrt_absolute_time();
			_query_response_received = false;

			struct can_frame frame = {};
			frame.can_id = CAN_FRAME_ID | CAN_EFF_FLAG;
			frame.can_dlc = 8;
			frame.data[0] = 0x00;
			frame.data[1] = 0x06;  // 查询 PID 命令（根据调试工具验证）
			frame.data[2] = 0x00;
			frame.data[3] = 0x00;
			frame.data[4] = 0x00;
			frame.data[5] = 0x00;
			frame.data[6] = 0x00;
			frame.data[7] = 0x00;

			ssize_t nbytes = send(_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
			if (nbytes == sizeof(frame)) {
				PX4_INFO("PID query sent: 00 06 00 00 00 00 00 00");
				PX4_INFO("CAN Frame ID: 0x%08X", (unsigned int)frame.can_id);
				PX4_INFO("Listening for response for 5 seconds...");
				_waiting_for_response = true;
			} else {
				PX4_ERR("Failed to send PID query: errno=%d (%s)", errno, strerror(errno));
			}
		} else if (_raw_mode) {
			// 原始数据模式
			_raw_mode = false;

			struct can_frame frame = {};
			frame.can_id = CAN_FRAME_ID | CAN_EFF_FLAG;
			frame.can_dlc = 8;
			for (int i = 0; i < 8; i++) {
				frame.data[i] = _raw_data[i];
			}

			ssize_t nbytes = send(_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
			if (nbytes == sizeof(frame)) {
				PX4_INFO("Raw frame sent successfully:");
				PX4_INFO("  ID: 0x%08X", (unsigned)(frame.can_id & CAN_EFF_MASK));
				PX4_INFO("  Data: %02X %02X %02X %02X %02X %02X %02X %02X",
					frame.data[0], frame.data[1], frame.data[2], frame.data[3],
					frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
			} else {
				PX4_ERR("Failed to send raw frame: errno=%d (%s)", errno, strerror(errno));
			}
		} else {
			// 正常位置控制模式
			send_servo_command(_cmd_servo_id, _cmd_position, _cmd_speed);
		}
	}

	// 持续查询模式：每秒发送一次查询
	if (_continuous_query_mode) {
		uint64_t now = hrt_absolute_time();
		if (now - _last_query_time > 1000000) {  // 1秒间隔
			_last_query_time = now;

			struct can_frame frame = {};
			frame.can_id = CAN_FRAME_ID | CAN_EFF_FLAG;
			frame.can_dlc = 8;
			frame.data[0] = 0x00;
			frame.data[1] = 0x06;  // 查询 PID 命令
			frame.data[2] = 0x00;
			frame.data[3] = 0x00;
			frame.data[4] = 0x00;
			frame.data[5] = 0x00;
			frame.data[6] = 0x00;
			frame.data[7] = 0x00;

			ssize_t nbytes = send(_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
			if (nbytes == sizeof(frame)) {
				_query_sent_count++;  // 统计发送的查询次数
				_tx_frame_count++;    // 统计发送的总帧数

				PX4_INFO("[%llu] PID query sent: 00 06 00 00 00 00 00 00", now / 1000000);
				PX4_INFO("  TX: ID=0x%08X EXT DLC=8 (raw=0x%08X)",
					(unsigned)(frame.can_id & CAN_EFF_MASK),
					(unsigned)frame.can_id);

				// 只设置等待标志，不在这里读取
				// 统一在下面的接收循环中处理响应
				_waiting_for_response = true;
				_query_start_time = now;
			} else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_ERR("Failed to send continuous query: errno=%d", errno);
			}
		}
	}

	// 尝试接收舵机的回复消息（持续接收，不只是查询时）
	// 直接使用 nonblocking read() 直到 EAGAIN，不依赖 poll()
	// 因为 FDCAN IE=0（中断未启用），poll() 不会收到通知
	{
		int reads = 0;

		while (reads < 20) {
			struct can_frame rx_frame = {};
			ssize_t nbytes = read(_can_fd, &rx_frame, sizeof(rx_frame));

			if (nbytes < (ssize_t)sizeof(rx_frame)) {
				if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
					static uint64_t last_err_time = 0;
					uint64_t now_err = hrt_absolute_time();

					if (now_err - last_err_time > 5000000) {
						PX4_WARN("read() error: errno=%d (%s)", errno, strerror(errno));
						last_err_time = now_err;
					}
				}

				break;  // EAGAIN 或错误，退出循环
			}

			reads++;

			// 移除扩展帧标志位，只看 ID
			uint32_t frame_id = rx_frame.can_id & CAN_EFF_MASK;
			bool is_ext = (rx_frame.can_id & CAN_EFF_FLAG) != 0;

			// 统计接收帧数
			_rx_frame_count++;

			PX4_INFO("Received CAN frame: ID=0x%08X (EXT=%d), DLC=%d, Data=%02X %02X %02X %02X %02X %02X %02X %02X",
				(unsigned int)frame_id, is_ext ? 1 : 0, rx_frame.can_dlc,
				rx_frame.data[0], rx_frame.data[1], rx_frame.data[2], rx_frame.data[3],
				rx_frame.data[4], rx_frame.data[5], rx_frame.data[6], rx_frame.data[7]);

			// 检查是否是舵机的回复帧 (0x18EF0102)
			if (frame_id == CAN_REPLY_ID) {
				// 如果正在等待查询响应
				if (_waiting_for_response) {
					_query_response_received = true;
					_waiting_for_response = false;
					_query_response_count++;  // 统计收到的查询响应次数

					// 解析 PID 响应 (根据说明书: 00 06 P_H P_L I_H I_L D_H D_L)
					if (rx_frame.can_dlc >= 8 && rx_frame.data[1] == 0x06) {
						uint16_t p_value = (rx_frame.data[2] << 8) | rx_frame.data[3];
						uint16_t i_value = (rx_frame.data[4] << 8) | rx_frame.data[5];
						uint16_t d_value = (rx_frame.data[6] << 8) | rx_frame.data[7];
						PX4_INFO("PID Response: P=0x%04X, I=0x%04X, D=0x%04X", p_value, i_value, d_value);
					}
				}
			}
		}
	}

	// 检查查询超时
	if (_waiting_for_response) {
		uint64_t now = hrt_absolute_time();
		if (now - _query_start_time > 5000000) {  // 5秒超时
			PX4_WARN("PID query timeout - no response received after 5 seconds");
			PX4_INFO("Possible issues:");
			PX4_INFO("  1. Servo ID mismatch (current ID=%d)", (int)_servo_id);
			PX4_INFO("  2. CAN bus wiring or termination problem");
			PX4_INFO("  3. Servo not supporting PID query command");
			PX4_INFO("  4. Wrong CAN frame ID or data format");
			_waiting_for_response = false;
		}
	}

	// 检查是否有新的执行器输出数据
	actuator_outputs_s actuator_outputs;

	if (_actuator_outputs_sub.update(&actuator_outputs)) {
		// 假设第一个输出通道用于控制舵机
		// 输出值范围通常是 -1.0 到 1.0，需要映射到 PWM 值 (1000-2000us)
		float output_value = actuator_outputs.output[0];

		// 将 -1.0~1.0 映射到 1000~2000us
		uint16_t pwm_value = 1500 + (int16_t)(output_value * 500.0f);

		// 限制范围
		if (pwm_value < 1000) { pwm_value = 1000; }

		if (pwm_value > 2000) { pwm_value = 2000; }

		// 转换为舵机位置
		uint16_t position = pwm_to_position(pwm_value);

		// 发送控制命令到配置的舵机ID
		send_servo_command((uint8_t)_servo_id, position, (uint16_t)_servo_speed);
	}
}

int16_t DjMk::send_servo_command(uint8_t servo_id, uint16_t position, uint16_t speed)
{
	if (_can_fd < 0) {
		PX4_ERR("CAN device not initialized (fd=%d)", _can_fd);
		return -1;
	}

	// 限制位置范围
	if (position > SERVO_POS_MAX) {
		position = SERVO_POS_MAX;
	}

	// 构建 SocketCAN 帧
	struct can_frame frame = {};

	// 使用扩展帧，ID = 0x18EF0201
	frame.can_id = CAN_FRAME_ID | CAN_EFF_FLAG;  // CAN_EFF_FLAG 表示扩展帧
	frame.can_dlc = 8;  // 数据长度8字节

	// 协议格式（实测）: 00 ID CMD POS_H POS_L SPEED_H SPEED_L 00
	frame.data[0] = 0x00;                    // 帧头/命令类型
	frame.data[1] = servo_id;                // 舵机ID
	frame.data[2] = 0x00;                    // 位置控制命令（修改为0x00）
	frame.data[3] = (position >> 8) & 0xFF;  // 位置高字节
	frame.data[4] = position & 0xFF;         // 位置低字节
	frame.data[5] = (speed >> 8) & 0xFF;     // 速度高字节（可调整，默认0x02）
	frame.data[6] = speed & 0xFF;            // 速度低字节（可调整，默认0x00）
	frame.data[7] = 0x00;                    // 保留

	// 使用 send() 而不是 write()，添加 MSG_DONTWAIT 标志
	ssize_t nbytes = send(_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);

	if (nbytes == sizeof(frame)) {
		return nbytes;
	}

	if (nbytes < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			PX4_WARN("Send buffer full, retry later");
			return -EAGAIN;
		}
		PX4_ERR("send() failed: errno=%d (%s), fd=%d", errno, strerror(errno), _can_fd);
		return -1;
	}

	PX4_ERR("send() partial: sent %d bytes", (int)nbytes);
	return -1;
}

uint16_t DjMk::pwm_to_position(uint16_t pwm_value)
{
	// PWM范围: 1000us ~ 2000us
	// 舵机位置范围: 0x0000 ~ 0x7FFF
	// 中间位置: 1500us -> 0x2000

	// 限制PWM范围
	if (pwm_value < 1000) { pwm_value = 1000; }

	if (pwm_value > 2000) { pwm_value = 2000; }

	// 线性映射: (pwm - 1000) / 1000 * 0x7FFF
	uint32_t position = ((uint32_t)(pwm_value - 1000) * SERVO_POS_MAX) / 1000;

	return (uint16_t)position;
}

int DjMk::start()
{
	// 启动定时任务，以SAMPLE_RATE频率运行
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
	PX4_INFO("DjMk driver started at %u Hz on %s", (unsigned int)SAMPLE_RATE, _can_dev);

	return PX4_OK;
}

int DjMk::task_spawn(int argc, char *argv[])
{
	const char *can_dev = "can1";  // 默认使用 can1 (对应硬件 CAN2 口)

	// 解析命令行参数
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			can_dev = myoptarg;
			break;
		default:
			PX4_WARN("Unknown option");
			break;
		}
	}

	DjMk *instance = new DjMk(can_dev);

	if (!instance) {
		PX4_ERR("Driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != 0) {
		PX4_ERR("Init failed");
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	instance->start();
	return PX4_OK;
}

int DjMk::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for ASMG-MD series servo control via CAN bus.

This driver controls ASMG-MD servos through CAN interface using extended CAN frames.

Hardware Configuration:
- Flight Controller: Pixhawk 6C
- Default CAN Port: CAN2 (can1 interface)
- Baudrate: 250kbps
- Frame Type: Extended (EXT)
- Frame ID: 0x18EF0201

Note: Hardware CAN1 port = can0, Hardware CAN2 port = can1 (default)

Protocol:
- Data Format: FF FF [ID] 01 [POS_H] [POS_L] [SPEED_H] [SPEED_L]
- Position Range: 0x0000 ~ 0x7FFF (center: 0x2000)
- Default Speed: 0x0100

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dj_mk", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "can1", "<device>", "CAN device (can0=CAN1, can1=CAN2)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Set servo position directly");
	PRINT_MODULE_USAGE_ARG("position", "Position value (0-32767, center=8192)", false);
	PRINT_MODULE_USAGE_ARG("speed", "Speed value (0-65535, default=256)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Set servo position using PWM value");
	PRINT_MODULE_USAGE_ARG("value", "PWM value (1000-2000us, center=1500)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("raw", "Send raw 8-byte CAN data");
	PRINT_MODULE_USAGE_ARG("b0-b7", "8 bytes in hex (e.g., 00 01 00 00 00 00 02 00)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("query_pid", "Query servo PID parameters (single)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("query_continuous", "Start continuous PID query (1Hz)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("query_stop", "Stop continuous PID query");
	PRINT_MODULE_USAGE_COMMAND_DESCR("listen", "Listen for all CAN frames (debug)");
	PRINT_MODULE_USAGE_ARG("duration", "Listen duration in seconds (default=5)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("test_tx", "Send a test control frame to servo");
	PRINT_MODULE_USAGE_COMMAND_DESCR("flush_rx", "Flush RX FIFO and display frames");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stats", "Display RX statistics and packet loss rate");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stats_reset", "Reset statistics counters");
	PRINT_MODULE_USAGE_COMMAND_DESCR("diag", "Display FDCAN2 hardware registers");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int DjMk::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("Driver not running");
		return PX4_ERROR;
	}

	// 查询舵机 PID 参数（单次）
	if (!strcmp(argv[0], "query_pid")) {
		DjMk *instance = (DjMk *)_object.load();
		if (instance) {
			instance->_cmd_servo_id = (uint8_t)instance->_servo_id;
			instance->_cmd_position = 0;
			instance->_cmd_speed = 0;
			instance->_cmd_pending = true;
			instance->_query_mode = true;

			instance->ScheduleNow();

			PX4_INFO("PID query command sent, check dmesg for response");
			return PX4_OK;
		}
		return PX4_ERROR;
	}

	// 持续查询 PID（每秒一次）
	if (!strcmp(argv[0], "query_continuous")) {
		DjMk *instance = (DjMk *)_object.load();
		if (instance) {
			if (!instance->_continuous_query_mode) {
				instance->_continuous_query_mode = true;
				instance->_last_query_time = 0;  // 立即发送第一次
				PX4_INFO("Continuous PID query started (1Hz)");
				PX4_INFO("Use 'dj_mk query_stop' to stop");
			} else {
				PX4_INFO("Continuous query already running");
			}
			return PX4_OK;
		}
		return PX4_ERROR;
	}

	// 停止持续查询
	if (!strcmp(argv[0], "query_stop")) {
		DjMk *instance = (DjMk *)_object.load();
		if (instance) {
			if (instance->_continuous_query_mode) {
				instance->_continuous_query_mode = false;
				PX4_INFO("Continuous PID query stopped");
			} else {
				PX4_INFO("Continuous query not running");
			}
			return PX4_OK;
		}
		return PX4_ERROR;
	}

	// 监听 CAN 总线上的所有帧（调试用）
	if (!strcmp(argv[0], "listen")) {
		DjMk *instance = (DjMk *)_object.load();
		if (!instance || instance->_can_fd < 0) {
			PX4_ERR("Driver not running or CAN not opened");
			return PX4_ERROR;
		}

		int duration = (argc >= 2) ? atoi(argv[1]) : 5;
		PX4_INFO("Listening for CAN frames for %d seconds...", duration);
		PX4_INFO("Expected TX ID: 0x%08X, Expected RX ID: 0x%08X",
			(unsigned)CAN_FRAME_ID, (unsigned)CAN_REPLY_ID);

		uint64_t start_time = hrt_absolute_time();
		uint64_t end_time = start_time + (duration * 1000000ULL);
		int frame_count = 0;

		while (hrt_absolute_time() < end_time) {
			// 直接 nonblocking read，不使用 poll
			struct can_frame rx_frame = {};
			ssize_t rx_bytes = read(instance->_can_fd, &rx_frame, sizeof(rx_frame));

			if (rx_bytes == sizeof(rx_frame)) {
				uint32_t rx_id = rx_frame.can_id & CAN_EFF_MASK;
				bool is_ext = (rx_frame.can_id & CAN_EFF_FLAG) != 0;
				frame_count++;

				PX4_INFO("[%d] RX: ID=0x%08X %s DLC=%d Data=%02X %02X %02X %02X %02X %02X %02X %02X",
					frame_count,
					(unsigned)rx_id,
					is_ext ? "EXT" : "STD",
					rx_frame.can_dlc,
					rx_frame.data[0], rx_frame.data[1], rx_frame.data[2], rx_frame.data[3],
					rx_frame.data[4], rx_frame.data[5], rx_frame.data[6], rx_frame.data[7]);
			} else {
				// 没有数据，短暂休眠避免空转
				px4_usleep(10000);  // 10ms
			}
		}

		PX4_INFO("Listening complete. Received %d frames", frame_count);
		return PX4_OK;
	}

	// 发送测试帧（用于验证 CAN 发送）
	if (!strcmp(argv[0], "test_tx")) {
		DjMk *instance = (DjMk *)_object.load();
		if (!instance || instance->_can_fd < 0) {
			PX4_ERR("Driver not running or CAN not opened");
			return PX4_ERROR;
		}

		struct can_frame frame = {};
		frame.can_id = CAN_FRAME_ID | CAN_EFF_FLAG;
		frame.can_dlc = 8;
		frame.data[0] = 0xFF;
		frame.data[1] = 0xFF;
		frame.data[2] = 0x01;  // Servo ID
		frame.data[3] = 0x01;
		frame.data[4] = 0x20;  // Position high byte (0x2000 = center)
		frame.data[5] = 0x00;  // Position low byte
		frame.data[6] = 0x01;  // Speed high byte (0x0100 = 256)
		frame.data[7] = 0x00;  // Speed low byte

		ssize_t nbytes = send(instance->_can_fd, &frame, sizeof(frame), 0);
		if (nbytes == sizeof(frame)) {
			PX4_INFO("Test frame sent successfully");
			PX4_INFO("  ID: 0x%08X (raw: 0x%08X)",
				(unsigned)(frame.can_id & CAN_EFF_MASK),
				(unsigned)frame.can_id);
			PX4_INFO("  Data: %02X %02X %02X %02X %02X %02X %02X %02X",
				frame.data[0], frame.data[1], frame.data[2], frame.data[3],
				frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
			PX4_INFO("Servo should move to center position");
		} else {
			PX4_ERR("Failed to send test frame: errno=%d (%s)", errno, strerror(errno));
		}
		return PX4_OK;
	}

	// 清空接收 FIFO（调试用）
	if (!strcmp(argv[0], "flush_rx")) {
		DjMk *instance = (DjMk *)_object.load();
		if (!instance || instance->_can_fd < 0) {
			PX4_ERR("Driver not running or CAN not opened");
			return PX4_ERROR;
		}

		int flushed = 0;
		struct can_frame frame = {};

		// 尝试读取所有待处理的帧
		for (int i = 0; i < 100; i++) {
			ssize_t nbytes = read(instance->_can_fd, &frame, sizeof(frame));
			if (nbytes == sizeof(frame)) {
				flushed++;
				uint32_t id = frame.can_id & CAN_EFF_MASK;
				bool is_ext = (frame.can_id & CAN_EFF_FLAG) != 0;
				PX4_INFO("[%d] Flushed: ID=0x%08X %s Data=%02X %02X %02X %02X %02X %02X %02X %02X",
					flushed, (unsigned)id, is_ext ? "EXT" : "STD",
					frame.data[0], frame.data[1], frame.data[2], frame.data[3],
					frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
			} else {
				break;
			}
		}

		PX4_INFO("Flushed %d frames from RX FIFO", flushed);
		return PX4_OK;
	}

	// 显示统计信息（丢包率等）
	if (!strcmp(argv[0], "stats")) {
		DjMk *instance = (DjMk *)_object.load();
		if (!instance) {
			PX4_ERR("Driver not running");
			return PX4_ERROR;
		}

		PX4_INFO("=== DJ_MK Statistics ===");
		PX4_INFO("Total RX Frames:    %u", (unsigned)instance->_rx_frame_count);
		PX4_INFO("Total TX Frames:    %u", (unsigned)instance->_tx_frame_count);
		PX4_INFO("");
		PX4_INFO("Query Sent:         %u", (unsigned)instance->_query_sent_count);
		PX4_INFO("Query Responses:    %u", (unsigned)instance->_query_response_count);

		if (instance->_query_sent_count > 0) {
			uint32_t lost = instance->_query_sent_count - instance->_query_response_count;
			float loss_rate = (float)lost / (float)instance->_query_sent_count * 100.0f;
			float success_rate = (float)instance->_query_response_count / (float)instance->_query_sent_count * 100.0f;

			PX4_INFO("Query Lost:         %u", (unsigned)lost);
			PX4_INFO("Query Loss Rate:    %.2f%%", (double)loss_rate);
			PX4_INFO("Query Success Rate: %.2f%%", (double)success_rate);
		} else {
			PX4_INFO("Query Loss Rate:    N/A (no queries sent)");
		}

		return PX4_OK;
	}

	// 重置统计信息
	if (!strcmp(argv[0], "stats_reset")) {
		DjMk *instance = (DjMk *)_object.load();
		if (!instance) {
			PX4_ERR("Driver not running");
			return PX4_ERROR;
		}

		instance->_rx_frame_count = 0;
		instance->_tx_frame_count = 0;
		instance->_query_sent_count = 0;
		instance->_query_response_count = 0;

		PX4_INFO("Statistics reset");
		return PX4_OK;
	}

	// 手动控制舵机位置
	if (!strcmp(argv[0], "set")) {
		if (argc < 2) {
			PX4_ERR("Usage: dj_mk set <position> [speed]");
			PX4_INFO("  position: 0-32767 (center=8192)");
			PX4_INFO("  speed: 0-65535 (default=256)");
			return PX4_ERROR;
		}

		uint16_t position = atoi(argv[1]);
		uint16_t speed = (argc >= 3) ? atoi(argv[2]) : 256;

		DjMk *instance = (DjMk *)_object.load();

		if (instance) {
			// 通过共享变量传递命令，由 Run() 在 work queue 上下文中发送
			instance->_cmd_servo_id = (uint8_t)instance->_servo_id;
			instance->_cmd_position = position;
			instance->_cmd_speed = speed;
			instance->_cmd_pending = true;

			// 立即触发 Run() 执行命令
			instance->ScheduleNow();

			PX4_INFO("Command queued: ID=%d, Pos=%u, Speed=%u",
				 (int)instance->_servo_id, (unsigned int)position, (unsigned int)speed);
			return PX4_OK;
		}
	}

	// PWM 方式控制
	if (!strcmp(argv[0], "pwm")) {
		if (argc < 2) {
			PX4_ERR("Usage: dj_mk pwm <value>");
			PX4_INFO("  value: 1000-2000 (center=1500)");
			return PX4_ERROR;
		}

		uint16_t pwm = atoi(argv[1]);
		if (pwm < 1000 || pwm > 2000) {
			PX4_ERR("PWM value must be 1000-2000");
			return PX4_ERROR;
		}

		DjMk *instance = (DjMk *)_object.load();

		if (instance) {
			uint16_t position = instance->pwm_to_position(pwm);
			// 设置命令并立即触发执行
			instance->_cmd_servo_id = (uint8_t)instance->_servo_id;
			instance->_cmd_position = position;
			instance->_cmd_speed = (uint16_t)instance->_servo_speed;
			instance->_cmd_pending = true;

			// 立即触发 Run() 执行命令
			instance->ScheduleNow();

			PX4_INFO("PWM command queued: PWM=%u -> Pos=%u",
				 (unsigned int)pwm, (unsigned int)position);
			return PX4_OK;
		}
	}

	// 原始数据方式控制（直接发送8字节数据）
	if (!strcmp(argv[0], "raw")) {
		if (argc < 9) {
			PX4_ERR("Usage: dj_mk raw <b0> <b1> <b2> <b3> <b4> <b5> <b6> <b7>");
			PX4_INFO("  Example: dj_mk raw 00 01 00 00 00 00 02 00");
			PX4_INFO("  Each byte should be in hex format (00-FF)");
			return PX4_ERROR;
		}

		DjMk *instance = (DjMk *)_object.load();
		if (!instance) {
			PX4_ERR("Driver not running");
			return PX4_ERROR;
		}

		// 解析8个字节到缓冲区
		for (int i = 0; i < 8; i++) {
			unsigned int byte_val;
			if (sscanf(argv[i + 1], "%x", &byte_val) != 1 || byte_val > 0xFF) {
				PX4_ERR("Invalid byte value at position %d: %s", i, argv[i + 1]);
				return PX4_ERROR;
			}
			instance->_raw_data[i] = (uint8_t)byte_val;
		}

		// 设置标志并触发 Run() 执行
		instance->_raw_mode = true;
		instance->_cmd_pending = true;
		instance->ScheduleNow();

		PX4_INFO("Raw command queued: %02X %02X %02X %02X %02X %02X %02X %02X",
			instance->_raw_data[0], instance->_raw_data[1],
			instance->_raw_data[2], instance->_raw_data[3],
			instance->_raw_data[4], instance->_raw_data[5],
			instance->_raw_data[6], instance->_raw_data[7]);
		return PX4_OK;
	}

	// 硬件寄存器诊断
	if (!strcmp(argv[0], "diag")) {
		// FDCAN2 base address for STM32H7
		const uint32_t FDCAN2_BASE = 0x4000A400;

		volatile uint32_t *ecr  = (volatile uint32_t *)(FDCAN2_BASE + 0x0040);  // Error Counter
		volatile uint32_t *psr  = (volatile uint32_t *)(FDCAN2_BASE + 0x0044);  // Protocol Status
		volatile uint32_t *ir   = (volatile uint32_t *)(FDCAN2_BASE + 0x0050);  // Interrupt Register
		volatile uint32_t *ie   = (volatile uint32_t *)(FDCAN2_BASE + 0x0054);  // Interrupt Enable
		volatile uint32_t *ils  = (volatile uint32_t *)(FDCAN2_BASE + 0x0058);  // Interrupt Line Select
		volatile uint32_t *ile  = (volatile uint32_t *)(FDCAN2_BASE + 0x005C);  // Interrupt Line Enable
		volatile uint32_t *gfc  = (volatile uint32_t *)(FDCAN2_BASE + 0x0080);  // Global Filter Config
		volatile uint32_t *sidfc = (volatile uint32_t *)(FDCAN2_BASE + 0x0084); // Std ID Filter Config
		volatile uint32_t *xidfc = (volatile uint32_t *)(FDCAN2_BASE + 0x0088); // Ext ID Filter Config
		volatile uint32_t *rxf0c = (volatile uint32_t *)(FDCAN2_BASE + 0x00A0); // Rx FIFO 0 Config
		volatile uint32_t *rxf0s = (volatile uint32_t *)(FDCAN2_BASE + 0x00A4); // Rx FIFO 0 Status
		volatile uint32_t *cccr = (volatile uint32_t *)(FDCAN2_BASE + 0x0018);  // CC Control
		volatile uint32_t *nbtp = (volatile uint32_t *)(FDCAN2_BASE + 0x001C);  // Nominal Bit Timing

		uint32_t ecr_val = *ecr;
		uint32_t psr_val = *psr;
		uint32_t ir_val = *ir;
		uint32_t ie_val = *ie;
		uint32_t ils_val = *ils;
		uint32_t ile_val = *ile;
		uint32_t gfc_val = *gfc;
		uint32_t sidfc_val = *sidfc;
		uint32_t xidfc_val = *xidfc;
		uint32_t rxf0c_val = *rxf0c;
		uint32_t rxf0s_val = *rxf0s;
		uint32_t cccr_val = *cccr;
		uint32_t nbtp_val = *nbtp;

		PX4_INFO("=== FDCAN2 Hardware Register Dump ===");
		PX4_INFO("CCCR  = 0x%08X (INIT=%d, CCE=%d, ASM=%d, FDOE=%d)",
			(unsigned)cccr_val,
			(cccr_val & 0x1) ? 1 : 0,       // INIT
			(cccr_val & 0x2) ? 1 : 0,       // CCE
			(cccr_val & 0x4) ? 1 : 0,       // ASM (Restricted Operation Mode)
			(cccr_val & 0x100) ? 1 : 0);    // FDOE
		PX4_INFO("NBTP  = 0x%08X", (unsigned)nbtp_val);
		PX4_INFO("ECR   = 0x%08X (TEC=%u, REC=%u, RP=%d, CEL=%u)",
			(unsigned)ecr_val,
			(unsigned)(ecr_val & 0xFF),                  // TEC (Transmit Error Counter)
			(unsigned)((ecr_val >> 8) & 0x7F),           // REC (Receive Error Counter)
			(int)((ecr_val >> 15) & 0x1),           // RP (Receive Error Passive)
			(unsigned)((ecr_val >> 16) & 0xFF));         // CEL (CAN Error Logging)
		PX4_INFO("PSR   = 0x%08X (LEC=%u, ACT=%u, EP=%d, EW=%d, BO=%d)",
			(unsigned)psr_val,
			(unsigned)(psr_val & 0x7),                   // LEC (Last Error Code)
			(unsigned)((psr_val >> 3) & 0x3),            // ACT (Activity)
			(int)((psr_val >> 5) & 0x1),            // EP (Error Passive)
			(int)((psr_val >> 6) & 0x1),            // EW (Warning Status)
			(int)((psr_val >> 7) & 0x1));           // BO (Bus_Off Status)
		PX4_INFO("  LEC: 0=None, 1=Stuff, 2=Form, 3=Ack, 4=Bit1, 5=Bit0, 6=CRC, 7=NoChange");
		PX4_INFO("  ACT: 0=Sync, 1=Idle, 2=Rx, 3=Tx");
		PX4_INFO("IR    = 0x%08X (RF0N=%d, RF0F=%d, RF1N=%d, TC=%d)",
			(unsigned)ir_val,
			(ir_val & 0x1) ? 1 : 0,         // RF0N (Rx FIFO 0 New Message)
			(ir_val & 0x2) ? 1 : 0,         // RF0F (Rx FIFO 0 Full)
			(ir_val & 0x10) ? 1 : 0,        // RF1N
			(ir_val & 0x200) ? 1 : 0);      // TC (Transmission Completed)
		PX4_INFO("IE    = 0x%08X", (unsigned)ie_val);
		PX4_INFO("ILS   = 0x%08X", (unsigned)ils_val);
		PX4_INFO("ILE   = 0x%08X (EINT0=%d, EINT1=%d)",
			(unsigned)ile_val,
			(ile_val & 0x1) ? 1 : 0,
			(ile_val & 0x2) ? 1 : 0);
		PX4_INFO("GFC   = 0x%08X (RRFE=%d, RRFS=%d, ANFE=%u, ANFS=%u)",
			(unsigned)gfc_val,
			(int)(gfc_val & 0x1),                   // RRFE
			(int)((gfc_val >> 1) & 0x1),            // RRFS
			(unsigned)((gfc_val >> 2) & 0x3),            // ANFE (0=FIFO0, 1=FIFO1, 2=Reject)
			(unsigned)((gfc_val >> 4) & 0x3));           // ANFS (0=FIFO0, 1=FIFO1, 2=Reject)
		PX4_INFO("  ANFE: 0=Accept to FIFO0, 1=Accept to FIFO1, 2=Reject, 3=Reject");
		PX4_INFO("SIDFC = 0x%08X (LSS=%u)", (unsigned)sidfc_val, (unsigned)((sidfc_val >> 16) & 0xFF));
		PX4_INFO("XIDFC = 0x%08X (LSE=%u)", (unsigned)xidfc_val, (unsigned)((xidfc_val >> 16) & 0x7F));
		PX4_INFO("RXF0C = 0x%08X (F0S=%u)", (unsigned)rxf0c_val, (unsigned)((rxf0c_val >> 16) & 0x7F));
		PX4_INFO("RXF0S = 0x%08X (F0FL=%u, F0GI=%u, F0PI=%u, F0F=%d, RF0L=%d)",
			(unsigned)rxf0s_val,
			(unsigned)(rxf0s_val & 0x7F),                // F0FL (Fill Level)
			(unsigned)((rxf0s_val >> 8) & 0x3F),         // F0GI (Get Index)
			(unsigned)((rxf0s_val >> 16) & 0x3F),        // F0PI (Put Index)
			(int)((rxf0s_val >> 24) & 0x1),         // F0F (FIFO Full)
			(int)((rxf0s_val >> 25) & 0x1));        // RF0L (Message Lost)

		return PX4_OK;
	}

	return print_usage("Unrecognized command");
}

extern "C" __EXPORT int dj_mk_main(int argc, char *argv[])
{
	return DjMk::main(argc, argv);
}
