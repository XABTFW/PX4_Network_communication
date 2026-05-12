/**
 * @file dm_imu_l1.cpp
 * DM-IMU L1 CAN driver implementation for PX4
 *
 * 协议说明（基于 MC02_CAN收发例程）：
 *
 * 发送请求帧：
 *   - CAN ID: 使用设备ID（例如 0x01）
 *   - 数据格式: [0xCC][reg_id][cmd][0xDD][data0][data1][data2][data3] (8字节)
 *   - cmd: 0=READ, 1=WRITE
 *
 * 接收响应帧：
 *   - CAN ID: 使用相同的设备ID（例如 0x01）
 *   - 数据格式: [data_type][reserved][payload...] (8字节)
 *   - data_type: 1=ACCEL, 2=GYRO, 3=EULER, 4=QUAT
 *   - payload 从 data[2] 开始，格式为 [low_byte][high_byte] 的 16位数据
 */

#include "dm_imu_l1.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <nuttx/net/netdev.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>

#ifndef IFF_UP
#define IFF_UP 0x1
#endif

DmImuL1::DmImuL1(uint8_t can_id, uint8_t mst_id, const char *can_dev) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_can_dev(can_dev),
	_can_id(can_id),
	_mst_id(mst_id),
	_px4_accel(0),
	_px4_gyro(0)
{
}

DmImuL1::~DmImuL1()
{
	stop();

	if (_can_fd >= 0) {
		close(_can_fd);
	}
}

int DmImuL1::init()
{
	// init() 只负责基本准备，不创建 socket
	_state = STATE_INITIALIZED;
	_last_data_time = hrt_absolute_time();

	// 清零初始化状态机变量
	_socket_open_time = 0;
	_accel_seen_after_start = false;
	_gyro_seen_after_start = false;
	_last_request_mode_cmd_time = 0;
	_request_mode_cmd_count = 0;
	_last_accel_request_time = 0;
	_last_gyro_request_time = 0;

	PX4_INFO("DM-IMU driver initialized, ready to start");

	return 0;
}

void DmImuL1::start()
{
	// 问题A修复：明确设置状态为 INITIALIZED
	_state = STATE_INITIALIZED;

	// 问题B修复：清零所有计数器和时间戳
	_run_count = 0;
	_raw_rx_count = 0;
	_accel_count = 0;
	_gyro_count = 0;
	_euler_count = 0;
	_quat_count = 0;
	_last_accel_time = 0;
	_last_gyro_time = 0;

	// 清零启动期变量
	_socket_open_time = 0;
	_accel_seen_after_start = false;
	_gyro_seen_after_start = false;
	_last_request_mode_cmd_time = 0;
	_request_mode_cmd_count = 0;
	_last_accel_request_time = 0;
	_last_gyro_request_time = 0;
	_error_count = 0;
	_error_enter_time = 0;
	_startup_retry_count = 0;

	// 清零初始化闭环控制变量
	_init_accel_request_time = 0;
	_init_gyro_request_time = 0;
	_init_round_start_time = 0;
	_init_round_count = 0;

	// 启动周期性任务，100Hz
	ScheduleOnInterval(10_ms);

	PX4_INFO("DM-IMU start: entering initialization state machine");
}

void DmImuL1::stop()
{
	ScheduleClear();

	// 关闭socket
	if (_can_fd >= 0) {
		close(_can_fd);
		_can_fd = -1;
		PX4_INFO("CAN socket closed");
	}

	// 重置所有状态
	_socket_open_time = 0;
	_accel_seen_after_start = false;
	_gyro_seen_after_start = false;
	_last_request_mode_cmd_time = 0;
	_request_mode_cmd_count = 0;
	_last_accel_request_time = 0;
	_last_gyro_request_time = 0;
	_state = STATE_INITIALIZED; // 保持初始化状态，可以再次 start
}

void DmImuL1::Run()
{
	_run_count++;
	uint64_t now = hrt_absolute_time();

	// 前3次运行用 ERROR 级别打印，确保不会被覆盖
	if (_run_count <= 3) {
		PX4_ERR("*** Run() #%lu: state=%d, _can_fd=%d ***", (unsigned long)_run_count, (int)_state, _can_fd);
	} else if (_run_count <= 10) {
		PX4_INFO("Run() #%lu, state=%d, _can_fd=%d", (unsigned long)_run_count, (int)_state, _can_fd);
	}

	// 每次状态变化时打印
	static DeviceState last_logged_state = STATE_UNINITIALIZED;
	if (_state != last_logged_state) {
		PX4_INFO("State changed: %d -> %d", (int)last_logged_state, (int)_state);
		last_logged_state = _state;
	}

	// ========== 第一步：如果 socket 未创建，先创建 ==========
	if (_can_fd < 0 && _state == STATE_INITIALIZED) {
		PX4_ERR("*** CREATING CAN SOCKET (this should appear first!) ***");
		if (open_can_socket() < 0) {
			PX4_ERR("Failed to open CAN socket");
			_state = STATE_ERROR;
			return;
		}
		PX4_ERR("*** CAN SOCKET CREATED: fd=%d ***", _can_fd);
		_socket_open_time = now;
		_state = STATE_WAIT_STABLE;
		PX4_ERR("*** ENTERING STATE_WAIT_STABLE (state=2) ***");
		return;
	}

	// socket 未创建但状态不对，报错
	if (_can_fd < 0) {
		PX4_WARN("Socket not created, state=%d", (int)_state);
		return;
	}

	// ========== 第二步：每次都调用接收，处理所有可用数据 ==========
	can_receive();

	// 刷新时间戳：can_receive() 内部用 hrt_absolute_time() 更新了
	// _last_accel_time/_last_gyro_time，可能比 Run() 顶部的 now 更新，
	// 导致无符号减法下溢触发假超时
	now = hrt_absolute_time();

	// ========== 第三步：状态机主逻辑 ==========
	switch (_state) {
	case STATE_WAIT_STABLE: {
		// 等待总线和设备稳定
		uint64_t elapsed = now - _socket_open_time;

		// 等待 300ms 让 CAN 接口和 IMU 稳定
		if (elapsed > 300000) {
			PX4_INFO("Bus stable, entering STATE_ENTER_REQUEST");
			_state = STATE_ENTER_REQUEST;
			_last_request_mode_cmd_time = 0;
			_request_mode_cmd_count = 0;
		}
		break;
	}

	case STATE_ENTER_REQUEST: {
		// 反复发送请求模式命令，每 100ms 一次，最多 5 次
		if (_last_request_mode_cmd_time == 0 || (now - _last_request_mode_cmd_time) >= 100000) {
			_request_mode_cmd_count++;
			PX4_INFO("Sending request mode command (attempt %lu)", (unsigned long)_request_mode_cmd_count);
			send_request_mode_cmd();
			_last_request_mode_cmd_time = now;
		}

		// 发够 5 次后，进入等待数据阶段
		if (_request_mode_cmd_count >= 5) {
			PX4_INFO("Request mode commands sent, entering STATE_WAIT_DATA");
			_state = STATE_WAIT_DATA;
			_last_data_time = now; // 重置超时计时起点
			// 清零初始化闭环变量，准备开始新的请求-响应轮次
			_init_round_start_time = 0;
			_init_accel_request_time = 0;
			_init_gyro_request_time = 0;
			_init_round_count = 0;
		}
		break;
	}

	case STATE_WAIT_DATA: {
		// 低频闭环请求模式：请求一次，等待响应，确认成功
		// 每一轮初始化：先请求 accel（等50ms）→ 再请求 gyro（等50ms）→ 检查结果

		// 第一次进入这个状态，初始化轮次
		if (_init_round_start_time == 0) {
			_init_round_start_time = now;
			_init_round_count = 0;
			_init_accel_request_time = 0;
			_init_gyro_request_time = 0;
			PX4_INFO("Starting WAIT_DATA: low-frequency request-response loop");
		}

		uint64_t round_elapsed = now - _init_round_start_time;

		// 阶段1：发送 accel 请求（在轮次开始时立即发送）
		if (!_accel_seen_after_start && _init_accel_request_time == 0) {
			PX4_INFO("Init round %lu: requesting accel", (unsigned long)_init_round_count);
			request_accel();
			_init_accel_request_time = now;
		}

		// 阶段2：等待 50ms 后发送 gyro 请求
		if (!_gyro_seen_after_start && _init_gyro_request_time == 0 && round_elapsed >= 50000) {
			PX4_INFO("Init round %lu: requesting gyro", (unsigned long)_init_round_count);
			request_gyro();
			_init_gyro_request_time = now;
		}

		// 阶段3：检查是否两路数据都来了
		if (_accel_seen_after_start && _gyro_seen_after_start) {
			PX4_INFO("Both accel and gyro received after %lu rounds, entering STATE_RUNNING",
				(unsigned long)_init_round_count + 1);
			_state = STATE_RUNNING;
			_error_count = 0;
			// 清零初始化闭环变量
			_init_round_start_time = 0;
			_init_accel_request_time = 0;
			_init_gyro_request_time = 0;
			break;
		}

		// 阶段4：一轮超时（150ms），开始下一轮或重试
		if (round_elapsed > 150000) {
			_init_round_count++;

			if (_accel_seen_after_start && !_gyro_seen_after_start) {
				PX4_WARN("Init round %lu timeout: only accel seen, gyro missing",
					(unsigned long)_init_round_count);
			} else if (!_accel_seen_after_start && _gyro_seen_after_start) {
				PX4_WARN("Init round %lu timeout: only gyro seen, accel missing",
					(unsigned long)_init_round_count);
			} else if (!_accel_seen_after_start && !_gyro_seen_after_start) {
				PX4_WARN("Init round %lu timeout: neither accel nor gyro seen",
					(unsigned long)_init_round_count);
			}

			// 如果已经尝试了 5 轮还没成功，重新发送请求模式命令
			if (_init_round_count >= 5) {
				PX4_WARN("Failed after %lu rounds, re-entering STATE_ENTER_REQUEST",
					(unsigned long)_init_round_count);
				_state = STATE_ENTER_REQUEST;
				_last_request_mode_cmd_time = 0;
				_request_mode_cmd_count = 0;
				_init_round_start_time = 0;
				_init_accel_request_time = 0;
				_init_gyro_request_time = 0;
				_startup_retry_count++;

				// 如果重试太多次，进入错误状态
				if (_startup_retry_count > 3) {
					PX4_ERR("Failed to initialize after %lu retries", (unsigned long)_startup_retry_count);
					_state = STATE_ERROR;
				}
			} else {
				// 开始下一轮
				_init_round_start_time = now;
				_init_accel_request_time = 0;
				_init_gyro_request_time = 0;
			}
		}
		break;
	}

	case STATE_RUNNING: {
		// 正常运行：周期性请求数据（50Hz）
		if ((now - _last_accel_request_time) >= 20000) {
			request_accel();
			_last_accel_request_time = now;
		}

		if ((now - _last_gyro_request_time) >= 20000) {
			request_gyro();
			_last_gyro_request_time = now;
		}

		// 可选：请求欧拉角数据（默认关闭，频率 10Hz）
		if (_request_euler_enabled && (now - _last_euler_request_time) >= 100000) {
			request_euler();
			_last_euler_request_time = now;
		}

		// 可选：请求四元数数据（默认关闭，频率 10Hz）
		if (_request_quat_enabled && (now - _last_quat_request_time) >= 100000) {
			request_quat();
			_last_quat_request_time = now;
		}

		// 问题F：运行期超时检查，分别检查 accel 和 gyro
		// 因为在 WAIT_DATA 阶段已经确保两路都来过，这里的判断是安全的
		bool accel_timeout = (_last_accel_time > 0) && ((now - _last_accel_time) > TIMEOUT_US);
		bool gyro_timeout = (_last_gyro_time > 0) && ((now - _last_gyro_time) > TIMEOUT_US);

		if (accel_timeout || gyro_timeout) {
			if (accel_timeout) {
				PX4_WARN("Accel timeout in RUNNING state (last update: %lu ms ago)",
					(unsigned long)((now - _last_accel_time) / 1000));
			}
			if (gyro_timeout) {
				PX4_WARN("Gyro timeout in RUNNING state (last update: %lu ms ago)",
					(unsigned long)((now - _last_gyro_time) / 1000));
			}

			// 尝试恢复：重新进入请求模式
			PX4_INFO("Attempting recovery: re-entering STATE_ENTER_REQUEST");
			_state = STATE_ENTER_REQUEST;
			_last_request_mode_cmd_time = 0;
			_request_mode_cmd_count = 0;
			_accel_seen_after_start = false;
			_gyro_seen_after_start = false;
			_error_count++;

			// 如果错误太多，进入错误状态
			if (_error_count > 10) {
				PX4_ERR("Too many errors in RUNNING state");
				_state = STATE_ERROR;
			}
		}
		break;
	}

	case STATE_ERROR: {
		// 问题C修复：使用成员变量而非 static
		// 问题D修复：恢复时清零所有启动标志
		if (_error_enter_time == 0) {
			_error_enter_time = now;
			PX4_ERR("Entered ERROR state, will retry in 1 second");
		}

		if ((now - _error_enter_time) > 1000000) {
			PX4_ERR("*** RETRYING FROM ERROR: going to STATE_INITIALIZED first ***");
			_error_enter_time = 0;

			// 先回到 INITIALIZED 状态，让下一次 Run() 重新创建 socket 和进入 WAIT_STABLE
			// 注意：不要直接设置为 WAIT_STABLE，因为需要重新设置 _socket_open_time
			_state = STATE_INITIALIZED;
			_startup_retry_count = 0;
			_error_count = 0;

			// 问题D修复：清零已收到标志
			_accel_seen_after_start = false;
			_gyro_seen_after_start = false;

			// 关闭旧的 socket，下次 Run() 会重新创建
			if (_can_fd >= 0) {
				close(_can_fd);
				_can_fd = -1;
				PX4_INFO("Closed old socket, will recreate on next Run()");
			}
		}
		break;
	}

	default:
		break;
	}
}

int DmImuL1::open_can_socket()
{
	// 首先检查网络设备状态
	FAR struct net_driver_s *dev = netdev_findbyname(_can_dev);
	if (dev == NULL) {
		PX4_ERR("CAN device '%s' not found", _can_dev);
		return -1;
	}

	PX4_INFO("%s flags before ifup: 0x%04lx (UP=%d, RUNNING=%d)",
		_can_dev, (unsigned long)dev->d_flags,
		(dev->d_flags & IFF_UP) ? 1 : 0,
		(dev->d_flags & IFF_RUNNING) ? 1 : 0);

	if ((dev->d_flags & IFF_UP) == 0) {
		PX4_WARN("%s not up, calling netdev_ifup()", _can_dev);
		int ret = netdev_ifup(dev);
		PX4_INFO("netdev_ifup(%s) ret=%d, flags after=0x%04lx",
			_can_dev, ret, (unsigned long)dev->d_flags);

		if (ret < 0) {
			PX4_ERR("Failed to bring up CAN interface: %d", ret);
			return -1;
		}

		px4_usleep(100000);
	} else {
		PX4_INFO("%s already UP", _can_dev);
	}

	// 创建socket
	_can_fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
	if (_can_fd < 0) {
		PX4_ERR("socket() failed: errno=%d (%s)", errno, strerror(errno));
		return -1;
	}

	// 准备ifreq结构
	struct ifreq ifr = {};
	snprintf(ifr.ifr_name, IFNAMSIZ, "%s", _can_dev);

	// 检查接口是否存在
	unsigned ifindex = if_nametoindex(ifr.ifr_name);
	if (ifindex == 0) {
		PX4_ERR("if_nametoindex() failed: errno=%d (%s)", errno, strerror(errno));
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}
	PX4_INFO("CAN interface '%s' found, ifindex=%u", _can_dev, ifindex);

	// 使用ioctl再次检查接口状态
	if (ioctl(_can_fd, SIOCGIFFLAGS, (unsigned long)&ifr) < 0) {
		PX4_ERR("SIOCGIFFLAGS failed: errno=%d (%s)", errno, strerror(errno));
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}

	PX4_INFO("ioctl flags: 0x%04lx (UP=%d, RUNNING=%d)",
		(unsigned long)ifr.ifr_flags,
		(ifr.ifr_flags & IFF_UP) ? 1 : 0,
		(ifr.ifr_flags & IFF_RUNNING) ? 1 : 0);

	// 关闭 CAN socket 本地 loopback（必改2）
	int loopback = 0;
	int ret_loopback = setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
	PX4_INFO("CAN_RAW_LOOPBACK disabled: ret=%d", ret_loopback);

	int recv_own_msgs = 0;
	int ret_own = setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
	PX4_INFO("CAN_RAW_RECV_OWN_MSGS disabled: ret=%d", ret_own);

	// 设置过滤器为接收所有帧
	struct can_filter filter = {};
	filter.can_id = 0;
	filter.can_mask = 0; // 接收所有帧
	if (setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
		PX4_ERR("setsockopt(CAN_RAW_FILTER) failed: errno=%d", errno);
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}
	PX4_INFO("CAN filter set to receive ALL frames");

	// 绑定
	struct sockaddr_can addr = {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifindex;
	if (bind(_can_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind() failed: errno=%d", errno);
		close(_can_fd);
		_can_fd = -1;
		return -1;
	}

	// 设置设备ID（符合PX4规范）
	// 格式: [31:24]=device_type, [23:16]=bus_type, [15:8]=bus_num, [7:0]=address
	// Device types: ACCEL=0xE1, GYRO=0xE2
	// Bus type: CAN=0x03
	uint32_t bus_type = 0x03;  // CAN
	uint32_t bus_num = 0;      // CAN0
	uint32_t address = _can_id;

	_accel_device_id = (0xE1 << 24) | (bus_type << 16) | (bus_num << 8) | address;
	_gyro_device_id = (0xE2 << 24) | (bus_type << 16) | (bus_num << 8) | address;

	PX4_INFO("CAN socket opened: fd=%d, ifindex=%u, device_id=0x%02X", _can_fd, ifindex, _can_id);
	PX4_INFO("Accel device_id: 0x%08lX, Gyro device_id: 0x%08lX",
		(unsigned long)_accel_device_id, (unsigned long)_gyro_device_id);

	// 配置 PX4 标准 IMU 发布器
	_px4_accel.set_device_id(_accel_device_id);
	_px4_accel.set_range(ACCEL_CAN_MAX);
	_px4_accel.set_scale(1.0f);
	_px4_accel.set_temperature(NAN);

	_px4_gyro.set_device_id(_gyro_device_id);
	_px4_gyro.set_range(GYRO_CAN_MAX);
	_px4_gyro.set_scale(1.0f);
	_px4_gyro.set_temperature(NAN);

	return 0;
}

int DmImuL1::can_send_cmd(uint8_t reg_id, uint8_t cmd, uint32_t data)
{
	if (_can_fd < 0) {
		PX4_WARN("Cannot send: socket not open, fd=%d", _can_fd);
		return -1;
	}

	struct can_frame frame = {};

	// 使用例程协议格式（MC02_CAN收发例程）：
	// 发送CAN ID: 使用设备ID (_can_id)
	// 数据格式: [0xCC][reg_id][cmd][0xDD][data0][data1][data2][data3]
	frame.can_id = _can_id;
	frame.can_dlc = 8;
	frame.data[0] = 0xCC;                  // 固定标识
	frame.data[1] = reg_id;                // 寄存器ID
	frame.data[2] = cmd;                   // 命令类型（READ/WRITE）
	frame.data[3] = 0xDD;                  // 固定标识
	memcpy(&frame.data[4], &data, 4);      // 数据（小端序）

	// 统计发送
	_tx_frame_count++;
	if (cmd == CMD_READ) {
		_request_sent_count++;  // 只统计读请求
	}

	// 打印发送的数据（前20次）
	static int send_count = 0;
	send_count++;
	if (send_count <= 20) {
		PX4_INFO("TX[%d]: ID=0x%03lx, DLC=%d, Data=[%02x %02x %02x %02x %02x %02x %02x %02x]",
			send_count, (unsigned long)frame.can_id, frame.can_dlc,
			frame.data[0], frame.data[1], frame.data[2], frame.data[3],
			frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
	}

	// 使用简单的send()，非阻塞模式
	ssize_t ret = send(_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);

	if (ret == sizeof(frame)) {
		return 0;
	}

	if (ret < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			// 发送缓冲区满，稍后重试
			return -EAGAIN;
		}
		PX4_ERR("send() failed: ret=%d, errno=%d (%s), pid=%d",
			(int)ret, errno, strerror(errno), getpid());
		return -1;
	}

	PX4_ERR("send() partial: ret=%d", (int)ret);
	return -1;
}

int DmImuL1::can_receive()
{
	if (_can_fd < 0) {
		return -1;
	}

	struct pollfd fds;
	fds.fd = _can_fd;
	fds.events = POLLIN;

	// 非阻塞轮询，一次处理多个消息
	// reads: 总共读取的帧数（包括无关帧）
	// imu_processed: 实际处理的 IMU 帧数
	int reads = 0;
	int imu_processed = 0;

	while (reads < 50 && imu_processed < 10) { // 最多读 50 帧，处理 10 个 IMU 帧
		errno = 0;  // 清除errno
		int ret = poll(&fds, 1, 0);

		if (ret < 0) {
			PX4_ERR("poll() failed: errno=%d (%s)", errno, strerror(errno));
			_error_count++;
			break;
		}

		if (ret == 0) {
			// 超时，没有数据，每10秒打印一次状态
			static uint64_t last_poll_log = 0;
			uint64_t now = hrt_absolute_time();
			if (now - last_poll_log > 10000000) {
				PX4_INFO("Poll timeout: ret=0, revents=0x%lx, fd=%d", (unsigned long)fds.revents, _can_fd);
				last_poll_log = now;
			}
			break;
		}

		// ret > 0, 有数据可读
		struct can_frame frame;
		ret = read(_can_fd, &frame, sizeof(struct can_frame));

		if (ret < (int)sizeof(struct can_frame)) {
			PX4_WARN("Read failed: ret=%d, errno=%d", ret, errno);
			break;
		}

		reads++; // 读取计数
		_rx_frame_count++;  // 统计总接收帧数

		// 提取 CAN ID
		uint16_t sid = frame.can_id & CAN_SFF_MASK;

		// 处理非 8 字节帧（打印日志但不处理）
		if (frame.can_dlc != 8) {
			static int non8b_count = 0;
			if (non8b_count < 10) {
				PX4_INFO("RX non-8B frame: ID=0x%03x DLC=%d", sid, frame.can_dlc);
				non8b_count++;
			}
			continue;
		}

		// 提取数据类型
		uint8_t data_type = frame.data[0];

		// 必改1：接收侧只认 0x11（_mst_id），不再认 _can_id
		bool candidate = (sid == _mst_id);

		// 打印原始数据（前 100 帧），标记候选帧
		if (_raw_rx_count < 100) {
			PX4_INFO("RX RAW[%lu]: ID=0x%03x DLC=%d candidate=%d Data=[%02x %02x %02x %02x %02x %02x %02x %02x]",
				(unsigned long)_raw_rx_count,
				sid, frame.can_dlc, candidate ? 1 : 0,
				frame.data[0], frame.data[1], frame.data[2], frame.data[3],
				frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
			_raw_rx_count++;
		}

		// 候选帧但 type 不合法
		if (candidate && !(data_type >= ACCEL_DATA && data_type <= QUAT_DATA)) {
			PX4_INFO("RX candidate unknown type: ID=0x%03x type=0x%02x", sid, data_type);
			continue;
		}

		// 非候选帧但 type 看起来像 IMU
		if (!candidate && data_type >= ACCEL_DATA && data_type <= QUAT_DATA) {
			static int unexpected_imu_count = 0;
			if (unexpected_imu_count < 10) {
				PX4_INFO("RX possible IMU on unexpected ID: 0x%03x type=%u", sid, data_type);
				unexpected_imu_count++;
			}
			continue;
		}

		// 只处理候选帧，且 data_type 在有效范围内
		if (candidate && data_type >= ACCEL_DATA && data_type <= QUAT_DATA) {
			// 必改5：不要只靠 _last_data_time，分开更新 accel/gyro 时间
			// 这里先更新通用时间，具体的 accel/gyro 时间在各自解析函数里更新
			_last_data_time = hrt_absolute_time();
			_error_count = 0;

			// 解析数据帧
			switch (data_type) {
			case ACCEL_DATA:
				parse_accel_data(frame.data);
				break;

			case GYRO_DATA:
				parse_gyro_data(frame.data);
				break;

			case EULER_DATA:
				parse_euler_data(frame.data);
				break;

			case QUAT_DATA:
				parse_quat_data(frame.data);
				break;

			default:
				break;
			}

			imu_processed++; // 只有真正处理了 IMU 帧才计数
		}
	}

	// 必改3：在日志里打印 accel/gyro 分开的统计信息
	static uint64_t last_status_log = 0;
	uint64_t now = hrt_absolute_time();
	if (now - last_status_log > 5000000) { // 每 5 秒打印一次状态
		uint64_t accel_age = _last_accel_time > 0 ? (now - _last_accel_time) / 1000 : 999999;
		uint64_t gyro_age = _last_gyro_time > 0 ? (now - _last_gyro_time) / 1000 : 999999;
		PX4_INFO("IMU Status: accel_count=%lu (age=%lu ms), gyro_count=%lu (age=%lu ms)",
			(unsigned long)_accel_count, (unsigned long)accel_age,
			(unsigned long)_gyro_count, (unsigned long)gyro_age);
		last_status_log = now;
	}

	return imu_processed;
}

void DmImuL1::parse_accel_data(const uint8_t *data)
{
	// 统计响应
	_response_count++;

	// 数据格式: [data_type][reserved][accel_x_L][accel_x_H][accel_y_L][accel_y_H][accel_z_L][accel_z_H]
	// 实际数据从索引2开始
	uint16_t accel_raw[3];

	accel_raw[0] = (data[3] << 8) | data[2];
	accel_raw[1] = (data[5] << 8) | data[4];
	accel_raw[2] = (data[7] << 8) | data[6];

	_accel[0] = uint_to_float(accel_raw[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	_accel[1] = uint_to_float(accel_raw[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	_accel[2] = uint_to_float(accel_raw[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);

	const hrt_abstime now = hrt_absolute_time();

	// 更新 accel 专用时间戳
	_last_accel_time = now;

	// 标记启动后已收到 accel（用于初始化状态机）
	if (!_accel_seen_after_start) {
		_accel_seen_after_start = true;
		PX4_INFO("First accel data received after start");
	}

	_px4_accel.set_error_count(_error_count);
	_px4_accel.update(now, _accel[0], _accel[1], _accel[2]);

	_accel_count++;

	// 打印解析后的数据（前20次），核对静止时是否正常
	if (_accel_count <= 20) {
		PX4_INFO("ACC parsed[%lu]: x=%.3f y=%.3f z=%.3f (expect: x≈0, y≈0, z≈±9.8)",
			(unsigned long)_accel_count,
			(double)_accel[0], (double)_accel[1], (double)_accel[2]);
	}
}

void DmImuL1::parse_gyro_data(const uint8_t *data)
{
	// 统计响应
	_response_count++;

	// 数据格式: [data_type][reserved][gyro_x_L][gyro_x_H][gyro_y_L][gyro_y_H][gyro_z_L][gyro_z_H]
	uint16_t gyro_raw[3];

	gyro_raw[0] = (data[3] << 8) | data[2];
	gyro_raw[1] = (data[5] << 8) | data[4];
	gyro_raw[2] = (data[7] << 8) | data[6];

	_gyro[0] = uint_to_float(gyro_raw[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	_gyro[1] = uint_to_float(gyro_raw[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	_gyro[2] = uint_to_float(gyro_raw[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);

	const hrt_abstime now = hrt_absolute_time();

	// 更新 gyro 专用时间戳
	_last_gyro_time = now;

	// 标记启动后已收到 gyro（用于初始化状态机）
	if (!_gyro_seen_after_start) {
		_gyro_seen_after_start = true;
		PX4_INFO("First gyro data received after start");
	}

	_px4_gyro.set_error_count(_error_count);
	_px4_gyro.update(now, _gyro[0], _gyro[1], _gyro[2]);

	_gyro_count++;

	// 打印解析后的数据（前20次）
	if (_gyro_count <= 20) {
		PX4_INFO("GYRO parsed[%lu]: %.3f %.3f %.3f", (unsigned long)_gyro_count,
			(double)_gyro[0], (double)_gyro[1], (double)_gyro[2]);
	}
}

void DmImuL1::parse_euler_data(const uint8_t *data)
{
	// 统计响应（如果启用了欧拉角请求）
	if (_request_euler_enabled) {
		_response_count++;
	}

	// 数据格式: [data_type][reserved][pitch_L][pitch_H][yaw_L][yaw_H][roll_L][roll_H]
	uint16_t euler_raw[3];

	euler_raw[0] = (data[3] << 8) | data[2];
	euler_raw[1] = (data[5] << 8) | data[4];
	euler_raw[2] = (data[7] << 8) | data[6];

	_euler[0] = uint_to_float(euler_raw[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16); // pitch
	_euler[1] = uint_to_float(euler_raw[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);     // yaw
	_euler[2] = uint_to_float(euler_raw[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);   // roll

	_euler_count++;

	// 打印前 20 次欧拉角数据
	if (_euler_count <= 20) {
		PX4_INFO("EULER parsed[%lu]: pitch=%.2f° yaw=%.2f° roll=%.2f°",
			(unsigned long)_euler_count,
			(double)_euler[0], (double)_euler[1], (double)_euler[2]);
	}

	// 欧拉角数据仅用于内部调试，不发布到系统姿态链路
}

void DmImuL1::parse_quat_data(const uint8_t *data)
{
	// 统计响应（如果启用了四元数请求）
	if (_request_quat_enabled) {
		_response_count++;
	}

	// 四元数解析（压缩格式，每个分量14位）
	// 数据格式: [data_type][w13:6][w5:0,x13:8][x7:0][y13:6][y5:0,z13:8][z7:0]
	// 根据厂家例程 dm_imu.c 第223行，数据从 data[1] 开始
	int w = (data[1] << 6) | ((data[2] & 0xF8) >> 2);
	int x = ((data[2] & 0x03) << 12) | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
	int y = ((data[4] & 0x0F) << 10) | (data[5] << 2) | ((data[6] & 0xC0) >> 6);
	int z = ((data[6] & 0x3F) << 8) | data[7];

	// 转换为浮点数
	_quat[0] = uint_to_float(w, -1.0f, 1.0f, 14); // w
	_quat[1] = uint_to_float(x, -1.0f, 1.0f, 14); // x
	_quat[2] = uint_to_float(y, -1.0f, 1.0f, 14); // y
	_quat[3] = uint_to_float(z, -1.0f, 1.0f, 14); // z

	_quat_count++;

	// 打印前 20 次四元数数据
	if (_quat_count <= 20) {
		PX4_INFO("QUAT parsed[%lu]: w=%.4f x=%.4f y=%.4f z=%.4f",
			(unsigned long)_quat_count,
			(double)_quat[0], (double)_quat[1], (double)_quat[2], (double)_quat[3]);
	}

	// 四元数数据仅用于内部调试，不发布到系统姿态链路
}

float DmImuL1::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ============ 配置和校准功能 ============

void DmImuL1::calibrate_accel()
{
	PX4_INFO("Starting accelerometer calibration...");
	can_send_cmd(ACCEL_CALI, CMD_WRITE, 0);
}

void DmImuL1::calibrate_gyro()
{
	PX4_INFO("Starting gyroscope calibration...");
	can_send_cmd(GYRO_CALI, CMD_WRITE, 0);
}

void DmImuL1::calibrate_mag()
{
	PX4_INFO("Starting magnetometer calibration...");
	can_send_cmd(MAG_CALI, CMD_WRITE, 0);
}

void DmImuL1::set_zero()
{
	PX4_INFO("Setting zero position...");
	can_send_cmd(SET_ZERO, CMD_WRITE, 0);
}

void DmImuL1::reboot()
{
	PX4_INFO("Rebooting IMU...");
	can_send_cmd(REBOOT_IMU, CMD_WRITE, 0);

	// 重启后需要重新初始化
	_state = STATE_UNINITIALIZED;

	// 重置所有初始化状态机变量
	_socket_open_time = 0;
	_accel_seen_after_start = false;
	_gyro_seen_after_start = false;
	_last_request_mode_cmd_time = 0;
	_request_mode_cmd_count = 0;
	_last_accel_request_time = 0;
	_last_gyro_request_time = 0;
	_startup_retry_count = 0;
}

// ============ 参数配置功能 ============

void DmImuL1::set_baudrate(ImuBaudrate baud)
{
	PX4_INFO("Setting CAN baudrate to %d", (int)baud);
	can_send_cmd(SET_BAUD, CMD_WRITE, (uint32_t)baud);
}

void DmImuL1::set_can_id(uint8_t id)
{
	PX4_INFO("Setting CAN ID to 0x%02X", id);
	can_send_cmd(SET_CAN_ID, CMD_WRITE, id);
	_can_id = id; // 更新本地ID
}

void DmImuL1::set_mst_id(uint8_t id)
{
	PX4_INFO("Setting Master ID to 0x%02X", id);
	can_send_cmd(SET_MST_ID, CMD_WRITE, id);
	_mst_id = id; // 更新本地ID
}

void DmImuL1::set_com_port(ImuComPort port)
{
	PX4_INFO("Setting communication port to %d", (int)port);
	can_send_cmd(CHANGE_COM, CMD_WRITE, (uint32_t)port);
}

void DmImuL1::set_active_delay(uint32_t delay_ms)
{
	PX4_INFO("Setting active mode delay to %lu ms", (unsigned long)delay_ms);
	can_send_cmd(SET_DELAY, CMD_WRITE, delay_ms);
}

void DmImuL1::save_parameters()
{
	PX4_INFO("Saving parameters to flash...");
	can_send_cmd(SAVE_PARAM, CMD_WRITE, 0);
}

void DmImuL1::restore_factory_settings()
{
	PX4_INFO("Restoring factory settings...");
	can_send_cmd(RESTORE_SETTING, CMD_WRITE, 0);
}

// ============ 模式切换功能 ============

void DmImuL1::change_to_active_mode()
{
	PX4_INFO("Switching to active mode (not used in current state machine)");
	can_send_cmd(CHANGE_ACTIVE, CMD_WRITE, 1);
	// 注意：新状态机不使用 active mode，保留此函数仅供手动调试
}

void DmImuL1::send_request_mode_cmd()
{
	PX4_INFO("Sending request mode command to IMU");
	int ret = can_send_cmd(CHANGE_ACTIVE, CMD_WRITE, 0);

	if (ret == 0) {
		PX4_INFO("Request mode command sent successfully");
	} else {
		PX4_WARN("Request mode command send failed: %d", ret);
	}
}

void DmImuL1::change_to_request_mode()
{
	PX4_INFO("Manually switching to request mode");
	send_request_mode_cmd();
	// 手动触发状态机重新初始化
	_state = STATE_ENTER_REQUEST;
	_last_request_mode_cmd_time = 0;
	_request_mode_cmd_count = 0;
	_accel_seen_after_start = false;
	_gyro_seen_after_start = false;
}

// ============ 数据请求功能（被动模式）============

void DmImuL1::request_accel()
{
	can_send_cmd(ACCEL_DATA, CMD_READ, 0);
}

void DmImuL1::request_gyro()
{
	can_send_cmd(GYRO_DATA, CMD_READ, 0);
}

void DmImuL1::request_euler()
{
	can_send_cmd(EULER_DATA, CMD_READ, 0);
}

void DmImuL1::request_quat()
{
	can_send_cmd(QUAT_DATA, CMD_READ, 0);
}

// ============ 错误处理功能 ============

void DmImuL1::handle_timeout()
{
	if (_error_count < 10) {
		_error_count++;
	}
	PX4_WARN("IMU data timeout (error count: %lu)", (unsigned long)_error_count);

	if (_error_count >= 10) {
		_state = STATE_ERROR;
		PX4_ERR("Too many errors, entering error state");
	} else {
		// 尝试恢复：重新进入请求模式初始化
		PX4_INFO("Timeout recovery: re-entering STATE_ENTER_REQUEST");
		_state = STATE_ENTER_REQUEST;
		_last_request_mode_cmd_time = 0;
		_request_mode_cmd_count = 0;
		_accel_seen_after_start = false;
		_gyro_seen_after_start = false;
	}
}

// ============ 删除 sensor_combined 发布函数 ============
// sensor_combined 应该由系统自动生成，不应该在驱动层发布
