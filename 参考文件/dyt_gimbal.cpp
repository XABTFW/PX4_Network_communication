/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/dyt_command.h>
#include <uORB/topics/dyt_target.h>
#include <uORB/topics/parameter_update.h>

#include <lib/mathlib/mathlib.h>

using namespace time_literals;

class DytGimbal : public ModuleBase<DytGimbal>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit DytGimbal(const char *device_path);
	~DytGimbal() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void show_status();

private:
	static constexpr uint8_t FRAME_SYNC_1{0xEE};
	static constexpr uint8_t FRAME_SYNC_2{0x16};
	static constexpr size_t FRAME_LEN{32};
	static constexpr size_t COMMAND_LEN{16};

	void Run() override;

	bool open_serial();
	void close_serial();
	int configure_serial(int fd, int baud);
	speed_t baud_to_speed(int baud) const;

	void read_serial();
	void process_byte(uint8_t byte);
	bool validate_frame(const uint8_t *frame) const;
	void handle_frame(const uint8_t *frame, hrt_abstime now);
	void publish_link_state(hrt_abstime now, uint8_t tracking_state);

	void handle_command_updates();
	void send_protocol_command(const dyt_command_s &cmd);
	void send_command_frame(uint8_t control, int16_t param_x, int16_t param_y, uint8_t param3, int8_t zoom_rate);

	void update_params_if_needed();

	int _uart_fd{-1};
	char _device_path[32]{};
	uint8_t _frame[FRAME_LEN]{};
	size_t _frame_index{0};
	bool _awaiting_sync_2{false};

	hrt_abstime _last_rx_time{0};
	hrt_abstime _last_open_attempt{0};
	hrt_abstime _last_state_publish{0};

	uint32_t _frame_counter{0};
	uint16_t _parse_error_count{0};
	uint16_t _read_error_count{0};

	dyt_target_s _last_target{};

	uORB::Subscription _dyt_command_sub{ORB_ID(dyt_command)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<dyt_target_s> _dyt_target_pub{ORB_ID(dyt_target)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DYT_BAUD>) _param_dyt_baud,
		(ParamInt<px4::params::DYT_TO_MS>) _param_dyt_timeout_ms,
		(ParamInt<px4::params::DYT_RTRY_MS>) _param_dyt_retry_ms
	);
};

/**
 * @brief 构造 DYT 吊舱驱动模块实例。
 *
 * @param device_path 串口设备路径字符串。
 * 输出：初始化驱动对象、工作队列绑定和内部缓存状态。
 * 功能：保存串口路径，清空最近一次目标缓存，并设置默认距离为 NAN。
 */
DytGimbal::DytGimbal(const char *device_path) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_device_path, device_path, sizeof(_device_path) - 1);
	_device_path[sizeof(_device_path) - 1] = '\0';
	memset(&_last_target, 0, sizeof(_last_target));
	_last_target.range_m = NAN;
}

/**
 * @brief 析构 DYT 吊舱驱动模块实例。
 *
 * 输入：无。
 * 输出：关闭已打开的串口设备。
 * 功能：在模块销毁时释放串口资源，避免文件描述符泄漏。
 */
DytGimbal::~DytGimbal()
{
	close_serial();
}

/**
 * @brief 初始化模块的周期调度。
 *
 * 输入：无。
 * @return 调度配置完成后返回 true。
 * 功能：以 5 ms 周期启动驱动主循环。
 */
bool DytGimbal::init()
{
	ScheduleOnInterval(5_ms);
	return true;
}

/**
 * @brief 打印当前驱动状态信息。
 *
 * 输入：缓存的串口、收帧和错误统计状态。
 * 输出：向 PX4 日志打印状态诊断信息。
 * 功能：便于查看串口是否打开、接收帧数量和解析/读取错误情况。
 */
void DytGimbal::show_status()
{
	PX4_INFO("port: %s", _device_path);
	PX4_INFO("fd: %d", _uart_fd);
	PX4_INFO("frames: %lu", static_cast<unsigned long>(_frame_counter));
	PX4_INFO("parse errors: %u", _parse_error_count);
	PX4_INFO("read errors: %u", _read_error_count);
	PX4_INFO("last rx: %.3f s", static_cast<double>(_last_rx_time > 0 ?
			(hrt_absolute_time() - _last_rx_time) * 1e-6 : -1.0));
}

/**
 * @brief 执行一次驱动主循环。
 *
 * 输入：参数更新、串口状态、串口接收数据和上游命令更新。
 * 输出：可能打开/关闭串口，发布 `dyt_target`，并向吊舱发送控制命令。
 * 功能：统一处理设备重连、串口读包、命令发送和链路超时状态发布。
 */
void DytGimbal::Run()
{
	if (should_exit()) {
		ScheduleClear();
		close_serial();
		exit_and_cleanup();
		return;
	}

	update_params_if_needed();

	const hrt_abstime now = hrt_absolute_time();

	if (_uart_fd < 0) {
		if ((now - _last_open_attempt) >= static_cast<hrt_abstime>(_param_dyt_retry_ms.get()) * 1000ULL) {
			_last_open_attempt = now;
			open_serial();
		}

		publish_link_state(now, dyt_target_s::TRACKING_STATE_TIMEOUT);
		return;
	}

	read_serial();
	handle_command_updates();

	const hrt_abstime timeout_us = static_cast<hrt_abstime>(_param_dyt_timeout_ms.get()) * 1000ULL;

	if ((_last_rx_time == 0) || ((now - _last_rx_time) > timeout_us)) {
		publish_link_state(now, dyt_target_s::TRACKING_STATE_TIMEOUT);
	}
}

/**
 * @brief 在收到参数更新时刷新驱动参数。
 *
 * 输入：来自 `parameter_update` 的通知。
 * 输出：更新本地参数缓存；若串口已打开则重新配置波特率。
 * 功能：支持运行中热更新串口相关参数。
 */
void DytGimbal::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();

		if (_uart_fd >= 0) {
			configure_serial(_uart_fd, _param_dyt_baud.get());
		}
	}
}

/**
 * @brief 打开并初始化串口设备。
 *
 * 输入：缓存的设备路径和当前配置波特率参数。
 * @return 串口打开并配置成功时返回 true，否则返回 false。
 * 功能：建立与吊舱的 UART 连接，并应用原始串口配置。
 */
bool DytGimbal::open_serial()
{
	_uart_fd = ::open(_device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_WARN("open %s failed (%d)", _device_path, errno);
		return false;
	}

	if (configure_serial(_uart_fd, _param_dyt_baud.get()) != PX4_OK) {
		close_serial();
		return false;
	}

	PX4_INFO("opened %s @ %d", _device_path, _param_dyt_baud.get());
	return true;
}

/**
 * @brief 关闭当前串口设备。
 *
 * 输入：无。
 * 输出：关闭文件描述符并将其复位为 -1。
 * 功能：在错误、退出或重连前释放串口资源。
 */
void DytGimbal::close_serial()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

/**
 * @brief 将整型波特率转换为系统串口速度常量。
 *
 * @param baud 配置的整型波特率。
 * @return 返回对应的 `speed_t` 常量；不支持时返回 0。
 * 功能：把参数系统中的波特率配置映射到 termios 所需格式。
 */
speed_t DytGimbal::baud_to_speed(int baud) const
{
	switch (baud) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
#ifdef B230400
	case 230400: return B230400;
#endif
#ifdef B460800
	case 460800: return B460800;
#endif
#ifdef B921600
	case 921600: return B921600;
#endif
	default: return 0;
	}
}

/**
 * @brief 配置串口的 termios 参数。
 *
 * @param fd 已打开的串口文件描述符。
 * @param baud 期望配置的波特率。
 * @return 配置成功返回 PX4_OK，否则返回 PX4_ERROR。
 * 功能：将串口设置为 8N1、raw 模式、无流控和非阻塞读取。
 */
int DytGimbal::configure_serial(int fd, int baud)
{
	const speed_t speed = baud_to_speed(baud);

	if (speed == 0) {
		PX4_ERR("unsupported baud %d", baud);
		return PX4_ERROR;
	}

	struct termios uart_config {};

	if (tcgetattr(fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr failed (%d)", errno);
		return PX4_ERROR;
	}

	cfmakeraw(&uart_config);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
	uart_config.c_cflag &= ~CRTSCTS;
#endif
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("baud config failed (%d)", errno);
		return PX4_ERROR;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed (%d)", errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * @brief 从串口持续读取可用字节流。
 *
 * 输入：当前已打开的串口文件描述符。
 * 输出：把读到的每个字节交给帧解析器；发生读错误时关闭串口并增加错误计数。
 * 功能：批量读取串口数据并驱动协议字节流解析。
 */
void DytGimbal::read_serial()
{
	uint8_t buffer[128]{};

	for (;;) {
		const ssize_t nread = ::read(_uart_fd, buffer, sizeof(buffer));
		bool would_block = false;

		if (nread < 0) {
			would_block = (errno == EAGAIN);
#if defined(EWOULDBLOCK) && (EWOULDBLOCK != EAGAIN)
			would_block = would_block || (errno == EWOULDBLOCK);
#endif
		}

		if (nread > 0) {
			for (ssize_t i = 0; i < nread; ++i) {
				process_byte(buffer[i]);
			}

		} else if (nread == 0 || would_block) {
			break;

		} else {
			++_read_error_count;
			close_serial();
			break;
		}
	}
}

/**
 * @brief 逐字节推进 DYT 协议帧解析状态机。
 *
 * @param byte 当前收到的一个字节。
 * 输出：更新帧缓存、同步头状态和解析错误统计；完整帧到达时触发校验与解码。
 * 功能：在字节流中识别完整的 32 字节 DYT 遥测帧。
 */
void DytGimbal::process_byte(uint8_t byte)
{
	if (_frame_index == 0) {
		if (byte == FRAME_SYNC_1) {
			_frame[_frame_index++] = byte;
			_awaiting_sync_2 = true;
		}

		return;
	}

	if (_awaiting_sync_2) {
		if (byte == FRAME_SYNC_2) {
			_frame[_frame_index++] = byte;
			_awaiting_sync_2 = false;

		} else if (byte == FRAME_SYNC_1) {
			_frame[0] = FRAME_SYNC_1;
			_frame_index = 1;

		} else {
			_frame_index = 0;
			_awaiting_sync_2 = false;
		}

		return;
	}

	_frame[_frame_index++] = byte;

	if (_frame_index == FRAME_LEN) {
		const hrt_abstime now = hrt_absolute_time();

		if (validate_frame(_frame)) {
			handle_frame(_frame, now);

		} else {
			++_parse_error_count;
			publish_link_state(now, dyt_target_s::TRACKING_STATE_ERROR);
		}

		_frame_index = 0;
		_awaiting_sync_2 = false;
	}
}

/**
 * @brief 校验一帧 DYT 协议数据的校验和。
 *
 * @param frame 指向待校验的完整帧缓冲区。
 * @return 校验通过返回 true，否则返回 false。
 * 功能：对前 31 个字节累加求和，并与最后一个校验字节比较。
 */
bool DytGimbal::validate_frame(const uint8_t *frame) const
{
	uint8_t checksum = 0;

	for (size_t i = 0; i < FRAME_LEN - 1; ++i) {
		checksum = static_cast<uint8_t>(checksum + frame[i]);
	}

	return checksum == frame[FRAME_LEN - 1];
}

/**
 * @brief 将一帧合法协议数据解码为 `dyt_target` 消息。
 *
 * @param frame 已通过校验的完整协议帧。
 * @param now 当前高精度时间戳。
 * 输出：更新最近接收时间和目标缓存，并发布一条 `dyt_target`。
 * 功能：把原始协议字段解码为带物理语义的目标跟踪状态、LOS、姿态和附加状态信息。
 */
void DytGimbal::handle_frame(const uint8_t *frame, hrt_abstime now)
{
	const auto u16_at = [frame](size_t index) -> uint16_t {
		return static_cast<uint16_t>(frame[index]) | (static_cast<uint16_t>(frame[index + 1]) << 8);
	};

	const auto s16_at = [&u16_at](size_t index) -> int16_t {
		return static_cast<int16_t>(u16_at(index));
	};

	dyt_target_s target{};
	target.timestamp = now;
	target.timestamp_sample = now;
	target.frame_counter = ++_frame_counter;

	target.status1 = frame[2];
	target.status2 = frame[3];
	target.status3 = frame[5];
	target.self_test_raw = frame[28];
	target.parse_error_count = _parse_error_count;

	target.video_source = (frame[2] >> 6) & 0x3;
	target.tracking_algorithm = (frame[2] >> 4) & 0x3;
	target.auto_hint = (frame[2] & (1 << 3)) != 0;
	target.tracking_state = (frame[2] & (1 << 2)) ? dyt_target_s::TRACKING_STATE_LOCKED :
				dyt_target_s::TRACKING_STATE_SEARCH;
	target.target_valid = target.tracking_state == dyt_target_s::TRACKING_STATE_LOCKED;

	target.image_enhance = (frame[3] & (1 << 7)) != 0;
	target.recording = (frame[3] & (1 << 5)) != 0;
	target.motor_on = (frame[3] & (1 << 3)) != 0;
	target.follow_mode = (frame[3] & (1 << 2)) != 0;
	target.laser_on = (frame[3] & (1 << 0)) != 0;

	const uint16_t zoom_raw = static_cast<uint16_t>(frame[4]) | (static_cast<uint16_t>(frame[5] & 0x0F) << 8);
	target.zoom_ratio = static_cast<float>(zoom_raw) * 0.1f;

	target.los_x_rad = math::radians(static_cast<float>(s16_at(6)) * 0.05f);
	target.los_y_rad = math::radians(static_cast<float>(s16_at(8)) * 0.05f);

	target.gimbal_roll_rad = math::radians(static_cast<float>(s16_at(10)) * 0.01f);
	target.gimbal_pitch_rad = math::radians(static_cast<float>(s16_at(12)) * 0.01f);
	target.gimbal_yaw_rad = math::radians(static_cast<float>(s16_at(14)) * 0.01f);

	target.bbox_width_px = static_cast<float>(frame[16]) * 4.f;
	target.bbox_height_px = static_cast<float>(frame[17]) * 4.f;

	target.gimbal_roll_rate_rad_s = math::radians(static_cast<float>(s16_at(20)) * 0.01f);
	target.gimbal_pitch_rate_rad_s = math::radians(static_cast<float>(s16_at(22)) * 0.01f);
	target.gimbal_yaw_rate_rad_s = math::radians(static_cast<float>(s16_at(24)) * 0.01f);

	const uint16_t range_raw = u16_at(26);
	target.range_m = (range_raw > 0) ? static_cast<float>(range_raw) * 0.1f : NAN;

	target.selftest_done = (frame[28] & (1 << 7)) != 0;
	target.gyro_calib_failed = (frame[28] & (1 << 2)) != 0;
	target.servo_fault = (frame[28] & (1 << 1)) != 0;
	target.image_board_fault = (frame[28] & (1 << 0)) != 0;

	target.frame_dt_s = (_last_rx_time > 0) ? (now - _last_rx_time) * 1e-6f : 0.f;
	target.last_rx_age_s = 0.f;

	_last_rx_time = now;
	_last_state_publish = now;
	_last_target = target;
	_dyt_target_pub.publish(target);
}

/**
 * @brief 在链路异常或超时时发布占位目标状态。
 *
 * @param now 当前高精度时间戳。
 * @param tracking_state 要发布的跟踪状态码，通常为 TIMEOUT 或 ERROR。
 * 输出：按节流频率发布一条 `dyt_target` 状态消息。
 * 功能：即使链路中断，也持续向下游报告目标不可用和最近接收年龄。
 */
void DytGimbal::publish_link_state(hrt_abstime now, uint8_t tracking_state)
{
	if ((now - _last_state_publish) < 100_ms) {
		return;
	}

	dyt_target_s target = _last_target;
	target.timestamp = now;
	target.tracking_state = tracking_state;
	target.target_valid = false;
	target.timestamp_sample = _last_rx_time;
	target.frame_counter = _frame_counter;
	target.parse_error_count = _parse_error_count;
	target.last_rx_age_s = (_last_rx_time > 0) ? (now - _last_rx_time) * 1e-6f : NAN;

	_dyt_target_pub.publish(target);
	_last_state_publish = now;
}

/**
 * @brief 处理所有待发送的吊舱命令。
 *
 * 输入：`dyt_command` 订阅中的最新命令消息。
 * 输出：把每条命令转发到协议命令发送函数。
 * 功能：消费上游模块发来的控制请求，并逐条下发给吊舱。
 */
void DytGimbal::handle_command_updates()
{
	dyt_command_s cmd{};

	while (_dyt_command_sub.update(&cmd)) {
		send_protocol_command(cmd);
	}
}

/**
 * @brief 将抽象的 `dyt_command` 映射成具体协议控制命令。
 *
 * @param cmd 上游发送的吊舱控制命令消息。
 * 输出：调用底层命令帧发送函数，向串口写入控制帧。
 * 功能：把自动锁定、停止跟踪和重新触发等高层命令转换为 DYT 协议控制字。
 */
void DytGimbal::send_protocol_command(const dyt_command_s &cmd)
{
	switch (cmd.command) {
	case dyt_command_s::CMD_AUTO_LOCK: {
			const int16_t param_x = (cmd.param_x != 0) ? cmd.param_x : -100;
			send_command_frame(0x0F, param_x, cmd.param_y, cmd.param3, cmd.zoom_rate);
		}
		break;

	case dyt_command_s::CMD_STOP_TRACK:
		send_command_frame(0x0E, cmd.param_x, cmd.param_y, cmd.param3, cmd.zoom_rate);
		break;

	case dyt_command_s::CMD_RETRIGGER:
		send_command_frame(0x0E, 0, 0, 0, 0);
		send_command_frame(0x0F, (cmd.param_x != 0) ? cmd.param_x : -100, cmd.param_y, cmd.param3, cmd.zoom_rate);
		break;

	case dyt_command_s::CMD_CENTER_GIMBAL:
		send_command_frame(0x26, cmd.param_x, cmd.param_y, cmd.param3, cmd.zoom_rate);
		break;

	default:
		break;
	}
}

/**
 * @brief 组装并发送一帧 DYT 串口控制命令。
 *
 * @param control DYT 协议控制字。
 * @param param_x 命令 X 参数。
 * @param param_y 命令 Y 参数。
 * @param param3 命令附加参数。
 * @param zoom_rate 缩放速率参数。
 * 输出：向串口写入一帧 16 字节命令；写失败时增加错误计数。
 * 功能：完成控制帧编码、校验和计算以及最终串口发送。
 */
void DytGimbal::send_command_frame(uint8_t control, int16_t param_x, int16_t param_y, uint8_t param3, int8_t zoom_rate)
{
	if (_uart_fd < 0) {
		return;
	}

	uint8_t buffer[COMMAND_LEN]{};
	buffer[0] = 0xEB;
	buffer[1] = 0x90;
	buffer[2] = control;
	buffer[3] = static_cast<uint8_t>(param_x & 0xFF);
	buffer[4] = static_cast<uint8_t>((param_x >> 8) & 0xFF);
	buffer[5] = static_cast<uint8_t>(param_y & 0xFF);
	buffer[6] = static_cast<uint8_t>((param_y >> 8) & 0xFF);
	buffer[7] = param3;
	buffer[8] = static_cast<uint8_t>(zoom_rate);

	for (size_t i = 0; i < COMMAND_LEN - 1; ++i) {
		buffer[COMMAND_LEN - 1] = static_cast<uint8_t>(buffer[COMMAND_LEN - 1] + buffer[i]);
	}

	const ssize_t written = ::write(_uart_fd, buffer, sizeof(buffer));

	if (written != static_cast<ssize_t>(sizeof(buffer))) {
		++_read_error_count;
	}
}

/**
 * @brief 创建并启动吊舱驱动模块实例。
 *
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return 启动成功返回 PX4_OK，否则返回 PX4_ERROR。
 * 功能：解析设备路径参数，创建模块单例并启动周期调度。
 */
int DytGimbal::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS3";

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			return print_usage("unknown option");
		}
	}

	DytGimbal *instance = new DytGimbal(device);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

/**
 * @brief 处理模块自定义命令行命令。
 *
 * @param argc 命令参数个数。
 * @param argv 命令参数数组。
 * @return 命令处理成功时返回 PX4_OK，否则返回 usage 的结果。
 * 功能：支持状态查看、手动自动锁定和手动停止跟踪命令。
 */
int DytGimbal::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return print_usage("module not running");
	}

	if (!strcmp(argv[0], "status")) {
		get_instance()->show_status();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "autolock")) {
		dyt_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = dyt_command_s::CMD_AUTO_LOCK;
		cmd.param_x = -100;
		get_instance()->send_protocol_command(cmd);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "stoptrk")) {
		dyt_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = dyt_command_s::CMD_STOP_TRACK;
		get_instance()->send_protocol_command(cmd);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "center")) {
		dyt_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = dyt_command_s::CMD_CENTER_GIMBAL;
		get_instance()->send_protocol_command(cmd);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

/**
 * @brief 打印模块帮助信息和命令用法。
 *
 * @param reason 可选原因，会在帮助文本前打印；可以为 nullptr。
 * @return 打印完成后固定返回 0。
 * 功能：向 PX4 shell 输出驱动用途、参数和支持的命令列表。
 */
int DytGimbal::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
DYT UART driver for periodic telemetry parsing and one-shot tracking commands.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dyt_gimbal", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("-d <device>", "UART device path, default /dev/ttyS3", true);
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("autolock");
	PRINT_MODULE_USAGE_COMMAND("stoptrk");
	PRINT_MODULE_USAGE_COMMAND("center");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

/**
 * @brief 供 PX4 shell 调用的模块 C 入口函数。
 *
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return 返回 `DytGimbal::main` 的执行结果。
 * 功能：将 PX4 shell 的调用转发到 C++ 驱动模块入口。
 */
extern "C" __EXPORT int dyt_gimbal_main(int argc, char *argv[])
{
	return DytGimbal::main(argc, argv);
}
