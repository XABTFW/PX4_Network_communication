#pragma once
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

#include <cstdint>
#include <px4_platform_common/px4_config.h>

#include <sys/socket.h>
#include <net/if.h>
#include <netpacket/can.h>
#include <nuttx/can.h>

// Forward declarations for NuttX network functions
__BEGIN_DECLS
struct net_driver_s;
FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname);
int netdev_ifup(FAR struct net_driver_s *dev);
__END_DECLS

using namespace time_literals;

// DM-IMU CAN协议帧定义
#define CMD_READ  0
#define CMD_WRITE 1

// DM-IMU CAN协议定义
#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN (-235.2f)
#define GYRO_CAN_MAX  (34.88f)
#define GYRO_CAN_MIN  (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX  (180.0f)
#define ROLL_CAN_MIN  (-180.0f)
#define YAW_CAN_MAX   (180.0f)
#define YAW_CAN_MIN   (-180.0f)

// 通信端口类型
enum ImuComPort {
	COM_USB = 0,
	COM_RS485 = 1,
	COM_CAN = 2,
	COM_VOFA = 3
};

// CAN波特率
enum ImuBaudrate {
	CAN_BAUD_1M = 0,
	CAN_BAUD_500K = 1,
	CAN_BAUD_400K = 2,
	CAN_BAUD_250K = 3,
	CAN_BAUD_200K = 4,
	CAN_BAUD_100K = 5,
	CAN_BAUD_50K = 6,
	CAN_BAUD_25K = 7
};

// 寄存器ID定义（完整版）
enum RegId {
	REBOOT_IMU = 0,
	ACCEL_DATA = 1,
	GYRO_DATA = 2,
	EULER_DATA = 3,
	QUAT_DATA = 4,
	SET_ZERO = 5,
	ACCEL_CALI = 6,
	GYRO_CALI = 7,
	MAG_CALI = 8,
	CHANGE_COM = 9,
	SET_DELAY = 10,
	CHANGE_ACTIVE = 11,
	SET_BAUD = 12,
	SET_CAN_ID = 13,
	SET_MST_ID = 14,
	DATA_OUTPUT_SELECTION = 15,
	SAVE_PARAM = 254,
	RESTORE_SETTING = 255
};

// 设备状态（重构为明确的初始化状态机）
enum DeviceState {
	STATE_UNINITIALIZED = 0,  // 未初始化
	STATE_INITIALIZED = 1,    // start后，准备初始化
	STATE_WAIT_STABLE = 2,    // socket打开后等待总线/设备稳定
	STATE_ENTER_REQUEST = 3,  // 反复发切换到request模式命令
	STATE_WAIT_DATA = 4,      // 等待第一批有效数据（accel+gyro都要来）
	STATE_RUNNING = 5,        // 正常运行
	STATE_ERROR = 6           // 错误状态
};

class DmImuL1 : public px4::ScheduledWorkItem
{
public:
	DmImuL1(uint8_t can_id = 0x10, uint8_t mst_id = 0x00, const char *can_dev = "can0");
	~DmImuL1();

	int init();
	void start();
	void stop();

	// 配置和校准功能
	void calibrate_accel();
	void calibrate_gyro();
	void calibrate_mag();
	void set_zero();
	void reboot();

	// 参数配置
	void set_baudrate(ImuBaudrate baud);
	void set_can_id(uint8_t id);
	void set_mst_id(uint8_t id);
	void set_com_port(ImuComPort port);
	void set_active_delay(uint32_t delay_ms);
	void save_parameters();
	void restore_factory_settings();

	// 模式切换
	void change_to_active_mode();
	void change_to_request_mode();
	void send_request_mode_cmd(); // 只发命令，不改状态

	// 数据请求（被动模式）
	void request_accel();
	void request_gyro();
	void request_euler();
	void request_quat();

	// 状态查询
	DeviceState get_state() const { return _state; }
	bool is_running() const { return _state == STATE_RUNNING; }
	uint32_t get_error_count() const { return _error_count; }

	// 统计信息访问（公共）
	uint32_t get_rx_frame_count() const { return _rx_frame_count; }
	uint32_t get_tx_frame_count() const { return _tx_frame_count; }
	uint32_t get_request_sent_count() const { return _request_sent_count; }
	uint32_t get_response_count() const { return _response_count; }
	void reset_stats() {
		_rx_frame_count = 0;
		_tx_frame_count = 0;
		_request_sent_count = 0;
		_response_count = 0;
	}

	// 数据请求开关控制
	void enable_euler_request(bool enable) { _request_euler_enabled = enable; }
	void enable_quat_request(bool enable) { _request_quat_enabled = enable; }
	bool is_euler_request_enabled() const { return _request_euler_enabled; }
	bool is_quat_request_enabled() const { return _request_quat_enabled; }

	// 姿态数据访问
	void get_euler(float &pitch, float &yaw, float &roll) const {
		pitch = _euler[0];
		yaw = _euler[1];
		roll = _euler[2];
	}
	void get_quat(float &w, float &x, float &y, float &z) const {
		w = _quat[0];
		x = _quat[1];
		y = _quat[2];
		z = _quat[3];
	}
	uint32_t get_euler_count() const { return _euler_count; }
	uint32_t get_quat_count() const { return _quat_count; }

private:
	void Run() override;

	// CAN通信函数
	int open_can_socket();
	int can_send_cmd(uint8_t reg_id, uint8_t cmd, uint32_t data);
	int can_receive();

	// 数据解析函数
	void parse_accel_data(const uint8_t *data);
	void parse_gyro_data(const uint8_t *data);
	void parse_euler_data(const uint8_t *data);
	void parse_quat_data(const uint8_t *data);

	// 工具函数
	float uint_to_float(int x_int, float x_min, float x_max, int bits);

	// 错误处理
	void handle_timeout();

	// CAN设备
	int _can_fd{-1};
	const char *_can_dev;
	uint8_t _can_id;
	uint8_t _mst_id;

	// 设备状态
	DeviceState _state{STATE_UNINITIALIZED};
	uint32_t _error_count{0};
	uint64_t _last_data_time{0};
	static constexpr uint64_t TIMEOUT_US = 1000000; // 1秒超时

	// 初始化状态机相关
	uint64_t _socket_open_time{0};           // socket 打开时间
	bool _accel_seen_after_start{false};     // 启动后是否收到过 accel
	bool _gyro_seen_after_start{false};      // 启动后是否收到过 gyro
	uint64_t _last_request_mode_cmd_time{0}; // 上次发送请求模式命令的时间
	uint32_t _request_mode_cmd_count{0};     // 已发送请求模式命令的次数
	uint32_t _startup_retry_count{0};        // 启动重试次数
	uint64_t _error_enter_time{0};           // 进入错误状态的时间（问题C修复）

	// 初始化阶段的"请求-等待-确认"闭环控制
	uint64_t _init_accel_request_time{0};    // 初始化阶段发送 accel 请求的时间
	uint64_t _init_gyro_request_time{0};     // 初始化阶段发送 gyro 请求的时间
	uint64_t _init_round_start_time{0};      // 当前初始化轮次开始时间
	uint32_t _init_round_count{0};           // 初始化轮次计数

	// 请求发送时间控制
	uint64_t _last_accel_request_time{0};
	uint64_t _last_gyro_request_time{0};
	uint64_t _last_euler_request_time{0};
	uint64_t _last_quat_request_time{0};

	// 数据请求开关（默认关闭欧拉角和四元数）
	bool _request_euler_enabled{false};
	bool _request_quat_enabled{false};

	// 运行计数
	uint32_t _run_count{0};
	uint32_t _raw_rx_count{0}; // 原始接收帧计数

	// 统计信息
	volatile uint32_t _rx_frame_count{0};           // 接收到的总帧数
	volatile uint32_t _tx_frame_count{0};           // 发送的总帧数
	volatile uint32_t _request_sent_count{0};       // 发送的请求次数
	volatile uint32_t _response_count{0};           // 收到的响应次数

	// PX4 标准 IMU 发布器
	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	// 传感器数据
	float _accel[3]{};
	float _gyro[3]{};
	float _euler[3]{}; // pitch, yaw, roll
	float _quat[4]{};  // w, x, y, z

	uint32_t _accel_device_id{0};
	uint32_t _gyro_device_id{0};

	// 数据接收计数
	uint32_t _accel_count{0};
	uint32_t _gyro_count{0};
	uint32_t _euler_count{0};
	uint32_t _quat_count{0};

	// 必改3：分开统计 accel 和 gyro 更新时间
	uint64_t _last_accel_time{0};
	uint64_t _last_gyro_time{0};
};
