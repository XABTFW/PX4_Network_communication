/**
 * @file dm_imu_l1_main.cpp
 * DM-IMU L1 CAN driver for PX4
 *
 * 达妙IMU CAN驱动主程序
 */

#include "dm_imu_l1.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

static DmImuL1 *g_dev = nullptr;

extern "C" __EXPORT int dm_imu_l1_main(int argc, char *argv[]);

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
DM-IMU L1 CAN driver for Damiao IMU sensor.

This driver communicates with the Damiao IMU via CAN bus and publishes:
- sensor_accel
- sensor_gyro
- vehicle_attitude
- vehicle_angular_velocity
- sensor_combined

### Examples
Start the driver with default settings (CAN ID 0x10, device can0):
$ dm_imu_l1 start

Start with custom CAN ID and device:
$ dm_imu_l1 start -i 0x20 -d can1

Calibrate accelerometer:
$ dm_imu_l1 accel_cal

Calibrate gyroscope:
$ dm_imu_l1 gyro_cal

Switch to request mode:
$ dm_imu_l1 request_mode

Save parameters to flash:
$ dm_imu_l1 save

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dm_imu_l1", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0x10, 0, 0xFF, "CAN ID (hex)", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', 0x00, 0, 0xFF, "Master ID (hex)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "can0", "<device>", "CAN device name", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");

	PRINT_MODULE_USAGE_COMMAND_DESCR("accel_cal", "Calibrate accelerometer");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gyro_cal", "Calibrate gyroscope");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mag_cal", "Calibrate magnetometer");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_zero", "Set current position as zero");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reboot", "Reboot the IMU");

	PRINT_MODULE_USAGE_COMMAND_DESCR("active_mode", "Switch to active mode (continuous output)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("request_mode", "Switch to request mode (on-demand)");

	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save parameters to flash");
	PRINT_MODULE_USAGE_COMMAND_DESCR("restore", "Restore factory settings");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stats", "Display statistics and packet loss rate");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stats_reset", "Reset statistics counters");

	PRINT_MODULE_USAGE_COMMAND_DESCR("enable_euler", "Enable Euler angle data request (10Hz)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable_euler", "Disable Euler angle data request");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable_quat", "Enable quaternion data request (10Hz)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable_quat", "Disable quaternion data request");
	PRINT_MODULE_USAGE_COMMAND_DESCR("show_attitude", "Display current Euler angles and quaternion");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

static int start(int argc, char *argv[])
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	uint8_t can_id = 0x10;
	uint8_t mst_id = 0x00;
	const char *can_dev = "can0";

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:m:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			can_id = (uint8_t)strtol(myoptarg, nullptr, 16);
			break;

		case 'm':
			mst_id = (uint8_t)strtol(myoptarg, nullptr, 16);
			break;

		case 'd':
			can_dev = myoptarg;
			break;

		default:
			usage();
			return -1;
		}
	}

	g_dev = new DmImuL1(can_id, mst_id, can_dev);

	if (g_dev == nullptr) {
		PX4_ERR("alloc failed");
		return -1;
	}

	if (g_dev->init() != 0) {
		PX4_ERR("init failed");
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	g_dev->start();

	return 0;
}

static int stop()
{
	if (g_dev == nullptr) {
		PX4_WARN("not running");
		return -1;
	}

	g_dev->stop();
	delete g_dev;
	g_dev = nullptr;

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_INFO("not running");
		return -1;
	}

	PX4_INFO("State: %d", (int)g_dev->get_state());
	PX4_INFO("Error count: %lu", (unsigned long)g_dev->get_error_count());
	PX4_INFO("Running: %s", g_dev->is_running() ? "yes" : "no");
	PX4_INFO("Euler request: %s", g_dev->is_euler_request_enabled() ? "enabled" : "disabled");
	PX4_INFO("Quat request: %s", g_dev->is_quat_request_enabled() ? "enabled" : "disabled");

	return 0;
}

int dm_imu_l1_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -1;
	}

	const char *verb = argv[1];

	if (!strcmp(verb, "start")) {
		return start(argc - 1, argv + 1);
	}

	// 以下命令需要驱动已启动
	if (g_dev == nullptr) {
		PX4_ERR("not running, start first");
		return -1;
	}

	if (!strcmp(verb, "stop")) {
		return stop();
	}

	if (!strcmp(verb, "status")) {
		return status();
	}

	// 校准命令
	if (!strcmp(verb, "accel_cal")) {
		g_dev->calibrate_accel();
		return 0;
	}

	if (!strcmp(verb, "gyro_cal")) {
		g_dev->calibrate_gyro();
		return 0;
	}

	if (!strcmp(verb, "mag_cal")) {
		g_dev->calibrate_mag();
		return 0;
	}

	if (!strcmp(verb, "set_zero")) {
		g_dev->set_zero();
		return 0;
	}

	if (!strcmp(verb, "reboot")) {
		g_dev->reboot();
		return 0;
	}

	// 模式切换
	if (!strcmp(verb, "active_mode")) {
		g_dev->change_to_active_mode();
		return 0;
	}

	if (!strcmp(verb, "request_mode")) {
		g_dev->change_to_request_mode();
		return 0;
	}

	// 参数管理
	if (!strcmp(verb, "save")) {
		g_dev->save_parameters();
		return 0;
	}

	if (!strcmp(verb, "restore")) {
		g_dev->restore_factory_settings();
		return 0;
	}

	// 统计信息
	if (!strcmp(verb, "stats")) {
		PX4_INFO("=== DM_IMU_L1 Statistics ===");
		PX4_INFO("Total RX Frames:    %u", (unsigned)g_dev->get_rx_frame_count());
		PX4_INFO("Total TX Frames:    %u", (unsigned)g_dev->get_tx_frame_count());
		PX4_INFO("");
		PX4_INFO("Request Sent:       %u", (unsigned)g_dev->get_request_sent_count());
		PX4_INFO("Responses:          %u", (unsigned)g_dev->get_response_count());

		uint32_t sent = g_dev->get_request_sent_count();
		uint32_t resp = g_dev->get_response_count();

		if (sent > 0) {
			uint32_t lost = sent - resp;
			float loss_rate = (float)lost / (float)sent * 100.0f;
			float success_rate = (float)resp / (float)sent * 100.0f;

			PX4_INFO("Lost:               %u", (unsigned)lost);
			PX4_INFO("Loss Rate:          %.2f%%", (double)loss_rate);
			PX4_INFO("Success Rate:       %.2f%%", (double)success_rate);
		} else {
			PX4_INFO("Loss Rate:          N/A (no requests sent)");
		}
		return 0;
	}

	if (!strcmp(verb, "stats_reset")) {
		g_dev->reset_stats();
		PX4_INFO("Statistics reset");
		return 0;
	}

	// 数据请求开关控制
	if (!strcmp(verb, "enable_euler")) {
		g_dev->enable_euler_request(true);
		PX4_INFO("Euler angle data request enabled (10Hz)");
		PX4_INFO("Note: This will increase CAN bus load");
		return 0;
	}

	if (!strcmp(verb, "disable_euler")) {
		g_dev->enable_euler_request(false);
		PX4_INFO("Euler angle data request disabled");
		return 0;
	}

	if (!strcmp(verb, "enable_quat")) {
		g_dev->enable_quat_request(true);
		PX4_INFO("Quaternion data request enabled (10Hz)");
		PX4_INFO("Note: This will increase CAN bus load");
		return 0;
	}

	if (!strcmp(verb, "disable_quat")) {
		g_dev->enable_quat_request(false);
		PX4_INFO("Quaternion data request disabled");
		return 0;
	}

	// 显示姿态数据
	if (!strcmp(verb, "show_attitude")) {
		float pitch, yaw, roll;
		float w, x, y, z;

		g_dev->get_euler(pitch, yaw, roll);
		g_dev->get_quat(w, x, y, z);

		uint32_t euler_count = g_dev->get_euler_count();
		uint32_t quat_count = g_dev->get_quat_count();

		PX4_INFO("=== Attitude Data ===");
		PX4_INFO("Euler Angles (count: %u):", (unsigned)euler_count);
		if (euler_count > 0) {
			PX4_INFO("  Pitch: %.2f°", (double)pitch);
			PX4_INFO("  Yaw:   %.2f°", (double)yaw);
			PX4_INFO("  Roll:  %.2f°", (double)roll);
		} else {
			PX4_INFO("  No Euler data received yet");
			PX4_INFO("  Use 'dm_imu_l1 enable_euler' to start receiving");
		}

		PX4_INFO("");
		PX4_INFO("Quaternion (count: %u):", (unsigned)quat_count);
		if (quat_count > 0) {
			PX4_INFO("  w: %.4f", (double)w);
			PX4_INFO("  x: %.4f", (double)x);
			PX4_INFO("  y: %.4f", (double)y);
			PX4_INFO("  z: %.4f", (double)z);

			// 计算四元数模长（应该接近1.0）
			float norm = sqrtf(w*w + x*x + y*y + z*z);
			PX4_INFO("  norm: %.4f (should be ~1.0)", (double)norm);
		} else {
			PX4_INFO("  No quaternion data received yet");
			PX4_INFO("  Use 'dm_imu_l1 enable_quat' to start receiving");
		}

		return 0;
	}

	usage();
	return -1;
}
