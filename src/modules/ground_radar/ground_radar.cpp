#include "ground_radar.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

GroundRadar::GroundRadar() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

GroundRadar::~GroundRadar()
{
}

bool GroundRadar::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void GroundRadar::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_tracking_enabled) {
		return;
	}

	vehicle_local_position_s local_pos;
	if (_vehicle_local_position_sub.update(&local_pos)) {
		hrt_abstime now = hrt_absolute_time();
		
		matrix::Vector3f current_position(local_pos.x, local_pos.y, local_pos.z);
		
		if (_last_update_time > 0) {
			float dt = (now - _last_update_time) / 1e6f;
			if (dt > 0.001f && dt < 1.0f) {
				_current_velocity = (current_position - _last_position) / dt;
			}
		}
		
		_last_position = current_position;
		_last_update_time = now;

		radar_target_s radar_target{};
		radar_target.timestamp = now;
		radar_target.target_id = _target_id;
		radar_target.x = local_pos.x;
		radar_target.y = local_pos.y;
		radar_target.z = local_pos.z;
		radar_target.vx = _current_velocity(0);
		radar_target.vy = _current_velocity(1);
		radar_target.vz = _current_velocity(2);
		
		float range = sqrtf(local_pos.x * local_pos.x + local_pos.y * local_pos.y + local_pos.z * local_pos.z);
		radar_target.range = range;
		radar_target.azimuth = atan2f(local_pos.y, local_pos.x);
		radar_target.elevation = atan2f(-local_pos.z, sqrtf(local_pos.x * local_pos.x + local_pos.y * local_pos.y));
		radar_target.valid = true;

		_radar_target_pub.publish(radar_target);
	}
}

int GroundRadar::task_spawn(int argc, char *argv[])
{
	GroundRadar *instance = new GroundRadar();

	if (instance) {
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

int GroundRadar::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("not running");
		return 1;
	}

	if (!strcmp(argv[0], "track")) {
		if (argc >= 2) {
			get_instance()->_target_id = atoi(argv[1]);
			get_instance()->_tracking_enabled = true;
			PX4_INFO("Tracking target ID: %d", get_instance()->_target_id);
			return 0;
		}
	}

	if (!strcmp(argv[0], "stop")) {
		get_instance()->_tracking_enabled = false;
		PX4_INFO("Tracking stopped");
		return 0;
	}

	return print_usage("unknown command");
}

int GroundRadar::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Ground radar simulator");
	PRINT_MODULE_USAGE_NAME("ground_radar", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("track", "Start tracking target");
	PRINT_MODULE_USAGE_ARG("<target_id>", "Target ID", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop tracking");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ground_radar_main(int argc, char *argv[])
{
	return GroundRadar::main(argc, argv);
}
