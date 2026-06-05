#include "cooperative_rendezvous.hpp"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

CooperativeRendezvous::CooperativeRendezvous(const Options &options) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_options(options)
{
}

bool CooperativeRendezvous::init()
{
	ScheduleOnInterval(50_ms);
	return true;
}

void CooperativeRendezvous::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

void CooperativeRendezvous::update_vehicle_id()
{
	if (_vehicle_id_initialized) {
		return;
	}

	int32_t mav_sys_id = 0;
	param_t handle = param_find("MAV_SYS_ID");

	if (handle != PARAM_INVALID && param_get(handle, &mav_sys_id) == PX4_OK && mav_sys_id > 0) {
		_vehicle_id = static_cast<uint32_t>(mav_sys_id);
		_vehicle_id_initialized = true;
	}
}

CooperativeRendezvous::Role CooperativeRendezvous::active_role() const
{
	if (_options.role != Role::Auto) {
		return _options.role;
	}

	return _vehicle_id == 2 ? Role::Rendezvous : Role::Broadcast;
}

float CooperativeRendezvous::aux_value(int index) const
{
	switch (index) {
	case 1: return _manual_control.aux1;
	case 2: return _manual_control.aux2;
	case 3: return _manual_control.aux3;
	case 4: return _manual_control.aux4;
	case 5: return _manual_control.aux5;
	case 6: return _manual_control.aux6;
	default: return NAN;
	}
}

bool CooperativeRendezvous::aux_switch_active(int index) const
{
	const float value = aux_value(index);
	return PX4_ISFINITE(value) && value > 0.5f;
}

bool CooperativeRendezvous::button_active(int button) const
{
	if (button < 0 || button > 15) {
		return false;
	}

	return (_manual_control.buttons & (1u << button)) != 0;
}

bool CooperativeRendezvous::rendezvous_switch_enabled() const
{
	const int act_aux = _param_act_aux.get();
	const int act_btn = _param_act_btn.get();

	return (act_aux == 0 && act_btn < 0) || aux_switch_active(act_aux) || button_active(act_btn);
}

bool CooperativeRendezvous::dyt_guidance_active() const
{
	// Yield to the seeker only while it is actually commanding aircraft motion
	// (camera locked and tracking). While the seeker is just searching or has lost
	// the target it controls the gimbal only, so the position-sharing follower keeps
	// driving the aircraft. This is the single handoff boundary between the two
	// controllers so they never publish trajectory setpoints at the same time.
	return _dyt_guidance_status.controlling_vehicle && _dyt_guidance_status.timestamp != 0 &&
	       hrt_elapsed_time(&_dyt_guidance_status.timestamp) < 500_ms;
}

bool CooperativeRendezvous::local_position_valid(const vehicle_local_position_s &local_pos) const
{
	return local_pos.xy_valid && local_pos.z_valid &&
	       local_pos.xy_global && local_pos.z_global &&
	       PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && PX4_ISFINITE(local_pos.z) &&
	       PX4_ISFINITE(local_pos.ref_lat) && PX4_ISFINITE(local_pos.ref_lon) && PX4_ISFINITE(local_pos.ref_alt);
}

bool CooperativeRendezvous::update_map_projection(const vehicle_local_position_s &local_pos)
{
	if (!local_position_valid(local_pos)) {
		return false;
	}

	if (!_map_ref_initialized) {
		_map_ref.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);
		_map_ref_initialized = true;
		PX4_INFO("cooperative map ref: lat=%.7f lon=%.7f", local_pos.ref_lat, local_pos.ref_lon);
	}

	return true;
}

void CooperativeRendezvous::publish_own_position(const vehicle_local_position_s &local_pos)
{
	if (!_vehicle_id_initialized || !update_map_projection(local_pos)) {
		return;
	}

	double lat = static_cast<double>(NAN);
	double lon = static_cast<double>(NAN);
	_map_ref.reproject(local_pos.x, local_pos.y, lat, lon);

	cooperative_position_s position{};
	position.timestamp = hrt_absolute_time();
	position.mavid = _vehicle_id;
	position.lat = lat;
	position.lon = lon;
	position.alt = static_cast<double>(static_cast<float>(local_pos.ref_alt) - local_pos.z);
	position.vx = local_pos.vx;
	position.vy = local_pos.vy;
	position.vz = local_pos.vz;
	position.yaw = local_pos.heading_good_for_control ? local_pos.heading : static_cast<float>(NAN);
	position.yawspeed = local_pos.delta_heading;

	_cooperative_position_pub.publish(position);
}

bool CooperativeRendezvous::update_target_from_link()
{
	follower_info_s info{};
	bool updated = false;

	while (_follower_info_sub.update(&info)) {
		if (info.mavid == _options.target_id && info.mavid != _vehicle_id &&
		    PX4_ISFINITE(info.lat) && PX4_ISFINITE(info.lon) && PX4_ISFINITE(info.alt)) {
			_target_info = info;
			_last_target_time = hrt_absolute_time();
			updated = true;
		}
	}

	return updated;
}

bool CooperativeRendezvous::target_position_local(const vehicle_local_position_s &local_pos,
		matrix::Vector3f &target_position) const
{
	if (!_map_ref_initialized || _last_target_time == 0) {
		return false;
	}

	if ((hrt_absolute_time() - _last_target_time) > static_cast<hrt_abstime>(_options.target_timeout_s * 1_s)) {
		return false;
	}

	float x = NAN;
	float y = NAN;
	_map_ref.project(_target_info.lat, _target_info.lon, x, y);

	if (!PX4_ISFINITE(x) || !PX4_ISFINITE(y)) {
		return false;
	}

	float offset_x = _options.target_offset(0);
	float offset_y = _options.target_offset(1);
	const float target_distance = _param_dist.get();

	if (PX4_ISFINITE(target_distance) && target_distance >= 0.f) {
		const float offset_norm = sqrtf(offset_x * offset_x + offset_y * offset_y);

		if (offset_norm > 0.001f) {
			const float scale = target_distance / offset_norm;
			offset_x *= scale;
			offset_y *= scale;

		} else {
			offset_x = -target_distance;
			offset_y = 0.f;
		}
	}

	target_position(0) = x + offset_x;
	target_position(1) = y + offset_y;
	target_position(2) = (static_cast<float>(local_pos.ref_alt) - static_cast<float>(_target_info.alt)) +
			     _options.target_offset(2) - _param_alt_diff.get();

	return PX4_ISFINITE(target_position(2));
}

void CooperativeRendezvous::publish_offboard_heartbeat(bool position_control, bool velocity_control)
{
	offboard_control_mode_s mode{};
	mode.timestamp = hrt_absolute_time();
	mode.position = position_control;
	mode.velocity = velocity_control;
	mode.acceleration = false;
	mode.attitude = false;
	mode.body_rate = false;
	_offboard_control_mode_pub.publish(mode);
}

void CooperativeRendezvous::publish_trajectory_setpoint(const matrix::Vector3f &position, float yaw)
{
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		setpoint.position[i] = position(i);
		setpoint.velocity[i] = static_cast<float>(NAN);
		setpoint.acceleration[i] = static_cast<float>(NAN);
		setpoint.jerk[i] = static_cast<float>(NAN);
	}

	setpoint.yaw = yaw;
	setpoint.yawspeed = static_cast<float>(NAN);
	_trajectory_setpoint_pub.publish(setpoint);
}

void CooperativeRendezvous::request_offboard(const vehicle_status_s &status)
{
	if (!_options.auto_offboard || status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_mode_request < 1_s) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f;
	command.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_mode_request = now;
}

void CooperativeRendezvous::request_arm(const vehicle_status_s &status)
{
	if (!_options.auto_arm || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_arm_request < 1_s) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	command.param1 = 1.f;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_arm_request = now;
}

void CooperativeRendezvous::configure_relaxed_failsafes()
{
	if (!_options.relax_failsafes || _failsafes_configured) {
		return;
	}

	struct ParamSetInt {
		const char *name;
		int32_t value;
	};

	struct ParamSetFloat {
		const char *name;
		float value;
	};

	const ParamSetInt int_params[] = {
		{"NAV_DLL_ACT", 0},     // GCS datalink loss: disabled
		{"COM_DLL_EXCEPT", 7},  // ignore datalink loss in Mission/Hold/Offboard
		{"COM_RC_IN_MODE", 3},  // keep SITL RC/joystick input enabled instead of disabling sticks
		{"NAV_RCL_ACT", 1},     // RC loss fallback: Hold if it still triggers
		{"COM_RCL_EXCEPT", 7},  // ignore RC loss in Mission/Hold/Offboard
		{"COM_OBL_RC_ACT", 5},  // offboard loss fallback: Hold
	};

	const ParamSetFloat float_params[] = {
		{"COM_OF_LOSS_T", 5.f},
	};

	for (const ParamSetInt &item : int_params) {
		const param_t handle = param_find(item.name);

		if (handle != PARAM_INVALID) {
			param_set(handle, &item.value);
		}
	}

	for (const ParamSetFloat &item : float_params) {
		const param_t handle = param_find(item.name);

		if (handle != PARAM_INVALID) {
			param_set(handle, &item.value);
		}
	}

	_failsafes_configured = true;
	PX4_WARN("cooperative_rendezvous: relaxed simulation failsafes enabled");
}

void CooperativeRendezvous::hold_position(const vehicle_local_position_s &local_pos)
{
	keep_current_position_setpoint(local_pos);
}

void CooperativeRendezvous::keep_current_position_setpoint(const vehicle_local_position_s &local_pos)
{
	const matrix::Vector3f position(local_pos.x, local_pos.y, local_pos.z);

	publish_offboard_heartbeat(true, false);
	publish_trajectory_setpoint(position, local_pos.heading);
}

void CooperativeRendezvous::run_rendezvous(const vehicle_local_position_s &local_pos, const vehicle_status_s &status)
{
	matrix::Vector3f target_position{};

	if (!target_position_local(local_pos, target_position)) {
		const hrt_abstime now = hrt_absolute_time();

		if (now - _last_status_log > 2_s) {
			if (_last_target_time == 0) {
				PX4_WARN("cooperative rendezvous: waiting for target=%" PRIu32 " position", _options.target_id);

			} else {
				const double age_s = (double)(now - _last_target_time) * 1e-6;
				PX4_WARN("cooperative rendezvous: target=%" PRIu32 " position stale age=%.1fs",
					 _options.target_id, age_s);
			}

			_last_status_log = now;
		}

		hold_position(local_pos);
		return;
	}

	matrix::Vector3f current_position(local_pos.x, local_pos.y, local_pos.z);
	matrix::Vector3f to_target = target_position - current_position;
	const float distance = to_target.norm();

	const float yaw = PX4_ISFINITE(_target_info.yaw) ? static_cast<float>(_target_info.yaw) : local_pos.heading;

	publish_offboard_heartbeat(true, false);
	publish_trajectory_setpoint(target_position, yaw);

	request_arm(status);

	if (status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
	    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER &&
	    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL &&
	    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		request_offboard(status);
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_status_log > 2_s) {
		PX4_INFO("cooperative rendezvous: target=%" PRIu32 " distance=%.1fm setpoint=(%.1f %.1f %.1f)",
			 _options.target_id, (double)distance,
			 (double)target_position(0), (double)target_position(1), (double)target_position(2));
		_last_status_log = now;
	}
}

void CooperativeRendezvous::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	update_params_if_needed();
	update_vehicle_id();
	configure_relaxed_failsafes();

	vehicle_local_position_s local_pos{};
	vehicle_status_s status{};
	_vehicle_local_position_sub.copy(&local_pos);
	_vehicle_status_sub.copy(&status);
	_manual_control_sub.update(&_manual_control);
	_dyt_guidance_status_sub.update(&_dyt_guidance_status);

	if (local_position_valid(local_pos)) {
		publish_own_position(local_pos);
	}

	update_target_from_link();

	if (!local_position_valid(local_pos)) {
		return;
	}

	if (active_role() == Role::Rendezvous) {
		if (!rendezvous_switch_enabled() || dyt_guidance_active()) {
			return;
		}

		run_rendezvous(local_pos, status);

	} else if (active_role() == Role::Broadcast && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
		   rendezvous_switch_enabled() && !dyt_guidance_active()) {
		keep_current_position_setpoint(local_pos);
	}
}

int CooperativeRendezvous::print_status()
{
	const char *role = "auto";

	switch (active_role()) {
	case Role::Broadcast:
		role = "broadcast";
		break;

	case Role::Rendezvous:
		role = "rendezvous";
		break;

	case Role::Auto:
		break;
	}

	PX4_INFO("running: vehicle=%" PRIu32 " role=%s target=%" PRIu32 " offset=(%.1f %.1f %.1f) dist=%.1f alt_diff=%.1f",
		 _vehicle_id, role, _options.target_id,
		 (double)_options.target_offset(0),
		 (double)_options.target_offset(1),
		 (double)_options.target_offset(2),
		 (double)_param_dist.get(),
		 (double)_param_alt_diff.get());
	PX4_INFO("activation aux=%d enabled=%d dyt_active=%d",
		 static_cast<int>(_param_act_aux.get()), rendezvous_switch_enabled(), dyt_guidance_active());
	PX4_INFO("activation button=%d buttons=0x%04x",
		 static_cast<int>(_param_act_btn.get()), static_cast<unsigned>(_manual_control.buttons));
	return 0;
}

static bool parse_role(const char *arg, CooperativeRendezvous::Role &role)
{
	if (!strcmp(arg, "auto")) {
		role = CooperativeRendezvous::Role::Auto;
		return true;
	}

	if (!strcmp(arg, "broadcast")) {
		role = CooperativeRendezvous::Role::Broadcast;
		return true;
	}

	if (!strcmp(arg, "rendezvous")) {
		role = CooperativeRendezvous::Role::Rendezvous;
		return true;
	}

	return false;
}

int CooperativeRendezvous::task_spawn(int argc, char *argv[])
{
	Options options{};
	bool error_flag = false;
	bool offset_x_set = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "r:t:d:x:y:z:v:T:AFh", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r':
			if (!parse_role(myoptarg, options.role)) {
				error_flag = true;
			}
			break;

		case 't':
			options.target_id = strtoul(myoptarg, nullptr, 10);
			break;

		case 'd':
			if (!offset_x_set) {
				options.target_offset(0) = -fabsf(strtof(myoptarg, nullptr));
			}
			break;

		case 'x':
			options.target_offset(0) = strtof(myoptarg, nullptr);
			offset_x_set = true;
			break;

		case 'y':
			options.target_offset(1) = strtof(myoptarg, nullptr);
			break;

		case 'z':
			options.target_offset(2) = strtof(myoptarg, nullptr);
			break;

		case 'v':
			options.max_speed = math::constrain(strtof(myoptarg, nullptr), 0.5f, 20.f);
			break;

		case 'T':
			options.target_timeout_s = math::constrain(strtof(myoptarg, nullptr), 0.5f, 10.f);
			break;

		case 'A':
			options.auto_arm = true;
			break;

		case 'F':
			options.relax_failsafes = true;
			break;

		case 'h':
		case '?':
		default:
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	CooperativeRendezvous *instance = new CooperativeRendezvous(options);

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

int CooperativeRendezvous::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CooperativeRendezvous::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Two-aircraft cooperative rendezvous module.

MAV_SYS_ID=1 broadcasts its local position converted to WGS84.
MAV_SYS_ID=2 flies to a configurable offset near aircraft 1.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cooperative_rendezvous", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('r', "auto", "auto|broadcast|rendezvous", "Role selection", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 255, "Target MAV_SYS_ID for rendezvous", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 5.f, 0.f, 100.f, "Distance behind target when x offset is not set", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('x', -5.f, -100.f, 100.f, "Target NED x offset", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('y', 0.f, -100.f, 100.f, "Target NED y offset", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('z', 0.f, -50.f, 50.f, "Target NED z offset", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 3.f, 0.5f, 20.f, "Maximum approach speed", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('T', 2.f, 0.5f, 10.f, "Target timeout", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Auto arm rendezvous aircraft", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('F', "Relax link/manual/offboard failsafes for simulation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cooperative_rendezvous_main(int argc, char *argv[])
{
	return CooperativeRendezvous::main(argc, argv);
}
