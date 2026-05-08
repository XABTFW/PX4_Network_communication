/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

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
#include <uORB/topics/dyt_guidance_status.h>
#include <uORB/topics/dyt_target.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class DytGuidance : public ModuleBase<DytGuidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DytGuidance();
	~DytGuidance() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void show_status();

private:
	static constexpr int OBS_BUFFER_LEN{6};
	static constexpr float TARGET_MIN_BBOX_PX{4.f};
	static constexpr float TARGET_MAX_LOS_RAD{0.45f};
	static constexpr float TARGET_MAX_GIMBAL_YAW_RAD{1.92f};
	static constexpr int SEARCH_CENTER_PASSES{2};
	static constexpr hrt_abstime MANUAL_TAKEOVER_GRACE{500_ms};

	struct ScanArea {
		float yaw_min_deg{0.f};
		float yaw_max_deg{0.f};
		float pitch_top_deg{0.f};
		float pitch_bottom_deg{0.f};
	};

	struct LosObservation {
		hrt_abstime sample_time{0};
		Vector3f los_body{};
		float frame_dt_s{0.f};
	};

	struct TrackProfile {
		float nav_gain{0.f};
		float v_cmd{0.f};
		float k_a{0.f};
		float k_v{0.f};
		uint8_t submode{dyt_guidance_status_s::SUBMODE_FOLLOW};
	};

	enum class TaskState : uint8_t {
		Idle = dyt_guidance_status_s::STATE_IDLE,
		SearchWaitLock = dyt_guidance_status_s::STATE_SEARCH_WAIT_LOCK,
		TrackFollow = dyt_guidance_status_s::STATE_TRACK_FOLLOW,
		TrackIntercept = dyt_guidance_status_s::STATE_TRACK_INTERCEPT,
		LostHold = dyt_guidance_status_s::STATE_LOST_HOLD,
		Abort = dyt_guidance_status_s::STATE_ABORT
	};

	enum class ScanRegion : uint8_t {
		Center,
		Global
	};

	void Run() override;

	void update_subscriptions();
	void update_params_if_needed();

	float aux_value(int index) const;
	bool aux_switch_active(int index) const;
	bool payload_switch_active() const;
	bool activation_requested() const;
	bool preconditions_ok() const;
	bool manual_takeover_detected() const;

	void handle_new_target(const dyt_target_s &target);
	bool build_los_body(const dyt_target_s &target, Vector3f &los_body) const;
	void push_observation(const Vector3f &los_body, hrt_abstime sample_time, float frame_dt_s);
	void clear_observations();
	bool update_los_estimate(hrt_abstime now);

	void capture_hold_setpoint();
	void publish_hold_setpoint();
	void publish_track_setpoint(const TrackProfile &profile);
	void publish_offboard_mode(bool hold_mode);
	void request_offboard_mode();
	void publish_status();

	void enter_state(TaskState new_state, uint8_t lost_reason = dyt_guidance_status_s::LOST_REASON_NONE);
	void activate_guidance(hrt_abstime now);
	void deactivate_guidance(uint8_t lost_reason);
	void abort_guidance(uint8_t lost_reason);
	void enter_lost_hold(uint8_t lost_reason);
	bool update_lost_reacquire(hrt_abstime now, hrt_abstime lost_enter_time = 0);
	void update_payload_only_reacquire(hrt_abstime now);

	bool target_locked() const;
	bool target_geometry_valid(const dyt_target_s &target) const;
	bool target_geometry_valid() const;
	bool target_hint_detected() const;
	bool target_fresh() const;
	bool target_lock_candidate() const;
	bool target_usable() const;
	bool intercept_allowed() const;

	TrackProfile follow_profile() const;
	TrackProfile intercept_profile() const;

	void send_dyt_command(uint8_t command, int16_t param_x = 0, int16_t param_y = 0, uint8_t param3 = 0, int8_t zoom_rate = 0);
	void retry_autolock(hrt_abstime now);
	bool update_hint_autolock(hrt_abstime now);
	bool handle_lock_candidate_or_timeout(hrt_abstime now);
	void reset_search_scan(hrt_abstime now);
	void update_search_scan(hrt_abstime now);
	void advance_search_scan_area(float pitch_step_deg);
	void send_scan_angle(float yaw_deg, float pitch_deg);
	int scan_row_count(const ScanArea &area, float pitch_step_deg) const;
	ScanArea active_scan_area() const;

	TaskState _state{TaskState::Idle};
	uint8_t _lost_reason{dyt_guidance_status_s::LOST_REASON_NONE};
	uint8_t _requested_submode{dyt_guidance_status_s::SUBMODE_FOLLOW};
	uint8_t _last_command{dyt_command_s::CMD_NONE};

	bool _prev_activation_request{false};
	bool _payload_lock_seen{false};
	bool _payload_lost_hold{false};
	int _lock_streak{0};
	int _relock_streak{0};
	int _lost_streak{0};
	ScanRegion _scan_region{ScanRegion::Center};
	int _scan_center_passes{0};
	int _scan_row{0};
	float _scan_segment_target_deg{NAN};

	hrt_abstime _state_enter_time{0};
	hrt_abstime _payload_lost_enter_time{0};
	hrt_abstime _last_offboard_request{0};
	hrt_abstime _last_retrigger_time{0};
	hrt_abstime _last_hint_lock_time{0};
	hrt_abstime _search_pause_until{0};
	hrt_abstime _next_scan_time{0};
	hrt_abstime _last_command_time{0};
	hrt_abstime _candidate_lock_start_time{0};
	hrt_abstime _candidate_ignore_until{0};
	hrt_abstime _candidate_ignored_sample_time{0};
	bool _candidate_lock_active{false};
	uint32_t _command_pub_count{0};
	float _scan_yaw_deg{0.f};
	float _scan_pitch_deg{0.f};

	LosObservation _observations[OBS_BUFFER_LEN]{};
	int _observation_count{0};

	dyt_target_s _last_target{};
	bool _have_target{false};

	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_status_s _vehicle_status{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	manual_control_setpoint_s _manual_control{};
	manual_control_switches_s _manual_switches{};

	Vector3f _hold_position{};
	float _hold_yaw{0.f};

	Vector3f _los_ned{};
	Vector3f _los_body_latest{};
	Vector3f _los_filtered{};
	Vector3f _prev_los_filtered{};
	Vector3f _omega_los{};
	Vector3f _velocity_sp{};
	Vector3f _acceleration_sp{};
	float _yaw_sp{NAN};
	float _yaw_rate_sp{NAN};
	hrt_abstime _prev_los_update{0};
	bool _los_filter_initialized{false};

	uORB::Subscription _dyt_target_sub{ORB_ID(dyt_target)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _manual_switches_sub{ORB_ID(manual_control_switches)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<dyt_command_s> _dyt_command_pub{ORB_ID(dyt_command)};
	uORB::Publication<dyt_guidance_status_s> _dyt_guidance_status_pub{ORB_ID(dyt_guidance_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DYTG_ACT_AUX>) _param_act_aux,
		(ParamInt<px4::params::DYTG_INT_AUX>) _param_int_aux,
		(ParamFloat<px4::params::DYTG_STK_TK>) _param_stick_takeover,
		(ParamInt<px4::params::DYTG_LOCK_N>) _param_lock_frames,
		(ParamInt<px4::params::DYTG_RELOCKN>) _param_relock_frames,
		(ParamInt<px4::params::DYTG_WAITMS>) _param_wait_ms,
		(ParamInt<px4::params::DYTG_LOSTMS>) _param_lost_ms,
		(ParamFloat<px4::params::DYTG_SC_YSPD>) _param_scan_yaw_speed,
		(ParamFloat<px4::params::DYTG_SC_PSPD>) _param_scan_pitch_speed,
		(ParamFloat<px4::params::DYTG_SC_STEP>) _param_scan_pitch_step,
		(ParamFloat<px4::params::DYTG_SC_PAUSE>) _param_scan_edge_pause,
		(ParamFloat<px4::params::DYTG_SC_YSTP>) _param_scan_yaw_step,
		(ParamFloat<px4::params::DYTG_SC_DWEL>) _param_scan_dwell,
		(ParamInt<px4::params::DYTG_CTRMS>) _param_center_ms,
		(ParamInt<px4::params::DYTG_RTRYMS>) _param_retry_ms,
		(ParamFloat<px4::params::DYTG_DLY_MS>) _param_delay_ms,
		(ParamFloat<px4::params::DYTG_MAXAGE>) _param_max_age,
		(ParamFloat<px4::params::DYTG_MAXJIT>) _param_max_gap,
		(ParamFloat<px4::params::DYTG_INTDLY>) _param_intercept_delay,
		(ParamFloat<px4::params::DYTG_N_FOL>) _param_n_follow,
		(ParamFloat<px4::params::DYTG_V_FOL>) _param_v_follow,
		(ParamFloat<px4::params::DYTG_KA_FOL>) _param_ka_follow,
		(ParamFloat<px4::params::DYTG_KV_FOL>) _param_kv_follow,
		(ParamFloat<px4::params::DYTG_N_INT>) _param_n_intercept,
		(ParamFloat<px4::params::DYTG_V_INT>) _param_v_intercept,
		(ParamFloat<px4::params::DYTG_KA_INT>) _param_ka_intercept,
		(ParamFloat<px4::params::DYTG_KV_INT>) _param_kv_intercept,
		(ParamFloat<px4::params::DYTG_VMIN>) _param_vmin,
		(ParamFloat<px4::params::DYTG_MAXV>) _param_max_vel,
		(ParamFloat<px4::params::DYTG_MAXACC>) _param_max_acc,
		(ParamFloat<px4::params::DYTG_MAXYAWR>) _param_max_yaw_rate_deg,
		(ParamFloat<px4::params::DYTG_YAWLIM>) _param_yaw_limit_deg,
		(ParamFloat<px4::params::DYTG_MAXDZ>) _param_max_dz,
		(ParamFloat<px4::params::DYTG_ZSCALE>) _param_z_scale,
		(ParamFloat<px4::params::DYTG_FCONE>) _param_front_cone_deg,
		(ParamFloat<px4::params::DYTG_LPF_A>) _param_lpf_alpha,
		(ParamFloat<px4::params::DYTG_PREDMAX>) _param_pred_max,
		(ParamInt<px4::params::DYTG_LXSIGN>) _param_los_x_sign,
		(ParamInt<px4::params::DYTG_LYSIGN>) _param_los_y_sign,
		(ParamInt<px4::params::DYTG_RSIGN>) _param_roll_sign,
		(ParamInt<px4::params::DYTG_PSIGN>) _param_pitch_sign,
		(ParamInt<px4::params::DYTG_YSIGN>) _param_yaw_sign,
		(ParamFloat<px4::params::DYTG_ROFF>) _param_roll_off_deg,
		(ParamFloat<px4::params::DYTG_POFF>) _param_pitch_off_deg,
		(ParamFloat<px4::params::DYTG_YOFF>) _param_yaw_off_deg
	);
};

DytGuidance::DytGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	memset(&_last_target, 0, sizeof(_last_target));
}

bool DytGuidance::init()
{
	ScheduleOnInterval(20_ms);
	return true;
}

void DytGuidance::show_status()
{
	PX4_INFO("state: %u", static_cast<unsigned>(_state));
	PX4_INFO("target fresh: %d", target_fresh());
	PX4_INFO("target locked: %d", target_locked());
	PX4_INFO("target geometry: %d", target_geometry_valid());
	PX4_INFO("observations: %d", _observation_count);
	PX4_INFO("payload lock seen: %d", _payload_lock_seen);
	PX4_INFO("payload lost hold: %d", _payload_lost_hold);
	PX4_INFO("preconditions ok: %d", preconditions_ok());
	PX4_INFO("activation request: %d", activation_requested());
	PX4_INFO("activation aux value: %.2f", static_cast<double>(aux_value(_param_act_aux.get())));
	PX4_INFO("payload switch: %u", static_cast<unsigned>(_manual_switches.payload_power_switch));
	PX4_INFO("manual valid: %u roll=%.2f pitch=%.2f yaw=%.2f sticks=%u",
		 static_cast<unsigned>(_manual_control.valid),
		 static_cast<double>(_manual_control.roll),
		 static_cast<double>(_manual_control.pitch),
		 static_cast<double>(_manual_control.yaw),
		 static_cast<unsigned>(_manual_control.sticks_moving));
	PX4_INFO("activation aux: %ld", static_cast<long>(_param_act_aux.get()));
	PX4_INFO("intercept aux: %ld", static_cast<long>(_param_int_aux.get()));
	PX4_INFO("command pubs: %lu", static_cast<unsigned long>(_command_pub_count));
	PX4_INFO("last command: %u", static_cast<unsigned>(_last_command));
	PX4_INFO("last command age: %.3f s", static_cast<double>(_last_command_time > 0 ?
			(hrt_absolute_time() - _last_command_time) * 1e-6 : -1.0));
}

void DytGuidance::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

void DytGuidance::update_subscriptions()
{
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_manual_control_sub.update(&_manual_control);
	_manual_switches_sub.update(&_manual_switches);

	dyt_target_s target{};

	while (_dyt_target_sub.update(&target)) {
		_last_target = target;
		_have_target = true;
		handle_new_target(target);
	}
}

float DytGuidance::aux_value(int index) const
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

bool DytGuidance::aux_switch_active(int index) const
{
	const float value = aux_value(index);
	return PX4_ISFINITE(value) && value > 0.5f;
}

bool DytGuidance::payload_switch_active() const
{
	return _manual_switches.payload_power_switch == manual_control_switches_s::SWITCH_POS_ON;
}

bool DytGuidance::activation_requested() const
{
	return aux_switch_active(_param_act_aux.get()) || payload_switch_active();
}

bool DytGuidance::preconditions_ok() const
{
	return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && !_vehicle_status.failsafe
	       && _vehicle_local_position.xy_valid
	       && _vehicle_local_position.z_valid;
}

bool DytGuidance::manual_takeover_detected() const
{
	if (!_manual_control.valid) {
		return false;
	}

	const float threshold = _param_stick_takeover.get();
	return fabsf(_manual_control.roll) > threshold
	       || fabsf(_manual_control.pitch) > threshold
	       || fabsf(_manual_control.yaw) > threshold;
}

void DytGuidance::handle_new_target(const dyt_target_s &target)
{
	if (!target.target_valid || !target_geometry_valid(target)) {
		return;
	}

	Vector3f los_body;

	if (!build_los_body(target, los_body)) {
		return;
	}

	push_observation(los_body, target.timestamp_sample, target.frame_dt_s);
}

bool DytGuidance::build_los_body(const dyt_target_s &target, Vector3f &los_body) const
{
	const float los_x = target.los_x_rad * static_cast<float>(_param_los_x_sign.get());
	const float los_y = target.los_y_rad * static_cast<float>(_param_los_y_sign.get());

	Vector3f los_gimbal(1.f, tanf(los_x), tanf(los_y));

	if (los_gimbal.norm_squared() < 1e-6f) {
		return false;
	}

	los_gimbal.normalize();

	const float roll = target.gimbal_roll_rad * static_cast<float>(_param_roll_sign.get())
			   + math::radians(_param_roll_off_deg.get());
	const float pitch = target.gimbal_pitch_rad * static_cast<float>(_param_pitch_sign.get())
			    + math::radians(_param_pitch_off_deg.get());
	const float yaw = target.gimbal_yaw_rad * static_cast<float>(_param_yaw_sign.get())
			  + math::radians(_param_yaw_off_deg.get());

	const Dcmf gimbal_to_body(Eulerf(roll, pitch, yaw));
	los_body = gimbal_to_body * los_gimbal;

	if (!PX4_ISFINITE(los_body(0)) || !PX4_ISFINITE(los_body(1)) || !PX4_ISFINITE(los_body(2))
	    || los_body.norm_squared() < 1e-6f) {
		return false;
	}

	los_body.normalize();
	return true;
}

void DytGuidance::push_observation(const Vector3f &los_body, hrt_abstime sample_time, float frame_dt_s)
{
	if (_observation_count < OBS_BUFFER_LEN) {
		_observations[_observation_count++] = {sample_time, los_body, frame_dt_s};

	} else {
		for (int i = 1; i < OBS_BUFFER_LEN; ++i) {
			_observations[i - 1] = _observations[i];
		}

		_observations[OBS_BUFFER_LEN - 1] = {sample_time, los_body, frame_dt_s};
	}

	_los_body_latest = los_body;
}

void DytGuidance::clear_observations()
{
	_observation_count = 0;

	for (auto &observation : _observations) {
		observation = LosObservation{};
	}

	_los_filtered.zero();
	_prev_los_filtered.zero();
	_omega_los.zero();
	_los_ned.zero();
	_los_filter_initialized = false;
	_prev_los_update = 0;
}

bool DytGuidance::update_los_estimate(hrt_abstime now)
{
	if (_observation_count <= 0) {
		return false;
	}

	Vector3f los_body = _observations[_observation_count - 1].los_body;

	if (_observation_count >= 2) {
		const LosObservation &latest = _observations[_observation_count - 1];
		const LosObservation &previous = _observations[_observation_count - 2];
		const float dt_obs = (latest.sample_time - previous.sample_time) * 1e-6f;

		if (dt_obs > 0.002f) {
			const Vector3f du_body = (latest.los_body - previous.los_body) / dt_obs;
			const float dt_pred = math::constrain((now - latest.sample_time) * 1e-6f, 0.f, _param_pred_max.get());
			los_body = latest.los_body + du_body * dt_pred;
		}
	}

	if (!PX4_ISFINITE(los_body(0)) || !PX4_ISFINITE(los_body(1)) || !PX4_ISFINITE(los_body(2))
	    || los_body.norm_squared() < 1e-6f) {
		return false;
	}

	los_body.normalize();

	const Dcmf body_to_ned(Quatf(_vehicle_attitude.q));
	const Vector3f los_raw = body_to_ned * los_body;

	if (!_los_filter_initialized) {
		_los_filtered = los_raw;
		_prev_los_filtered = los_raw;
		_prev_los_update = now;
		_omega_los.zero();
		_los_filter_initialized = true;

	} else {
		const float alpha = math::constrain(_param_lpf_alpha.get(), 0.f, 0.99f);
		_los_filtered = _los_filtered * alpha + los_raw * (1.f - alpha);

		if (_los_filtered.norm_squared() > 1e-6f) {
			_los_filtered.normalize();
		}

		const float dt = (now - _prev_los_update) * 1e-6f;

		if (dt > 0.005f) {
			const Vector3f du_dt = (_los_filtered - _prev_los_filtered) / dt;
			_omega_los = _los_filtered.cross(du_dt);
			_prev_los_filtered = _los_filtered;
			_prev_los_update = now;
		}
	}

	_los_ned = _los_filtered;
	return true;
}

void DytGuidance::capture_hold_setpoint()
{
	_hold_position(0) = _vehicle_local_position.x;
	_hold_position(1) = _vehicle_local_position.y;
	_hold_position(2) = _vehicle_local_position.z;
	_hold_yaw = Eulerf(Quatf(_vehicle_attitude.q)).psi();
}

bool DytGuidance::target_locked() const
{
	return _have_target && _last_target.tracking_state == dyt_target_s::TRACKING_STATE_LOCKED
	       && _last_target.target_valid;
}

bool DytGuidance::target_geometry_valid(const dyt_target_s &target) const
{
	const bool bbox_valid = PX4_ISFINITE(target.bbox_width_px) && PX4_ISFINITE(target.bbox_height_px)
				&& target.bbox_width_px >= TARGET_MIN_BBOX_PX
				&& target.bbox_height_px >= TARGET_MIN_BBOX_PX;
	const bool los_valid = PX4_ISFINITE(target.los_x_rad) && PX4_ISFINITE(target.los_y_rad)
			       && fabsf(target.los_x_rad) <= TARGET_MAX_LOS_RAD
			       && fabsf(target.los_y_rad) <= TARGET_MAX_LOS_RAD;
	const bool yaw_valid = PX4_ISFINITE(target.gimbal_yaw_rad)
			       && fabsf(target.gimbal_yaw_rad) <= TARGET_MAX_GIMBAL_YAW_RAD;

	return bbox_valid && los_valid && yaw_valid;
}

bool DytGuidance::target_geometry_valid() const
{
	return _have_target && target_geometry_valid(_last_target);
}

bool DytGuidance::target_hint_detected() const
{
	const bool bbox_valid = PX4_ISFINITE(_last_target.bbox_width_px) && PX4_ISFINITE(_last_target.bbox_height_px)
				&& _last_target.bbox_width_px >= TARGET_MIN_BBOX_PX
				&& _last_target.bbox_height_px >= TARGET_MIN_BBOX_PX;

	return _have_target && _last_target.auto_hint && target_fresh() && bbox_valid;
}

bool DytGuidance::target_fresh() const
{
	if (!_have_target || _last_target.timestamp_sample == 0) {
		return false;
	}

	const float age_s = (hrt_absolute_time() - _last_target.timestamp_sample) * 1e-6f;
	const bool age_ok = age_s <= _param_max_age.get();
	const bool gap_ok = !_have_target || (_last_target.frame_dt_s <= 0.f) || (_last_target.frame_dt_s <= _param_max_gap.get());

	return age_ok && gap_ok && _last_target.tracking_state != dyt_target_s::TRACKING_STATE_TIMEOUT
	       && _last_target.tracking_state != dyt_target_s::TRACKING_STATE_ERROR;
}

bool DytGuidance::target_lock_candidate() const
{
	if (!_have_target || _last_target.timestamp_sample == 0) {
		return false;
	}

	const float age_s = (hrt_absolute_time() - _last_target.timestamp_sample) * 1e-6f;

	if (age_s > 0.3f) {
		return false;
	}

	if (_last_target.tracking_state == dyt_target_s::TRACKING_STATE_TIMEOUT ||
	    _last_target.tracking_state == dyt_target_s::TRACKING_STATE_ERROR) {
		return false;
	}

	const bool bbox_valid = PX4_ISFINITE(_last_target.bbox_width_px)
				&& PX4_ISFINITE(_last_target.bbox_height_px)
				&& _last_target.bbox_width_px >= TARGET_MIN_BBOX_PX
				&& _last_target.bbox_height_px >= TARGET_MIN_BBOX_PX;

	if (target_locked()) {
		return true;
	}

	if (_last_target.auto_hint && bbox_valid) {
		return true;
	}

	return false;
}

bool DytGuidance::target_usable() const
{
	return target_locked() && target_fresh() && target_geometry_valid() && _observation_count > 0;
}

bool DytGuidance::intercept_allowed() const
{
	if (!target_usable()) {
		return false;
	}

	if ((_param_delay_ms.get() * 1e-3f) > _param_intercept_delay.get()) {
		return false;
	}

	if (_last_target.frame_dt_s > _param_max_gap.get()) {
		return false;
	}

	const float cone_cos = cosf(math::radians(_param_front_cone_deg.get()));
	return _los_body_latest(0) > cone_cos;
}

DytGuidance::TrackProfile DytGuidance::follow_profile() const
{
	return {_param_n_follow.get(), _param_v_follow.get(), _param_ka_follow.get(), _param_kv_follow.get(),
		dyt_guidance_status_s::SUBMODE_FOLLOW};
}

DytGuidance::TrackProfile DytGuidance::intercept_profile() const
{
	return {_param_n_intercept.get(), _param_v_intercept.get(), _param_ka_intercept.get(), _param_kv_intercept.get(),
		dyt_guidance_status_s::SUBMODE_INTERCEPT};
}

void DytGuidance::publish_hold_setpoint()
{
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();
	_hold_position.copyTo(setpoint.position);
	setpoint.velocity[0] = 0.f;
	setpoint.velocity[1] = 0.f;
	setpoint.velocity[2] = 0.f;
	setpoint.acceleration[0] = NAN;
	setpoint.acceleration[1] = NAN;
	setpoint.acceleration[2] = NAN;
	setpoint.yaw = _hold_yaw;
	setpoint.yawspeed = NAN;

	_velocity_sp.zero();
	_acceleration_sp.zero();
	_yaw_sp = _hold_yaw;
	_yaw_rate_sp = NAN;

	_trajectory_setpoint_pub.publish(setpoint);
}

void DytGuidance::publish_track_setpoint(const TrackProfile &profile)
{
	if (!update_los_estimate(hrt_absolute_time())) {
		publish_hold_setpoint();
		return;
	}

	Vector3f vehicle_velocity(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

	if (!PX4_ISFINITE(vehicle_velocity(0)) || !PX4_ISFINITE(vehicle_velocity(1)) || !PX4_ISFINITE(vehicle_velocity(2))) {
		vehicle_velocity.zero();
	}

	Vector2f horizontal_los(_los_ned(0), _los_ned(1));
	Vector2f horizontal_dir(1.f, 0.f);

	if (horizontal_los.norm() > 1e-3f) {
		horizontal_dir = horizontal_los.normalized();
	}

	_velocity_sp(0) = horizontal_dir(0) * profile.v_cmd;
	_velocity_sp(1) = horizontal_dir(1) * profile.v_cmd;
	_velocity_sp(2) = math::constrain(_los_ned(2) * profile.v_cmd * _param_z_scale.get(),
					  -_param_max_dz.get(), _param_max_dz.get());

	Vector2f vel_xy(_velocity_sp(0), _velocity_sp(1));

	if (vel_xy.norm() > _param_max_vel.get()) {
		vel_xy = vel_xy.normalized() * _param_max_vel.get();
		_velocity_sp(0) = vel_xy(0);
		_velocity_sp(1) = vel_xy(1);
	}

	const float closing_proxy = math::max(_param_vmin.get(), vehicle_velocity.dot(_los_ned));
	const Vector3f velocity_error = _velocity_sp - vehicle_velocity;

	_acceleration_sp = profile.nav_gain * closing_proxy * (_omega_los.cross(_los_ned))
			   + profile.k_a * _los_ned
			   + profile.k_v * velocity_error;

	Vector2f acc_xy(_acceleration_sp(0), _acceleration_sp(1));

	if (acc_xy.norm() > _param_max_acc.get()) {
		acc_xy = acc_xy.normalized() * _param_max_acc.get();
		_acceleration_sp(0) = acc_xy(0);
		_acceleration_sp(1) = acc_xy(1);
	}

	_acceleration_sp(2) = math::constrain(_acceleration_sp(2), -_param_max_acc.get(), _param_max_acc.get());

	const float current_yaw = Eulerf(Quatf(_vehicle_attitude.q)).psi();
	float desired_yaw = _hold_yaw;

	if (horizontal_los.norm() > 1e-3f) {
		desired_yaw = atan2f(_los_ned(1), _los_ned(0));
	}

	const float yaw_error = matrix::wrap_pi(desired_yaw - current_yaw);
	const float yaw_limit = math::radians(_param_yaw_limit_deg.get());
	_yaw_sp = matrix::wrap_pi(current_yaw + math::constrain(yaw_error, -yaw_limit, yaw_limit));
	_yaw_rate_sp = math::constrain(yaw_error * 2.f,
				       -math::radians(_param_max_yaw_rate_deg.get()),
				       math::radians(_param_max_yaw_rate_deg.get()));

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();
	setpoint.position[0] = NAN;
	setpoint.position[1] = NAN;
	setpoint.position[2] = NAN;
	_velocity_sp.copyTo(setpoint.velocity);
	_acceleration_sp.copyTo(setpoint.acceleration);
	setpoint.yaw = _yaw_sp;
	setpoint.yawspeed = _yaw_rate_sp;

	_trajectory_setpoint_pub.publish(setpoint);
}

void DytGuidance::publish_offboard_mode(bool hold_mode)
{
	offboard_control_mode_s mode{};
	mode.timestamp = hrt_absolute_time();
	mode.position = hold_mode;
	mode.velocity = !hold_mode;
	mode.acceleration = false;
	mode.attitude = false;
	mode.body_rate = false;
	mode.thrust_and_torque = false;
	mode.direct_actuator = false;
	_offboard_control_mode_pub.publish(mode);
}

void DytGuidance::request_offboard_mode()
{
	const hrt_abstime now = hrt_absolute_time();

	if ((now - _last_offboard_request) < 1_s) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.0f;
	cmd.param2 = 6.0f;
	cmd.target_system = _vehicle_status.system_id;
	cmd.target_component = _vehicle_status.component_id;
	cmd.source_system = _vehicle_status.system_id;
	cmd.source_component = _vehicle_status.component_id;
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);

	_last_offboard_request = now;
}

void DytGuidance::publish_status()
{
	dyt_guidance_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = static_cast<uint8_t>(_state);
	status.requested_submode = _requested_submode;

	if (_state == TaskState::TrackIntercept) {
		status.active_submode = dyt_guidance_status_s::SUBMODE_INTERCEPT;
	} else {
		status.active_submode = dyt_guidance_status_s::SUBMODE_FOLLOW;
	}

	status.lost_reason = _lost_reason;
	status.active = _state != TaskState::Idle && _state != TaskState::Abort;
	status.target_locked = target_locked();
	status.target_fresh = target_fresh();
	status.intercept_allowed = intercept_allowed();
	status.los_age_s = _have_target && _last_target.timestamp_sample > 0
			   ? (hrt_absolute_time() - _last_target.timestamp_sample) * 1e-6f : NAN;
	status.frame_dt_s = _have_target ? _last_target.frame_dt_s : NAN;
	status.delay_s = _param_delay_ms.get() * 1e-3f;
	_los_ned.copyTo(status.los_ned);
	_omega_los.copyTo(status.omega_los_ned);
	_velocity_sp.copyTo(status.velocity_sp);
	_acceleration_sp.copyTo(status.acceleration_sp);
	status.yaw_sp = _yaw_sp;
	status.yaw_rate_sp = _yaw_rate_sp;

	_dyt_guidance_status_pub.publish(status);
}

void DytGuidance::enter_state(TaskState new_state, uint8_t lost_reason)
{
	_state = new_state;
	_lost_reason = lost_reason;
	_state_enter_time = hrt_absolute_time();

	if (new_state == TaskState::SearchWaitLock) {
		_lock_streak = 0;
		_relock_streak = 0;
		_last_hint_lock_time = 0;
	} else if (new_state == TaskState::TrackFollow || new_state == TaskState::TrackIntercept) {
		_next_scan_time = 0;
		_search_pause_until = 0;
		_lost_streak = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
	} else if (new_state == TaskState::LostHold) {
		_relock_streak = 0;
		_last_retrigger_time = 0;
		_last_hint_lock_time = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
		clear_observations();
		capture_hold_setpoint();
		send_dyt_command(dyt_command_s::CMD_CENTER_GIMBAL);
		reset_search_scan(_state_enter_time);
	} else if (new_state == TaskState::Idle) {
		clear_observations();
		_velocity_sp.zero();
		_acceleration_sp.zero();
		_yaw_sp = NAN;
		_yaw_rate_sp = NAN;
		_next_scan_time = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
	} else if (new_state == TaskState::Abort) {
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
	}
}

void DytGuidance::activate_guidance(hrt_abstime now)
{
	_payload_lock_seen = false;
	_payload_lost_hold = false;
	_payload_lost_enter_time = 0;
	_last_retrigger_time = now;
	_last_hint_lock_time = 0;
	_search_pause_until = 0;
	_next_scan_time = 0;
	send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);

	if (!preconditions_ok()) {
		return;
	}

	clear_observations();
	capture_hold_setpoint();
	_requested_submode = dyt_guidance_status_s::SUBMODE_FOLLOW;
	enter_state(TaskState::SearchWaitLock);
}

void DytGuidance::deactivate_guidance(uint8_t lost_reason)
{
	_payload_lock_seen = false;
	_payload_lost_hold = false;
	_payload_lost_enter_time = 0;
	_last_retrigger_time = 0;
	_last_hint_lock_time = 0;
	_search_pause_until = 0;
	_next_scan_time = 0;
	send_dyt_command(dyt_command_s::CMD_STOP_TRACK);

	if (_state != TaskState::Idle) {
		enter_state(TaskState::Abort, lost_reason);
	}
}

void DytGuidance::abort_guidance(uint8_t lost_reason)
{
	send_dyt_command(dyt_command_s::CMD_STOP_TRACK);
	enter_state(TaskState::Abort, lost_reason);
}

void DytGuidance::enter_lost_hold(uint8_t lost_reason)
{
	enter_state(TaskState::LostHold, lost_reason);
}

bool DytGuidance::update_lost_reacquire(hrt_abstime now, hrt_abstime lost_enter_time)
{
	const hrt_abstime enter_time = lost_enter_time > 0 ? lost_enter_time : _state_enter_time;
	const int32_t center_ms = math::max(_param_center_ms.get(), int32_t{0});
	const hrt_abstime center_delay = static_cast<hrt_abstime>(center_ms) * 1000ULL;

	if ((now - enter_time) < center_delay) {
		return true;
	}

	return false;
}

void DytGuidance::update_payload_only_reacquire(hrt_abstime now)
{
	if (target_usable()) {
		_payload_lock_seen = true;
		_payload_lost_hold = false;
		_payload_lost_enter_time = 0;
		_last_retrigger_time = 0;
		_next_scan_time = 0;
		_search_pause_until = 0;
		return;
	}

	if (!_payload_lock_seen) {
		return;
	}

	if (!_payload_lost_hold) {
		_payload_lost_hold = true;
		_payload_lost_enter_time = now;
		_last_retrigger_time = 0;
		_last_hint_lock_time = 0;
		_search_pause_until = 0;
		clear_observations();
		send_dyt_command(dyt_command_s::CMD_CENTER_GIMBAL);
		reset_search_scan(now);
	}

	if (!update_lost_reacquire(now, _payload_lost_enter_time)) {
		update_search_scan(now);
	}
}

void DytGuidance::send_dyt_command(uint8_t command, int16_t param_x, int16_t param_y, uint8_t param3, int8_t zoom_rate)
{
	dyt_command_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.command = command;
	msg.param_x = param_x;
	msg.param_y = param_y;
	msg.param3 = param3;
	msg.zoom_rate = zoom_rate;
	_dyt_command_pub.publish(msg);
	_last_command = command;
	_last_command_time = msg.timestamp;
	++_command_pub_count;
}

void DytGuidance::retry_autolock(hrt_abstime now)
{
	const int32_t retry_ms = math::max(_param_retry_ms.get(), int32_t{100});
	const hrt_abstime retry_interval = static_cast<hrt_abstime>(retry_ms) * 1000ULL;

	if (_last_retrigger_time == 0 || (now - _last_retrigger_time) >= retry_interval) {
		send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
		_last_retrigger_time = now;
	}
}

bool DytGuidance::update_hint_autolock(hrt_abstime now)
{
	if (!target_lock_candidate()) {
		return false;
	}

	// 已经 LOCKED 时，不重复发锁定命令，但告诉外层：目标存在，应该停止搜索
	if (target_locked()) {
		return true;
	}

	const int32_t retry_ms = math::max(_param_retry_ms.get(), int32_t{100});
	const hrt_abstime retry_interval = static_cast<hrt_abstime>(retry_ms) * 1000ULL;

	if (_last_hint_lock_time == 0 || (now - _last_hint_lock_time) >= retry_interval) {
		send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
		_last_hint_lock_time = now;
		_last_retrigger_time = now;
		return true;
	}

	// 虽然没到重发时间，但目标候选仍然存在，所以外层必须停止搜索
	return true;
}

bool DytGuidance::handle_lock_candidate_or_timeout(hrt_abstime now)
{
	constexpr hrt_abstime LOCK_CANDIDATE_MAX_HOLD = 1200_ms;
	constexpr hrt_abstime SCAN_UPDATE_DELAY = 100_ms;
	constexpr hrt_abstime CANDIDATE_COOLDOWN = 800_ms;

	if (!target_lock_candidate()) {
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		return false;
	}

	// 候选刚超时过，在冷却期内不要再拦截搜索
	// 除非来了新的 target sample
	if (_candidate_ignore_until > now &&
	    _last_target.timestamp_sample == _candidate_ignored_sample_time) {
		return false;
	}

	if (!_candidate_lock_active) {
		_candidate_lock_active = true;
		_candidate_lock_start_time = now;
	}

	const hrt_abstime held_time = now - _candidate_lock_start_time;

	if (held_time < LOCK_CANDIDATE_MAX_HOLD) {
		update_hint_autolock(now);
		_search_pause_until = now + LOCK_CANDIDATE_MAX_HOLD;
		_next_scan_time = now + SCAN_UPDATE_DELAY;
		_scan_segment_target_deg = NAN;
		return true;
	}

	// 候选等待超时：允许搜索继续，不要立刻重新拦截
	_candidate_lock_active = false;
	_candidate_lock_start_time = 0;
	_candidate_ignore_until = now + CANDIDATE_COOLDOWN;
	_candidate_ignored_sample_time = _last_target.timestamp_sample;
	_last_hint_lock_time = 0;
	_search_pause_until = 0;

	return false;
}

void DytGuidance::reset_search_scan(hrt_abstime now)
{
	_scan_region = ScanRegion::Center;
	_scan_center_passes = 0;
	_scan_row = 0;
	_scan_yaw_deg = 0.f;
	_scan_pitch_deg = 0.f;
	_scan_segment_target_deg = NAN;
	_search_pause_until = 0;

	const int32_t center_ms = math::max(_param_center_ms.get(), int32_t{0});
	_next_scan_time = now + static_cast<hrt_abstime>(center_ms) * 1000ULL;
}

int DytGuidance::scan_row_count(const ScanArea &area, float pitch_step_deg) const
{
	const float step_deg = (PX4_ISFINITE(pitch_step_deg) && pitch_step_deg > 0.f) ? pitch_step_deg : 10.f;
	const float pitch_span = math::max(area.pitch_top_deg - area.pitch_bottom_deg, 0.f);
	return static_cast<int>(floorf(pitch_span / step_deg)) + 1;
}

DytGuidance::ScanArea DytGuidance::active_scan_area() const
{
	if (_scan_region == ScanRegion::Center) {
		return {-60.f, 60.f, 30.f, -20.f};
	}

	return {-110.f, 110.f, 40.f, -100.f};
}

void DytGuidance::advance_search_scan_area(float pitch_step_deg)
{
	if (_scan_row < scan_row_count(active_scan_area(), pitch_step_deg)) {
		return;
	}

	_scan_row = 0;

	if (_scan_region == ScanRegion::Center) {
		++_scan_center_passes;

		if (_scan_center_passes >= SEARCH_CENTER_PASSES) {
			_scan_center_passes = 0;
			_scan_region = ScanRegion::Global;
		}

	} else {
		_scan_region = ScanRegion::Center;
		_scan_center_passes = 0;
	}
}

void DytGuidance::send_scan_angle(float yaw_deg, float pitch_deg)
{
	const float yaw_limited = math::constrain(yaw_deg, -110.f, 110.f);
	const float pitch_limited = math::constrain(pitch_deg, -100.f, 40.f);
	const int16_t yaw_cmd = static_cast<int16_t>(roundf(yaw_limited * 100.f));
	const int16_t pitch_cmd = static_cast<int16_t>(roundf(pitch_limited * 100.f));

	send_dyt_command(dyt_command_s::CMD_SET_FRAME_ANGLE, yaw_cmd, pitch_cmd);
}

void DytGuidance::update_search_scan(hrt_abstime now)
{
	constexpr float SCAN_UPDATE_INTERVAL_S = 0.10f;

	// 已经 LOCKED 且新鲜，才真正禁止搜索
	if (target_locked() && target_fresh()) {
		_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
		_search_pause_until = now + 300_ms;
		_scan_segment_target_deg = NAN;
		return;
	}

	// 只是候选目标：触发锁定，但不要永久挡住搜索
	if (target_lock_candidate()) {
		update_hint_autolock(now);

		if (_last_hint_lock_time != 0 && (now - _last_hint_lock_time) < 300_ms) {
			_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
			return;
		}

		// 注意：这里不要 return，让搜索逻辑继续执行
	}

	if (_next_scan_time == 0) {
		reset_search_scan(now);
	}

	if (now < _next_scan_time) {
		return;
	}

	if (_search_pause_until > now) {
		_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
		return;
	}

	const float pitch_step_deg = math::constrain(_param_scan_pitch_step.get(), 1.f, 30.f);
	advance_search_scan_area(pitch_step_deg);

	const ScanArea area = active_scan_area();
	const bool forward = (_scan_row % 2) == 0;
	const float row_end_deg = forward ? area.yaw_max_deg : area.yaw_min_deg;
	float pitch_deg = area.pitch_top_deg - static_cast<float>(_scan_row) * pitch_step_deg;

	if (pitch_deg < area.pitch_bottom_deg) {
		pitch_deg = area.pitch_bottom_deg;
	}

	const float yaw_speed_deg_s = math::constrain(_param_scan_yaw_speed.get(), 0.1f, 90.f);
	const float edge_pause_s = math::constrain(_param_scan_edge_pause.get(), 0.f, 2.f);
	const float yaw_step_deg = math::constrain(_param_scan_yaw_step.get(), 5.f, 60.f);
	const float dwell_s = math::constrain(_param_scan_dwell.get(), 0.1f, 2.f);

	float current_yaw_deg = _scan_yaw_deg;

	if (_have_target && target_fresh() && PX4_ISFINITE(_last_target.gimbal_yaw_rad)) {
		current_yaw_deg = math::degrees(_last_target.gimbal_yaw_rad);
	}

	if (!PX4_ISFINITE(_scan_segment_target_deg)) {
		if (fabsf(row_end_deg - current_yaw_deg) <= yaw_step_deg * 0.5f) {
			++_scan_row;
			advance_search_scan_area(pitch_step_deg);
			_scan_segment_target_deg = NAN;
			_search_pause_until = now + static_cast<hrt_abstime>(edge_pause_s * 1e6f);
			_scan_yaw_deg = current_yaw_deg;
			_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
			return;
		}

		if (forward) {
			_scan_segment_target_deg = math::min(current_yaw_deg + yaw_step_deg, row_end_deg);
		} else {
			_scan_segment_target_deg = math::max(current_yaw_deg - yaw_step_deg, row_end_deg);
		}
	}

	const float target_yaw = _scan_segment_target_deg;
	const float travel_deg = fabsf(target_yaw - current_yaw_deg);
	const float travel_s = (yaw_speed_deg_s > 0.1f) ? (travel_deg / yaw_speed_deg_s) : 1.f;

	send_scan_angle(target_yaw, pitch_deg);

	const bool at_row_end = fabsf(target_yaw - row_end_deg) < 1.f;

	if (at_row_end) {
		_scan_segment_target_deg = NAN;
	} else {
		if (forward) {
			_scan_segment_target_deg = math::min(target_yaw + yaw_step_deg, row_end_deg);
		} else {
			_scan_segment_target_deg = math::max(target_yaw - yaw_step_deg, row_end_deg);
		}
	}

	const float wait_s = travel_s + dwell_s;
	_search_pause_until = now + static_cast<hrt_abstime>(wait_s * 1e6f);
	_scan_yaw_deg = target_yaw;
	_scan_pitch_deg = pitch_deg;
	_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
}

void DytGuidance::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	update_params_if_needed();
	update_subscriptions();

	const hrt_abstime now = hrt_absolute_time();
	const bool activation_request = activation_requested();
	const bool intercept_request = aux_switch_active(_param_int_aux.get());

	if (activation_request && !_prev_activation_request) {
		activate_guidance(now);
	}

	if (!activation_request && _prev_activation_request) {
		deactivate_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
	}

	_prev_activation_request = activation_request;
	_requested_submode = intercept_request ? dyt_guidance_status_s::SUBMODE_INTERCEPT :
			     dyt_guidance_status_s::SUBMODE_FOLLOW;

	if (activation_request && (_state == TaskState::Idle || _state == TaskState::Abort)) {
		update_payload_only_reacquire(now);
	}

	if (_state != TaskState::Idle && _state != TaskState::Abort) {
		if (!preconditions_ok()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
		} else if ((now - _state_enter_time) > MANUAL_TAKEOVER_GRACE && manual_takeover_detected()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_MANUAL);
		}
	}

	switch (_state) {
	case TaskState::Idle:
		break;

	case TaskState::SearchWaitLock:
		if (target_usable()) {
			++_lock_streak;

			if (_lock_streak >= _param_lock_frames.get()) {
				enter_state(TaskState::TrackFollow);
			}

		} else {
			_lock_streak = 0;

			if (target_lock_candidate()) {
				update_hint_autolock(now);
				_search_pause_until = now + 1200_ms;
				_next_scan_time = now + 100_ms;
			} else {
				retry_autolock(now);
			}

			if ((now - _state_enter_time) > static_cast<hrt_abstime>(_param_wait_ms.get()) * 1000ULL) {
				enter_lost_hold(dyt_guidance_status_s::LOST_REASON_TIMEOUT);
			}
		}
		break;

	case TaskState::TrackFollow:
		if (!target_usable()) {
			++_lost_streak;

			if (_lost_streak >= _param_relock_frames.get()) {
				enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
						 dyt_guidance_status_s::LOST_REASON_TRACKING);
			}

		} else {
			_lost_streak = 0;

			if (intercept_request && intercept_allowed()) {
				enter_state(TaskState::TrackIntercept);
			}
		}
		break;

	case TaskState::TrackIntercept:
		if (!target_usable()) {
			++_lost_streak;

			if (_lost_streak >= _param_relock_frames.get()) {
				enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
						 dyt_guidance_status_s::LOST_REASON_TRACKING);
			}

		} else {
			_lost_streak = 0;

			if (!intercept_request || !intercept_allowed()) {
				enter_state(TaskState::TrackFollow);
			}
		}
		break;

	case TaskState::LostHold:
		if (target_usable()) {
			++_relock_streak;

			if (_relock_streak >= _param_relock_frames.get()) {
				enter_state(intercept_request && intercept_allowed() ? TaskState::TrackIntercept : TaskState::TrackFollow);
			}

		} else {
			_relock_streak = 0;

			const int32_t center_ms = math::max(_param_center_ms.get(), int32_t{0});
			const hrt_abstime center_delay = static_cast<hrt_abstime>(center_ms) * 1000ULL;
			const hrt_abstime lost_timeout = static_cast<hrt_abstime>(math::max(_param_lost_ms.get(), int32_t{0})) * 1000ULL;
			const bool center_done = (now - _state_enter_time) >= center_delay;

			// 建议：DYTG_LOSTMS=0 时不要自动停止搜索
			if (center_done && lost_timeout > 0 && (now - _state_enter_time) > center_delay + lost_timeout) {
				abort_guidance(_lost_reason);
			} else if (center_done) {
				update_search_scan(now);

				// 只有没有候选目标时才普通 retry
				if (!target_lock_candidate()) {
					retry_autolock(now);
				}
			}
		}
		break;

	case TaskState::Abort:
		enter_state(TaskState::Idle, dyt_guidance_status_s::LOST_REASON_NONE);
		break;
	}

	if (_state == TaskState::SearchWaitLock || _state == TaskState::LostHold) {
		request_offboard_mode();
		publish_offboard_mode(true);
		publish_hold_setpoint();
	}

	if (_state == TaskState::TrackFollow) {
		request_offboard_mode();
		publish_offboard_mode(false);
		publish_track_setpoint(follow_profile());
	}

	if (_state == TaskState::TrackIntercept) {
		request_offboard_mode();
		publish_offboard_mode(false);
		publish_track_setpoint(intercept_profile());
	}

	publish_status();
}

int DytGuidance::task_spawn(int argc, char *argv[])
{
	DytGuidance *instance = new DytGuidance();

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

int DytGuidance::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return print_usage("module not running");
	}

	if (!strcmp(argv[0], "status")) {
		get_instance()->show_status();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "retrigger")) {
		get_instance()->send_dyt_command(dyt_command_s::CMD_RETRIGGER, -100);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int DytGuidance::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
DYT visual guidance controller that consumes DYT telemetry, maintains a small LOS observation buffer,
and publishes offboard trajectory setpoints for follow and intercept behaviors.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dyt_guidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("retrigger");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int dyt_guidance_main(int argc, char *argv[])
{
	return DytGuidance::main(argc, argv);
}
