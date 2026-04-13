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

	void Run() override;

	void update_subscriptions();
	void update_params_if_needed();

	float aux_value(int index) const;
	bool aux_switch_active(int index) const;
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
	void activate_guidance();
	void abort_guidance(uint8_t lost_reason);
	void enter_lost_hold(uint8_t lost_reason);

	bool target_locked() const;
	bool target_fresh() const;
	bool target_usable() const;
	bool intercept_allowed() const;

	TrackProfile follow_profile() const;
	TrackProfile intercept_profile() const;

	void send_dyt_command(uint8_t command, int16_t param_x = 0);

	TaskState _state{TaskState::Idle};
	uint8_t _lost_reason{dyt_guidance_status_s::LOST_REASON_NONE};
	uint8_t _requested_submode{dyt_guidance_status_s::SUBMODE_FOLLOW};

	bool _prev_activation_request{false};
	int _lock_streak{0};
	int _relock_streak{0};

	hrt_abstime _state_enter_time{0};
	hrt_abstime _last_offboard_request{0};

	LosObservation _observations[OBS_BUFFER_LEN]{};
	int _observation_count{0};

	dyt_target_s _last_target{};
	bool _have_target{false};

	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_status_s _vehicle_status{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	manual_control_setpoint_s _manual_control{};

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
	PX4_INFO("observations: %d", _observation_count);
	PX4_INFO("activation aux: %d", _param_act_aux.get());
	PX4_INFO("intercept aux: %d", _param_int_aux.get());
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
	if (!target.target_valid) {
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
	return _have_target && _last_target.tracking_state == dyt_target_s::TRACKING_STATE_LOCKED && _last_target.target_valid;
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

bool DytGuidance::target_usable() const
{
	return target_locked() && target_fresh() && _observation_count > 0;
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
	} else if (new_state == TaskState::LostHold) {
		_relock_streak = 0;
		capture_hold_setpoint();
	} else if (new_state == TaskState::Idle) {
		clear_observations();
		_velocity_sp.zero();
		_acceleration_sp.zero();
		_yaw_sp = NAN;
		_yaw_rate_sp = NAN;
	}
}

void DytGuidance::activate_guidance()
{
	if (!preconditions_ok()) {
		PX4_WARN("DYT guidance preconditions not met");
		return;
	}

	clear_observations();
	capture_hold_setpoint();
	_requested_submode = dyt_guidance_status_s::SUBMODE_FOLLOW;
	send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
	enter_state(TaskState::SearchWaitLock);
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

void DytGuidance::send_dyt_command(uint8_t command, int16_t param_x)
{
	dyt_command_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.command = command;
	msg.param_x = param_x;
	_dyt_command_pub.publish(msg);
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
	const bool activation_request = aux_switch_active(_param_act_aux.get());
	const bool intercept_request = aux_switch_active(_param_int_aux.get());

	if (activation_request && !_prev_activation_request) {
		activate_guidance();
	}

	if (!activation_request && _prev_activation_request && _state != TaskState::Idle) {
		abort_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
	}

	_prev_activation_request = activation_request;
	_requested_submode = intercept_request ? dyt_guidance_status_s::SUBMODE_INTERCEPT :
			     dyt_guidance_status_s::SUBMODE_FOLLOW;

	if (_state != TaskState::Idle && _state != TaskState::Abort) {
		if (!preconditions_ok()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
		} else if (manual_takeover_detected()) {
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

			if ((now - _state_enter_time) > static_cast<hrt_abstime>(_param_wait_ms.get()) * 1000ULL) {
				enter_lost_hold(dyt_guidance_status_s::LOST_REASON_TIMEOUT);
			}
		}
		break;

	case TaskState::TrackFollow:
		if (!target_usable()) {
			enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
					 dyt_guidance_status_s::LOST_REASON_TRACKING);
		} else if (intercept_request && intercept_allowed()) {
			enter_state(TaskState::TrackIntercept);
		}
		break;

	case TaskState::TrackIntercept:
		if (!target_usable()) {
			enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
					 dyt_guidance_status_s::LOST_REASON_TRACKING);
		} else if (!intercept_request || !intercept_allowed()) {
			enter_state(TaskState::TrackFollow);
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

			if ((now - _state_enter_time) > static_cast<hrt_abstime>(_param_lost_ms.get()) * 1000ULL) {
				abort_guidance(_lost_reason);
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
