/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "dyt_guidance.hpp"

#include <math.h>
#include <string.h>

#include <px4_platform_common/log.h>

#include <lib/mathlib/mathlib.h>

/**
 * @brief 构造制导模块实例。
 *
 * 输入：无。
 * 输出：初始化基于工作队列的模块对象。
 * 功能：将控制器挂接到 PX4 工作队列，并清空缓存的目标数据。
 */
DytGuidance::DytGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	memset(&_last_target, 0, sizeof(_last_target));
}

/**
 * @brief 初始化模块的周期执行。
 *
 * 输入：无。
 * @return 周期循环配置完成后返回 true。
 * 功能：以 20 ms 周期启动控制器的 Run 循环。
 */
bool DytGuidance::init()
{
	ScheduleOnInterval(20_ms);
	return true;
}

/**
 * @brief 打印当前控制器状态。
 *
 * 输入：缓存的模块状态、目标状态和参数配置。
 * 输出：向 PX4 日志输出供操作员查看的状态信息。
 * 功能：暴露高层级运行诊断信息。
 */
void DytGuidance::show_status()
{
	PX4_INFO("state: %u", static_cast<unsigned>(_state));
	PX4_INFO("target fresh: %d", target_fresh());
	PX4_INFO("target locked: %d", target_locked());
	PX4_INFO("observations: %d", _observation_count);
	PX4_INFO("activation aux: %d", _param_act_aux.get());
	PX4_INFO("intercept aux: %d", _param_int_aux.get());
}

/**
 * @brief 当 PX4 报告参数变化时刷新参数包装器。
 *
 * 输入：来自 uORB 的参数更新通知。
 * 输出：更新控制器使用的缓存参数值。
 * 功能：使运行时行为与最新参数配置保持同步。
 */
void DytGuidance::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

/**
 * @brief 将最新订阅数据拉取到本地缓存。
 *
 * 输入：待处理的飞行器、遥控和目标话题更新。
 * 输出：刷新缓存的话题结构体，并将新目标样本交给后续处理。
 * 功能：在每次控制循环开始时集中完成所有订阅更新。
 */
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

/**
 * @brief 按编号读取一个遥控 AUX 通道。
 *
 * @param index AUX 通道编号，范围为 1..6。
 * @return 编号有效时返回通道值，否则返回 NAN。
 * 功能：将通用 AUX 编号转换为对应的 PX4 遥控字段。
 */
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

/**
 * @brief 将 AUX 通道值转换为布尔激活状态。
 *
 * @param index AUX 通道编号，范围为 1..6。
 * @return 当通道值有限且大于 0.5 时返回 true。
 * 功能：把飞手的 AUX 输入解释为开关量。
 */
bool DytGuidance::aux_switch_active(int index) const
{
	const float value = aux_value(index);
	return PX4_ISFINITE(value) && value > 0.5f;
}

/**
 * @brief 检查当前是否允许自主制导运行。
 *
 * 输入：缓存的解锁状态、失效保护状态和本地位置有效性状态。
 * @return 最低飞行器状态前置条件满足时返回 true。
 * 功能：以基本安全条件限制制导激活和持续控制。
 */
bool DytGuidance::preconditions_ok() const
{
	return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && !_vehicle_status.failsafe
	       && _vehicle_local_position.xy_valid
	       && _vehicle_local_position.z_valid;
}

/**
 * @brief 检测操作员是否通过杆量进行接管。
 *
 * 输入：遥控设定值和接管阈值参数。
 * @return 当 roll、pitch 或 yaw 杆量超过接管阈值时返回 true。
 * 功能：识别飞手干预并停止自主控制。
 */
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

/**
 * @brief 处理新收到的一条目标样本。
 *
 * @param target 最新视觉跟踪器输出。
 * 输出：当样本可用时向 LOS 观测历史中追加一条记录。
 * 功能：校验目标数据，并将其转换为缓存 LOS 历史使用的格式。
 */
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

/**
 * @brief 将 LOS 角度和云台姿态转换为机体系 LOS 向量。
 *
 * @param target 包含 LOS 角和云台角度的目标消息。
 * @param[out] los_body 转换成功时输出归一化的机体系 LOS 向量。
 * @return 成功生成有限且有效的 LOS 向量时返回 true。
 * 功能：在坐标旋转前应用配置的符号和零偏修正。
 */
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

/**
 * @brief 向有界观测历史中插入一条 LOS 样本。
 *
 * @param los_body 机体系 LOS 单位向量。
 * @param sample_time 样本时间戳。
 * @param frame_dt_s 数据源帧间隔，单位为秒。
 * 输出：更新观测缓冲区和最新 LOS 缓存。
 * 功能：维护一段最近 LOS 历史，用于滤波和预测。
 */
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

/**
 * @brief 清空已保存的 LOS 观测和派生滤波状态。
 *
 * 输入：无。
 * 输出：清空观测缓冲区并重置滤波 LOS 估计。
 * 功能：将 LOS 估计器恢复到初始状态。
 */
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

/**
 * @brief 更新 NED 坐标系下预测并滤波后的 LOS 估计。
 *
 * @param now 当前时间戳，用于外推和求导。
 * @return LOS 估计成功时返回 true，否则返回 false。
 * 功能：预测最新 LOS 样本，将其旋转到 NED 系，并估计 LOS 角速度。
 */
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

/**
 * @brief 将当前飞行器位姿保存为悬停参考。
 *
 * 输入：缓存的飞行器本地位置和姿态。
 * 输出：更新模块中的悬停位置和悬停偏航成员。
 * 功能：记录一个稳定的回退设定值，用于悬停控制。
 */
void DytGuidance::capture_hold_setpoint()
{
	_hold_position(0) = _vehicle_local_position.x;
	_hold_position(1) = _vehicle_local_position.y;
	_hold_position(2) = _vehicle_local_position.z;
	_hold_yaw = Eulerf(Quatf(_vehicle_attitude.q)).psi();
}

/**
 * @brief 检查跟踪器当前是否报告有效锁定。
 *
 * 输入：缓存的最近一条目标样本。
 * @return 目标有效且处于 locked 跟踪状态时返回 true。
 * 功能：区分真正锁定和普通的目标存在状态。
 */
bool DytGuidance::target_locked() const
{
	return _have_target && _last_target.tracking_state == dyt_target_s::TRACKING_STATE_LOCKED && _last_target.target_valid;
}

/**
 * @brief 检查最新目标样本是否足够新鲜，可用于控制。
 *
 * 输入：缓存的目标时间戳、帧间隔和新鲜度参数。
 * @return 目标年龄和帧间隔都在配置范围内时返回 true。
 * 功能：剔除陈旧、超时或错误的跟踪输出。
 */
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

/**
 * @brief 检查目标是否已具备主动制导条件。
 *
 * 输入：目标锁定状态、新鲜度状态和 LOS 观测数量。
 * @return 控制器拥有已锁定、足够新鲜且带 LOS 缓存的目标时返回 true。
 * 功能：为跟踪模式提供统一的可用性判断。
 */
bool DytGuidance::target_usable() const
{
	return target_locked() && target_fresh() && _observation_count > 0;
}

/**
 * @brief 检查当前是否允许进入拦截模式。
 *
 * 输入：目标可用性、时延设置、帧间隔和最新 LOS 方向。
 * @return 时序和几何约束都允许时返回 true。
 * 功能：以比跟随模式更严格的条件保护拦截模式。
 */
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

/**
 * @brief 构造跟随模式使用的控制参数组合。
 *
 * 输入：跟随模式调参。
 * @return 返回跟随模式的制导增益和指令速度。
 * 功能：将跟随模式调参打包成可复用结构体。
 */
DytGuidance::TrackProfile DytGuidance::follow_profile() const
{
	return {_param_n_follow.get(), _param_v_follow.get(), _param_ka_follow.get(), _param_kv_follow.get(),
		dyt_guidance_status_s::SUBMODE_FOLLOW};
}

/**
 * @brief 构造拦截模式使用的控制参数组合。
 *
 * 输入：拦截模式调参。
 * @return 返回拦截模式的制导增益和指令速度。
 * 功能：将拦截模式调参打包成可复用结构体。
 */
DytGuidance::TrackProfile DytGuidance::intercept_profile() const
{
	return {_param_n_intercept.get(), _param_v_intercept.get(), _param_ka_intercept.get(), _param_kv_intercept.get(),
		dyt_guidance_status_s::SUBMODE_INTERCEPT};
}

/**
 * @brief 发布悬停位置轨迹设定值。
 *
 * 输入：缓存的悬停位置和悬停偏航角。
 * 输出：发布一个用于位置保持的 `trajectory_setpoint`。
 * 功能：指令飞行器保持在记录下来的悬停位姿。
 */
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

/**
 * @brief 使用当前制导参数发布跟踪设定值。
 *
 * @param profile 跟随或拦截行为对应的制导增益和速度指令。
 * 输出：发布速度、加速度、偏航和偏航角速度设定值。
 * 功能：根据 LOS 几何关系和当前飞行器速度计算运动指令。
 */
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

/**
 * @brief 发布当前启用的 offboard 控制字段。
 *
 * @param hold_mode 为 true 表示位置保持，为 false 表示速度跟踪。
 * 输出：发布 `offboard_control_mode` 消息。
 * 功能：让 PX4 的 offboard 模式标志与下一条设定值类型保持一致。
 */
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

/**
 * @brief 周期性发送切换或保持 offboard 模式的请求。
 *
 * 输入：当前时间、上次请求时间、当前导航模式和飞行器标识信息。
 * 输出：未进入 offboard 前最多以 5 Hz 发布请求，进入后降为 1 Hz 保活。
 * 功能：在制导工作期间尽快切入并持续维持 PX4 处于 offboard 模式。
 */
void DytGuidance::request_offboard_mode()
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime request_period = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
					   ? 1_s : 200_ms;

	if ((now - _last_offboard_request) < request_period) {
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

/**
 * @brief 发布内部制导状态汇总信息。
 *
 * 输入：控制器状态、LOS 估计和最新输出设定值。
 * 输出：发布 `dyt_guidance_status` 消息。
 * 功能：为遥测、调试和监控暴露模块状态。
 */
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

/**
 * @brief 切换任务状态机并执行进入动作。
 *
 * @param new_state 目标控制器状态。
 * @param lost_reason 与本次切换相关的原因码。
 * 输出：更新状态记录，并在需要时执行各状态的复位逻辑。
 * 功能：集中处理所有状态切换及配套清理动作。
 */
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
		_last_retrigger_time = 0;
		clear_observations();
		capture_hold_setpoint();
		send_dyt_command(dyt_command_s::CMD_CENTER_GIMBAL);
	} else if (new_state == TaskState::Idle) {
		clear_observations();
		_velocity_sp.zero();
		_acceleration_sp.zero();
		_yaw_sp = NAN;
		_yaw_rate_sp = NAN;
	}
}

/**
 * @brief 启动一轮新的自主制导流程。
 *
 * 输入：当前飞行器前置条件状态和当前位姿。
 * 输出：重置控制器状态、记录悬停位姿，并向跟踪器发送自动锁定命令。
 * 功能：将模块从空闲状态推进到搜索和跟踪流程。
 */
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

/**
 * @brief 中止当前制导流程。
 *
 * @param lost_reason 本次中止记录的原因码。
 * 输出：命令跟踪器停止，并将状态机推进到中止状态。
 * 功能：在出现阻塞条件时终止自主制导。
 */
void DytGuidance::abort_guidance(uint8_t lost_reason)
{
	send_dyt_command(dyt_command_s::CMD_STOP_TRACK);
	enter_state(TaskState::Abort, lost_reason);
}

/**
 * @brief 进入临时丢目标保持状态。
 *
 * @param lost_reason 描述目标跟踪丢失的原因码。
 * 输出：携带该原因码将控制器切换到保持状态。
 * 功能：在等待目标重新锁定期间维持飞行器稳定。
 */
void DytGuidance::enter_lost_hold(uint8_t lost_reason)
{
	enter_state(TaskState::LostHold, lost_reason);
}

/**
 * @brief 在丢目标保持期间主动重捕获目标。
 *
 * @param now 当前高精度时间戳。
 * 输出：回中等待结束后按周期发送重新触发锁定命令。
 * 功能：处理吊舱丢锁后退出跟踪且停在边界角的情况。
 */
void DytGuidance::update_lost_reacquire(hrt_abstime now)
{
	const int center_ms = _param_center_ms.get() > 0 ? _param_center_ms.get() : 0;
	const hrt_abstime center_delay = static_cast<hrt_abstime>(center_ms) * 1000ULL;

	if ((now - _state_enter_time) < center_delay) {
		return;
	}

	const int retry_ms = _param_retry_ms.get() > 0 ? _param_retry_ms.get() : 1000;
	const hrt_abstime retry_interval = static_cast<hrt_abstime>(retry_ms) * 1000ULL;

	if (_last_retrigger_time == 0 || (now - _last_retrigger_time) >= retry_interval) {
		send_dyt_command(dyt_command_s::CMD_RETRIGGER, -100);
		_last_retrigger_time = now;
	}
}

/**
 * @brief 向 DYT 跟踪器接口发布命令。
 *
 * @param command 要发送的命令标识。
 * @param param_x 命令附带的可选整型参数。
 * @param param_y 命令附带的第二个可选整型参数。
 * 输出：发布 `dyt_command` 消息。
 * 功能：发送自动锁定、停止、重新触发等跟踪器控制请求。
 */
void DytGuidance::send_dyt_command(uint8_t command, int16_t param_x, int16_t param_y)
{
	dyt_command_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.command = command;
	msg.param_x = param_x;
	msg.param_y = param_y;
	_dyt_command_pub.publish(msg);
}

/**
 * @brief 执行一次完整的控制器更新周期。
 *
 * 输入：订阅数据、参数、飞手请求和内部状态机状态。
 * 输出：状态切换、offboard 模式请求、轨迹设定值和状态遥测。
 * 功能：从感知处理到指令发布，完整运行一次制导状态机。
 */
void DytGuidance::Run()
{
	// 模块被要求退出时，停止调度并完成清理。
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 每个周期先刷新参数和订阅缓存，后续逻辑只读本地快照。
	update_params_if_needed();
	update_subscriptions();

	const hrt_abstime now = hrt_absolute_time();
	const bool activation_request = aux_switch_active(_param_act_aux.get());
	const bool intercept_request = aux_switch_active(_param_int_aux.get());

	// 激活开关上升沿时启动制导流程。
	if (activation_request && !_prev_activation_request) {
		activate_guidance();
	}

	// 激活开关下降沿时，若当前不在 Idle，则按前置条件失效中止。
	if (!activation_request && _prev_activation_request && _state != TaskState::Idle) {
		abort_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
	}

	// 缓存当前遥控请求，供下一周期做边沿检测和子模式选择。
	_prev_activation_request = activation_request;
	_requested_submode = intercept_request ? dyt_guidance_status_s::SUBMODE_INTERCEPT :
			     dyt_guidance_status_s::SUBMODE_FOLLOW;

	// 非 Idle/Abort 状态下持续检查安全前置条件和飞手接管。
	if (_state != TaskState::Idle && _state != TaskState::Abort) {
		if (!preconditions_ok()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
		} else if (manual_takeover_detected()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_MANUAL);
		}
	}

	switch (_state) {
	case TaskState::Idle:
		// 空闲态不主动做状态转换，只等待激活请求。
		break;

	case TaskState::SearchWaitLock:
		// 搜索等待锁定：累计连续有效目标帧，达到门限后进入跟踪。
		if (target_usable()) {
			++_lock_streak;

			if (_lock_streak >= _param_lock_frames.get()) {
				enter_state(TaskState::TrackFollow);
			}

		} else {
			// 一旦目标不连续可用，锁定计数清零；超时则进入失锁保持。
			_lock_streak = 0;

			if ((now - _state_enter_time) > static_cast<hrt_abstime>(_param_wait_ms.get()) * 1000ULL) {
				enter_lost_hold(dyt_guidance_status_s::LOST_REASON_TIMEOUT);
			}
		}
		break;

	case TaskState::TrackFollow:
		// 常规跟踪：目标失效则转 LostHold，满足条件且飞手请求时切到截获。
		if (!target_usable()) {
			enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
					 dyt_guidance_status_s::LOST_REASON_TRACKING);
		} else if (intercept_request && intercept_allowed()) {
			enter_state(TaskState::TrackIntercept);
		}
		break;

	case TaskState::TrackIntercept:
		// 截获跟踪：目标失效退入 LostHold，截获请求取消或条件不满足则退回 Follow。
		if (!target_usable()) {
			enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
					 dyt_guidance_status_s::LOST_REASON_TRACKING);
		} else if (!intercept_request || !intercept_allowed()) {
			enter_state(TaskState::TrackFollow);
		}
		break;

	case TaskState::LostHold:
		// 失锁保持：允许短时间等待重捕获，连续重捕获足够帧后恢复跟踪。
		if (target_usable()) {
			++_relock_streak;

			if (_relock_streak >= _param_relock_frames.get()) {
				enter_state(intercept_request && intercept_allowed() ? TaskState::TrackIntercept : TaskState::TrackFollow);
			}

		} else {
			// 重捕获中断则重新计数；超过保持时长后彻底中止本次制导。
			_relock_streak = 0;

			if ((now - _state_enter_time) > static_cast<hrt_abstime>(_param_lost_ms.get()) * 1000ULL) {
				abort_guidance(_lost_reason);
			} else {
				update_lost_reacquire(now);
			}
		}
		break;

	case TaskState::Abort:
		// Abort 作为过渡态，仅保留一个周期，下一拍回到 Idle。
		enter_state(TaskState::Idle, dyt_guidance_status_s::LOST_REASON_NONE);
		break;
	}

	// 搜索和失锁保持阶段仅维持 offboard+悬停，不输出跟踪机动。
	if (_state == TaskState::SearchWaitLock || _state == TaskState::LostHold) {
		request_offboard_mode();
		publish_offboard_mode(true);
		publish_hold_setpoint();
	}

	// Follow 子模式下发布常规跟踪轨迹。
	if (_state == TaskState::TrackFollow) {
		request_offboard_mode();
		publish_offboard_mode(false);
		publish_track_setpoint(follow_profile());
	}

	// Intercept 子模式下发布更激进的截获轨迹。
	if (_state == TaskState::TrackIntercept) {
		request_offboard_mode();
		publish_offboard_mode(false);
		publish_track_setpoint(intercept_profile());
	}

	// 无论状态如何都发布一次状态遥测，供上位机和日志观察。
	publish_status();
}

/**
 * @brief 创建并启动模块单例实例。
 *
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return 启动成功返回 PX4_OK，否则返回 PX4_ERROR。
 * 功能：分配模块对象、保存单例指针并初始化调度。
 */
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

/**
 * @brief 分发模块自定义命令行命令。
 *
 * @param argc 命令参数个数。
 * @param argv 命令参数数组。
 * @return 命令处理成功时返回 PX4_OK，否则返回 usage 的结果。
 * 功能：支持 `status`、`retrigger` 等运行时命令。
 */
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

/**
 * @brief 打印模块帮助文本和命令用法。
 *
 * @param reason 可选原因，会在帮助文本前打印；可以为 nullptr。
 * @return 打印完成后固定返回 0。
 * 功能：为模块启动和使用提供命令行说明。
 */
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

/**
 * @brief 供 PX4 shell 调用的模块 C 入口函数。
 *
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return 返回 `DytGuidance::main` 的执行结果。
 * 功能：将 PX4 shell 的调用转发到 C++ 模块框架入口。
 */
extern "C" __EXPORT int dyt_guidance_main(int argc, char *argv[])
{
	return DytGuidance::main(argc, argv);
}
