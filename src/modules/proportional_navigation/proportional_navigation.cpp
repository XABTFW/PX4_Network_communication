/**
 * @file proportional_navigation.cpp
 * 基于PPN的碰撞角约束制导策略（PPNIACG）实现
 * 严格按照论文公式实现
 */

#include "proportional_navigation.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

ProportionalNavigation::ProportionalNavigation() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_current_position.zero();
	_current_velocity.zero();
	_target_position.zero();
	_commanded_acceleration.zero();
	_commanded_velocity.zero();
	_radar_filtered_position.zero();
	_radar_filtered_velocity.zero();
	_radar_predicted_position.zero();
	_radar_predicted_velocity.zero();
}

ProportionalNavigation::~ProportionalNavigation()
{
}

bool ProportionalNavigation::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void ProportionalNavigation::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 计算时间步长
	hrt_abstime now = hrt_absolute_time();
	if (_last_run_time != 0) {
		_dt = (now - _last_run_time) / 1e6f;
		_dt = math::constrain(_dt, 0.001f, 0.1f);
	}
	_last_run_time = now;

	// 更新状态
	updateState();

	// 自动跟踪雷达目标（增强版）
	if (_auto_track_radar && (_mission_state == MissionState::ENGAGING || _mission_state == MissionState::GROUND_STRIKE)) {
		updateRadarTarget();
	}

	// 如果任务激活，发送控制指令
	if (_mission_state != MissionState::IDLE) {
		if (!safetyCheck()) {
			emergencyAbort();
			return;
		}

		updateMissionState();

		if (_mission_state == MissionState::TAKEOFF) {
			executeTakeoff();
		} else if (_mission_state == MissionState::ENGAGING) {
			executeEngagement();
		} else if (_mission_state == MissionState::GROUND_STRIKE) {
			executeGroundStrike();
		} else if (_mission_state == MissionState::PULLUP) {
			executePullup();
		}
	}

	// 始终发送 setpoint（Offboard 模式需要持续信号）
	publishSetpoint();
}

void ProportionalNavigation::updateState()
{
	vehicle_local_position_s local_pos;
	if (_vehicle_local_position_sub.update(&local_pos)) {
		_current_position(0) = local_pos.x;
		_current_position(1) = local_pos.y;
		_current_position(2) = local_pos.z;

		_current_velocity(0) = local_pos.vx;
		_current_velocity(1) = local_pos.vy;
		_current_velocity(2) = local_pos.vz;

		// 初始化地图投影（使用本地位置的参考点）
		if (!_map_projection_initialized && local_pos.ref_timestamp > 0) {
			if (PX4_ISFINITE(local_pos.ref_lat) && PX4_ISFINITE(local_pos.ref_lon) &&
			    fabsf(local_pos.ref_lat) > 0.0001f && fabsf(local_pos.ref_lon) > 0.0001f) {
				_map_projection.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);
				_map_projection_initialized = true;
				PX4_INFO("Map projection initialized: lat=%.7f, lon=%.7f",
				         local_pos.ref_lat, local_pos.ref_lon);
			}
		}
	}

	vehicle_attitude_s attitude;
	if (_vehicle_attitude_sub.update(&attitude)) {
		_current_attitude = matrix::Quatf(attitude.q);
	}

	// 如果使用GPS目标，转换为NED坐标
	if (_use_gps_target && _map_projection_initialized) {
		float target_x, target_y;
		_map_projection.project(_target_lat, _target_lon, target_x, target_y);

		// 获取当前高度（AMSL）和home高度
		vehicle_global_position_s global_pos;
		if (_vehicle_global_position_sub.copy(&global_pos)) {
			// NED坐标：Z轴向下为正，所以 target_z = -(target_alt - home_alt)
			// 如果 target_alt < home_alt（地面），则 target_z > 0（向下）
			float home_alt = global_pos.alt - (-_current_position(2));  // 计算home点的AMSL高度
			float target_z = -((_target_alt - home_alt));  // NED坐标

			_target_position(0) = target_x;
			_target_position(1) = target_y;
			_target_position(2) = target_z;
		}
	}
}

void ProportionalNavigation::updateMissionState()
{
	switch (_mission_state) {
	case MissionState::IDLE:
		break;

	case MissionState::TAKEOFF: {
		// 检查是否到达起飞高度
		if (isAirborne() && (-_current_position(2)) >= (_takeoff_altitude - 2.0f)) {
			PX4_INFO("Takeoff complete! Altitude: %.2f m", (double)(-_current_position(2)));

			// 初始化打击任务
			initializeEngagement();

			// 给一个较高的初始速度，指向目标方向
			matrix::Vector3f to_target = _target_position - _current_position;
			float distance = to_target.norm();

			if (distance > 1.0f) {
				// 初始速度：20 m/s 指向目标（提高初速度）
				_commanded_velocity = to_target.normalized() * 20.0f;
				PX4_INFO("Initial velocity toward target: (%.1f, %.1f, %.1f) m/s",
				         (double)_commanded_velocity(0),
				         (double)_commanded_velocity(1),
				         (double)_commanded_velocity(2));
			} else {
				_commanded_velocity = _current_velocity;
			}

			// 判断任务类型：如果目标在地面附近，进入地面打击模式
			float target_altitude = -_target_position(2);  // NED坐标转为正高度
			if (target_altitude < 5.0f && _target_velocity.norm() < 0.1f) {
				// 地面固定目标
				_mission_state = MissionState::GROUND_STRIKE;
				PX4_INFO("Entering GROUND_STRIKE mode");
			} else {
				// 空中目标或运动目标
				_mission_state = MissionState::ENGAGING;
				PX4_INFO("Entering ENGAGING mode");
			}

			_mission_start_time = hrt_absolute_time();
		}

		// 起飞超时检查（30秒）
		if ((hrt_absolute_time() - _takeoff_start_time) > 30_s) {
			PX4_WARN("Takeoff timeout");
			_mission_state = MissionState::IDLE;
		}
		break;
	}

	case MissionState::ENGAGING:
		// 检查是否需要拉升
		if (_param_pn_enable_pullup.get() && _r < _param_pn_pullup_dist.get()) {
			PX4_INFO("Pullup triggered! Range: %.2f m", (double)_r);
			_mission_state = MissionState::PULLUP;
			_pullup_start_time = hrt_absolute_time();
			_pullup_target_altitude = _param_pn_pullup_alt.get();
			break;
		}

		// 检查是否命中目标（无拉升模式）
		if (!_param_pn_enable_pullup.get() && _r < _param_pn_miss_dist.get()) {
			PX4_INFO("Target hit! Range: %.2f m", (double)_r);
			_mission_state = MissionState::COMPLETE;
		}

		// 超时检查
		if ((hrt_absolute_time() - _mission_start_time) > 30_s) {
			PX4_WARN("Mission timeout");
			_mission_state = MissionState::COMPLETE;
		}
		break;

	case MissionState::GROUND_STRIKE: {
		// 地面打击模式的状态转换
		// 检查是否需要拉升（强制）
		if (_r < _param_pn_pullup_dist.get()) {
			PX4_INFO("Ground strike: Pullup triggered! Range: %.2f m", (double)_r);
			_mission_state = MissionState::PULLUP;
			_pullup_start_time = hrt_absolute_time();
			_pullup_target_altitude = -(_param_pn_pullup_alt.get());  // NED坐标
			break;
		}

		// 高度安全检查
		float current_altitude = -_current_position(2);
		if (current_altitude < (_param_pn_pullup_dist.get() + 5.0f)) {
			PX4_WARN("Ground strike: Low altitude, forcing pullup");
			_mission_state = MissionState::PULLUP;
			_pullup_start_time = hrt_absolute_time();
			_pullup_target_altitude = -(_param_pn_pullup_alt.get());
			break;
		}

		// 超时检查
		if ((hrt_absolute_time() - _mission_start_time) > 30_s) {
			PX4_WARN("Ground strike timeout");
			_mission_state = MissionState::COMPLETE;
		}
		break;
	}

	case MissionState::PULLUP:
		// 检查是否到达目标高度
		if (_current_position(2) <= _pullup_target_altitude) {
			PX4_INFO("Pullup complete! Altitude: %.2f m", (double)(-_current_position(2)));
			_mission_state = MissionState::COMPLETE;
		}

		// 拉升超时检查（10秒）
		if ((hrt_absolute_time() - _pullup_start_time) > 10_s) {
			PX4_WARN("Pullup timeout");
			_mission_state = MissionState::COMPLETE;
		}
		break;

	case MissionState::COMPLETE:
		_mission_state = MissionState::IDLE;
		break;
	}
}

void ProportionalNavigation::initializeEngagement()
{
	// 初始化时计算初始状态（关键！）
	// 这是PPNIACG算法的基础

	// 1. 计算初始距离
	_r0 = (_target_position - _current_position).norm();

	// 2. 根据约束模式在不同平面内计算初始视线角和前置角
	int constraint_mode = _param_pn_constraint_mode.get();

	if (constraint_mode == 1) {
		// 铅垂面内：使用x-z平面（北-下平面）
		matrix::Vector2f relative_pos_2d;
		relative_pos_2d(0) = _target_position(0) - _current_position(0);  // x (北)
		relative_pos_2d(1) = _target_position(2) - _current_position(2);  // z (下)

		// 初始视线角
		_q0 = atan2f(relative_pos_2d(1), relative_pos_2d(0));

		// 速度在铅垂面内的投影
		matrix::Vector2f velocity_2d;
		velocity_2d(0) = _current_velocity(0);  // vx
		velocity_2d(1) = _current_velocity(2);  // vz

		// 速度方向角
		float velocity_angle = atan2f(velocity_2d(1), velocity_2d(0));

		// 初始前置角：速度方向与视线方向的夹角
		_theta0 = velocity_angle - _q0;

	} else if (constraint_mode == 2) {
		// 水平面内：使用x-y平面（北-东平面）
		matrix::Vector2f relative_pos_2d;
		relative_pos_2d(0) = _target_position(0) - _current_position(0);  // x (北)
		relative_pos_2d(1) = _target_position(1) - _current_position(1);  // y (东)

		// 初始视线角
		_q0 = atan2f(relative_pos_2d(1), relative_pos_2d(0));

		// 速度在水平面内的投影
		matrix::Vector2f velocity_2d;
		velocity_2d(0) = _current_velocity(0);  // vx
		velocity_2d(1) = _current_velocity(1);  // vy

		// 速度方向角
		float velocity_angle = atan2f(velocity_2d(1), velocity_2d(0));

		// 初始前置角
		_theta0 = velocity_angle - _q0;

	} else {
		// 标准PPN：三维空间
		matrix::Vector3f los_dir = (_target_position - _current_position).normalized();
		matrix::Vector3f vel_dir = _current_velocity.normalized();

		float cos_theta0 = vel_dir.dot(los_dir);
		_theta0 = acosf(math::constrain(cos_theta0, -1.0f, 1.0f));

		_q0 = 0.0f;
	}

	// 3. 计算初始视线角速率
	// 公式(1): v_θ = -v_m sin(θ) = r·q̇
	float v_m = _current_velocity.norm();
	if (_r0 > 0.1f && v_m > 0.1f) {
		_q_dot0 = -v_m * sinf(_theta0) / _r0;
	} else {
		_q_dot0 = 0.0f;
	}

	// 4. 初始化导引系数
	_current_N = _param_pn_nav_gain.get();
	_switched_to_ppn = false;

	// 5. 初始化当前距离（重要！）
	_r = _r0;

	PX4_INFO("=== Engagement Initialized ===");
	PX4_INFO("r0=%.1f m, q0=%.1f°, theta0=%.1f°, q_dot0=%.4f rad/s",
	         (double)_r0,
	         (double)math::degrees(_q0),
	         (double)math::degrees(_theta0),
	         (double)_q_dot0);
}

void ProportionalNavigation::executeEngagement()
{
	// 计算极坐标系状态
	_r = computeRange();
	_q = computeLOSAngle();
	_theta = computeLeadAngle();      // 使用解析解（公式18）
	_q_dot = computeLOSRate();        // 使用解析解（公式11）
	_v_r = computeRadialVelocity();
	_v_theta = computeNormalVelocity();



	// 计算PPNIACG加速度指令
	matrix::Vector3f accel_cmd = computePPNIACG();

	// 限制加速度
	float max_accel = _param_pn_max_accel.get();
	if (accel_cmd.norm() > max_accel) {
		accel_cmd = accel_cmd.normalized() * max_accel;
	}

	_commanded_acceleration = accel_cmd;

	// 积分得到速度
	_commanded_velocity += _commanded_acceleration * _dt;

	// 限制速度（可选）
	if (!_param_pn_no_vel_limit.get()) {
		float max_vel = _param_pn_max_vel.get();
		if (_commanded_velocity.norm() > max_vel) {
			_commanded_velocity = _commanded_velocity.normalized() * max_vel;
		}
	}
	// 如果禁用速度限制，则允许无限加速（仅受加速度限制）
}

matrix::Vector3f ProportionalNavigation::computePPNIACG()
{
	// 获取约束模式
	int constraint_mode = _param_pn_constraint_mode.get();

	float accel_magnitude = 0.0f;
	matrix::Vector3f accel_vector;
	accel_vector.zero();

	if (constraint_mode == 1) {
		// 铅垂面内落角约束
		float q_f_desired = math::radians(_param_pn_impact_angle_v.get());
		accel_magnitude = computeVerticalIACG(_r, _q, _theta, q_f_desired);

		// 加速度方向：在铅垂面内，垂直于速度方向
		// 铅垂面：x-z平面
		matrix::Vector2f velocity_2d(
			_current_velocity(0),  // vx
			_current_velocity(2)   // vz
		);

		float v_norm = velocity_2d.norm();
		if (v_norm > 0.1f) {
			// 垂直于速度的方向（逆时针旋转90度）
			matrix::Vector2f accel_dir_2d(
				-velocity_2d(1),  // -vz
				velocity_2d(0)    // vx
			);

			accel_dir_2d = accel_dir_2d.normalized();

			// 转换回3D（y方向为0）
			accel_vector(0) = accel_dir_2d(0) * accel_magnitude;
			accel_vector(1) = 0.0f;
			accel_vector(2) = accel_dir_2d(1) * accel_magnitude;
		}

	} else if (constraint_mode == 2) {
		// 水平面内碰撞角约束
		float q_f_desired = math::radians(_param_pn_impact_angle_h.get());
		accel_magnitude = computeHorizontalIACG(_r, _q, _theta, q_f_desired);

		// 加速度方向：在水平面内，垂直于速度方向
		// 水平面：x-y平面
		matrix::Vector2f velocity_2d(
			_current_velocity(0),  // vx
			_current_velocity(1)   // vy
		);

		float v_norm = velocity_2d.norm();
		if (v_norm > 0.1f) {
			// 垂直于速度的方向
			matrix::Vector2f accel_dir_2d(
				-velocity_2d(1),  // -vy
				velocity_2d(0)    // vx
			);

			accel_dir_2d = accel_dir_2d.normalized();

			// 转换回3D（z方向为0）
			accel_vector(0) = accel_dir_2d(0) * accel_magnitude;
			accel_vector(1) = accel_dir_2d(1) * accel_magnitude;
			accel_vector(2) = 0.0f;
		}

	} else {
		// 标准PPN（无碰撞角约束）
		// 如果目标运动，使用 TPN（真实比例导引）
		float v_effective = _current_velocity.norm();

		if (_target_velocity.norm() > 0.1f) {
			// TPN: 使用接近速度
			v_effective = computeClosingVelocity();
			if (v_effective < 1.0f) {
				v_effective = 1.0f;  // 避免除零
			}
		}

		accel_magnitude = _current_N * v_effective * fabsf(_q_dot);

		// 三维空间中垂直于速度的方向
		matrix::Vector3f velocity_dir = _current_velocity.normalized();
		matrix::Vector3f los_dir = (_target_position - _current_position).normalized();

		matrix::Vector3f accel_dir = los_dir - velocity_dir * los_dir.dot(velocity_dir);

		if (accel_dir.norm() > 0.01f) {
			accel_dir = accel_dir.normalized();
			accel_vector = accel_dir * accel_magnitude;
		}
	}

	return accel_vector;
}

float ProportionalNavigation::computeVerticalIACG(float r, float q, float theta, float q_f_desired)
{
	// 铅垂面内落角约束制导

	// 公式(28): q_f(t) = q(t) - theta(t)/(N-1)
	_q_f = computeFinalLOSAngle(q, theta, _current_N);

	// 判断是否需要转入PPN
	// 公式(29): 当 q(t) = (N-1)/N * q_f 时转入PPN
	if (!_switched_to_ppn && shouldSwitchToPPN(q, q_f_desired, _current_N)) {
		_switched_to_ppn = true;
		_current_N = _param_pn_nav_gain.get();  // 固定导引系数
		PX4_INFO("Switched to PPN mode, N=%.2f", (double)_current_N);
	}

	if (_switched_to_ppn) {
		// 标准PPN: a = N * v_m * |q̇|
		float v_m = _current_velocity.norm();
		float accel = _current_N * v_m * fabsf(_q_dot);

		return accel;

	} else {
		// PPNIACG阶段：实时调整导引系数
		// 公式(32): N(t) = 1 + theta(t) / (q(t) - q_f)
		_current_N = computeAdaptiveN(q, theta, q_f_desired);

		// PPN加速度
		float v_m = _current_velocity.norm();
		float accel = _current_N * v_m * fabsf(_q_dot);

		return accel;
	}
}

float ProportionalNavigation::computeHorizontalIACG(float r, float q, float theta, float q_f_desired)
{
	// 水平面内碰撞角约束制导
	// 算法与铅垂面类似
	return computeVerticalIACG(r, q, theta, q_f_desired);
}

float ProportionalNavigation::computeRange()
{
	// 距离 r
	matrix::Vector3f relative_pos = _target_position - _current_position;
	return relative_pos.norm();
}

float ProportionalNavigation::computeLOSAngle()
{
	// 视线角 q（在指定平面内）
	int constraint_mode = _param_pn_constraint_mode.get();

	if (constraint_mode == 1) {
		// 铅垂面：x-z平面
		matrix::Vector2f relative_pos_2d;
		relative_pos_2d(0) = _target_position(0) - _current_position(0);
		relative_pos_2d(1) = _target_position(2) - _current_position(2);

		return atan2f(relative_pos_2d(1), relative_pos_2d(0));

	} else if (constraint_mode == 2) {
		// 水平面：x-y平面
		matrix::Vector2f relative_pos_2d;
		relative_pos_2d(0) = _target_position(0) - _current_position(0);
		relative_pos_2d(1) = _target_position(1) - _current_position(1);

		return atan2f(relative_pos_2d(1), relative_pos_2d(0));

	} else {
		// 三维空间
		matrix::Vector3f relative_pos = _target_position - _current_position;
		return atan2f(relative_pos(1), relative_pos(0));
	}
}

float ProportionalNavigation::computeLOSRate()
{
	// 如果目标有速度，使用运动目标算法
	if (_target_velocity.norm() > 0.1f) {
		return computeLOSRateMovingTarget();
	}

	// 静止目标：公式(11): q̇ = q̇_0 (r/r_0)^(N-2)
	// 这是解析解，不是简化的几何关系

	if (_r0 < 0.1f || _r < 0.1f) {
		return 0.0f;
	}

	float r_ratio = _r / _r0;
	float exponent = _current_N - 2.0f;

	// 使用解析解
	float q_dot = _q_dot0 * powf(r_ratio, exponent);

	return q_dot;
}

float ProportionalNavigation::computeLOSRateMovingTarget()
{
	// 运动目标的视线角速率计算
	// q̇ = (V_t × R) / |R|²
	// 其中 V_t 是垂直于视线的相对速度分量

	matrix::Vector3f relative_pos = _target_position - _current_position;
	matrix::Vector3f relative_vel = _target_velocity - _current_velocity;

	float range_sq = relative_pos.norm_squared();
	if (range_sq < 0.01f) {
		return 0.0f;
	}

	// 计算垂直于视线的相对速度
	matrix::Vector3f los_dir = relative_pos.normalized();
	matrix::Vector3f vel_perpendicular = relative_vel - los_dir * relative_vel.dot(los_dir);

	// 视线角速率 = |V_perpendicular| / |R|
	float q_dot = vel_perpendicular.norm() / sqrtf(range_sq);

	// 判断旋转方向（叉乘）
	matrix::Vector3f cross = relative_pos % relative_vel;  // 叉乘
	if (cross(2) < 0.0f) {  // 根据约束模式选择轴
		q_dot = -q_dot;
	}

	return q_dot;
}

float ProportionalNavigation::computeClosingVelocity()
{
	// 接近速度（用于真实比例导引 TPN）
	matrix::Vector3f relative_pos = _target_position - _current_position;
	matrix::Vector3f relative_vel = _current_velocity - _target_velocity;

	if (relative_pos.norm() < 0.1f) {
		return 0.0f;
	}

	matrix::Vector3f los_dir = relative_pos.normalized();
	float v_closing = -relative_vel.dot(los_dir);  // 负号因为是接近

	return v_closing;
}

float ProportionalNavigation::computeLeadAngle()
{
	// 公式(18): sin(θ) = sin(θ_0) (r/r_0)^(N-1)
	// 这是解析解，不是直接计算速度与视线夹角

	if (_r0 < 0.1f || _r < 0.1f) {
		return _theta0;
	}

	float r_ratio = _r / _r0;
	float exponent = _current_N - 1.0f;

	// 使用解析解
	float sin_theta = sinf(_theta0) * powf(r_ratio, exponent);

	// 限制在[-1, 1]范围内
	sin_theta = math::constrain(sin_theta, -1.0f, 1.0f);

	float theta = asinf(sin_theta);

	return theta;
}

float ProportionalNavigation::computeRadialVelocity()
{
	// 径向速度 v_r = ṙ = -v_m cos(θ)
	float v_m = _current_velocity.norm();
	return -v_m * cosf(_theta);
}

float ProportionalNavigation::computeNormalVelocity()
{
	// 法向速度 v_θ = r·q̇ = -v_m sin(θ)
	float v_m = _current_velocity.norm();
	return -v_m * sinf(_theta);
}

float ProportionalNavigation::computeFinalLOSAngle(float q, float theta, float N)
{
	// 公式(28): q_f(t) = q(t) - theta(t) / (N-1)

	if (fabsf(N - 1.0f) < 0.01f) {
		return q;  // 避免除零
	}

	return q - theta / (N - 1.0f);
}

float ProportionalNavigation::computeAdaptiveN(float q, float theta, float q_f_desired)
{
	// 公式(32): N(t) = 1 + theta(t) / (q(t) - q_f)

	float denominator = q - q_f_desired;

	if (fabsf(denominator) < 0.01f) {
		// 接近目标，使用固定导引系数
		return _param_pn_nav_gain.get();
	}

	float N = 1.0f + theta / denominator;

	// 限制导引系数范围 [2, 6]
	N = math::constrain(N, 2.0f, 6.0f);

	return N;
}

bool ProportionalNavigation::shouldSwitchToPPN(float q, float q_f_desired, float N)
{
	// 公式(29): 转入PPN条件: q(t) = (N-1)/N * q_f

	if (fabsf(N) < 0.01f) {
		return false;
	}

	float threshold = (N - 1.0f) / N * q_f_desired;

	// 当视线角接近阈值时，转入标准PPN
	return fabsf(q - threshold) < math::radians(5.0f);  // 5度容差
}

void ProportionalNavigation::generateTrajectorySetpoint()
{
	// 在 publishSetpoint 中调用
}

void ProportionalNavigation::publishSetpoint()
{
	// 使用 trajectory_setpoint 进行 Offboard 控制
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();

	// 根据任务状态设置不同的控制指令
	if (_mission_state == MissionState::TAKEOFF) {
		// 起飞阶段：使用速度控制
		setpoint.position[0] = NAN;
		setpoint.position[1] = NAN;
		setpoint.position[2] = NAN;

		// 使用 executeTakeoff() 计算的速度
		setpoint.velocity[0] = _commanded_velocity(0);
		setpoint.velocity[1] = _commanded_velocity(1);
		setpoint.velocity[2] = _commanded_velocity(2);

		setpoint.acceleration[0] = NAN;
		setpoint.acceleration[1] = NAN;
		setpoint.acceleration[2] = NAN;

		setpoint.yaw = NAN;  // 保持当前偏航角

	} else if (_mission_state == MissionState::ENGAGING || _mission_state == MissionState::GROUND_STRIKE) {
		// 打击阶段（包括地面打击）：使用速度控制
		setpoint.position[0] = NAN;
		setpoint.position[1] = NAN;
		setpoint.position[2] = NAN;

		// 直接使用计算出的速度指令
		setpoint.velocity[0] = _commanded_velocity(0);
		setpoint.velocity[1] = _commanded_velocity(1);
		setpoint.velocity[2] = _commanded_velocity(2);

		// 加速度前馈
		setpoint.acceleration[0] = _commanded_acceleration(0);
		setpoint.acceleration[1] = _commanded_acceleration(1);
		setpoint.acceleration[2] = _commanded_acceleration(2);

		setpoint.yaw = NAN;

	} else if (_mission_state == MissionState::PULLUP) {
		// 拉升阶段：位置+速度控制
		setpoint.position[0] = _current_position(0);
		setpoint.position[1] = _current_position(1);
		setpoint.position[2] = _pullup_target_altitude;

		setpoint.velocity[0] = _commanded_velocity(0);
		setpoint.velocity[1] = _commanded_velocity(1);
		setpoint.velocity[2] = _commanded_velocity(2);

		setpoint.yaw = NAN;
	} else {
		// IDLE 或 COMPLETE 状态：发送悬停指令
		setpoint.position[0] = _current_position(0);
		setpoint.position[1] = _current_position(1);
		setpoint.position[2] = _current_position(2);

		setpoint.velocity[0] = 0.0f;
		setpoint.velocity[1] = 0.0f;
		setpoint.velocity[2] = 0.0f;

		setpoint.yaw = NAN;
	}

	_trajectory_setpoint_pub.publish(setpoint);

	// 同时发布 offboard_control_mode 以启用 Offboard 模式
	offboard_control_mode_s offboard_mode{};
	offboard_mode.timestamp = hrt_absolute_time();
	offboard_mode.position = (_mission_state == MissionState::PULLUP || _mission_state == MissionState::IDLE || _mission_state == MissionState::COMPLETE);
	offboard_mode.velocity = (_mission_state == MissionState::TAKEOFF || _mission_state == MissionState::ENGAGING || _mission_state == MissionState::GROUND_STRIKE);
	offboard_mode.acceleration = false;
	offboard_mode.attitude = false;
	offboard_mode.body_rate = false;

	_offboard_control_mode_pub.publish(offboard_mode);
}

bool ProportionalNavigation::safetyCheck()
{
	if (!PX4_ISFINITE(_current_position(0)) ||
	    !PX4_ISFINITE(_current_position(1)) ||
	    !PX4_ISFINITE(_current_position(2))) {
		PX4_ERR("Invalid position data");
		return false;
	}

	if (_current_velocity.norm() > 50.0f) {
		PX4_ERR("Velocity too high: %.2f m/s", (double)_current_velocity.norm());
		return false;
	}

	return true;
}

void ProportionalNavigation::emergencyAbort()
{
	PX4_ERR("Emergency abort!");

	_mission_state = MissionState::IDLE;
	_commanded_velocity.zero();
	_commanded_acceleration.zero();

	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE;
	cmd.param1 = 0.0f;
	cmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(cmd);
}

bool ProportionalNavigation::isAirborne()
{
	// 检查是否在空中：高度大于 1 米
	float altitude = -_current_position(2);
	return altitude > 1.0f;
}

void ProportionalNavigation::sendTakeoffCommand()
{
	// 先发送解锁指令
	vehicle_command_s arm_cmd{};
	arm_cmd.timestamp = hrt_absolute_time();
	arm_cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	arm_cmd.param1 = 1.0f;  // 1 = arm, 0 = disarm
	arm_cmd.target_system = 1;
	arm_cmd.target_component = 1;
	arm_cmd.source_system = 1;
	arm_cmd.source_component = 1;
	arm_cmd.from_external = false;

	_vehicle_command_pub.publish(arm_cmd);

	PX4_INFO("Arming vehicle...");

	// 切换到 Offboard 模式（不等待，让系统自动处理）
	vehicle_command_s mode_cmd{};
	mode_cmd.timestamp = hrt_absolute_time();
	mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	mode_cmd.param1 = 1.0f;  // 自定义模式
	mode_cmd.param2 = 6.0f;  // Offboard 模式
	mode_cmd.target_system = 1;
	mode_cmd.target_component = 1;
	mode_cmd.source_system = 1;
	mode_cmd.source_component = 1;
	mode_cmd.from_external = false;

	_vehicle_command_pub.publish(mode_cmd);

	PX4_INFO("Switching to Offboard mode, target altitude: %.1f m", (double)_takeoff_altitude);
}

int ProportionalNavigation::task_spawn(int argc, char *argv[])
{
	ProportionalNavigation *instance = new ProportionalNavigation();

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

int ProportionalNavigation::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("Module not running");
		return 1;
	}

	// ========== 雷达引导空对空拦截命令（新增） ==========

	if (!strcmp(argv[0], "intercept_radar")) {
		// 雷达引导空对空拦截（从地面起飞）
		if (argc >= 2) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// 雷达目标ID
			get_instance()->_radar_target_id = atoi(argv[1]);
			get_instance()->_auto_track_radar = true;

			// 起飞高度（可选参数）
			if (argc >= 3) {
				get_instance()->_takeoff_altitude = atof(argv[2]);
			} else {
				get_instance()->_takeoff_altitude = 30.0f;  // 默认30米
			}

			// 初始化雷达跟踪
			get_instance()->_target_position.zero();
			get_instance()->_target_velocity.zero();
			get_instance()->_radar_filtered_position.zero();
			get_instance()->_radar_filtered_velocity.zero();
			get_instance()->_radar_loss_count = 0;
			get_instance()->_last_radar_update_time = 0;

			// 初始化速度为小幅上升
			get_instance()->_commanded_velocity(0) = 0.0f;
			get_instance()->_commanded_velocity(1) = 0.0f;
			get_instance()->_commanded_velocity(2) = -0.5f;

			// 进入起飞状态
			get_instance()->_mission_state = MissionState::TAKEOFF;
			get_instance()->_takeoff_start_time = hrt_absolute_time();
			get_instance()->_auto_takeoff = true;

			PX4_INFO("=== Radar-Guided Air Intercept Mission ===");
			PX4_INFO("Radar target ID: %d", get_instance()->_radar_target_id);
			PX4_INFO("Takeoff altitude: %.1f m", (double)get_instance()->_takeoff_altitude);
			PX4_INFO("Guidance: True Proportional Navigation (TPN)");
			PX4_INFO("Navigation gain: %.1f", (double)get_instance()->_param_pn_nav_gain.get());
			PX4_INFO("");
			PX4_INFO("Waiting for radar lock...");
			PX4_INFO("Wait 2 seconds, then: commander arm && commander mode offboard");

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation intercept_radar <target_id> [takeoff_altitude]");
			PX4_INFO("Example: intercept_radar 1 30");
			PX4_INFO("");
			PX4_INFO("This command will:");
			PX4_INFO("  1. Take off to specified altitude");
			PX4_INFO("  2. Lock onto radar target with given ID");
			PX4_INFO("  3. Use TPN to intercept the moving target");
			PX4_INFO("  4. Continuously track target until intercept");
			return 1;
		}
	}

	if (!strcmp(argv[0], "intercept_radar_airborne")) {
		// 雷达引导空对空拦截（已在空中）
		if (argc >= 2) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// 检查是否在空中
			if (!get_instance()->isAirborne()) {
				PX4_ERR("Not airborne! Use 'intercept_radar' for ground start");
				return 1;
			}

			// 雷达目标ID
			get_instance()->_radar_target_id = atoi(argv[1]);
			get_instance()->_auto_track_radar = true;

			// 初始化雷达跟踪
			get_instance()->_target_position.zero();
			get_instance()->_target_velocity.zero();
			get_instance()->_radar_filtered_position.zero();
			get_instance()->_radar_filtered_velocity.zero();
			get_instance()->_radar_loss_count = 0;
			get_instance()->_last_radar_update_time = 0;

			// 等待第一次雷达数据
			PX4_INFO("Waiting for radar lock on target ID %d...", get_instance()->_radar_target_id);

			// 短暂等待以获取雷达数据
			px4_usleep(500000);  // 等待0.5秒

			// 检查是否获取到雷达数据
			radar_target_s radar_target;
			bool radar_locked = false;

			if (get_instance()->_radar_target_sub.copy(&radar_target)) {
				if (radar_target.valid && radar_target.target_id == get_instance()->_radar_target_id) {
					get_instance()->_target_position(0) = radar_target.x;
					get_instance()->_target_position(1) = radar_target.y;
					get_instance()->_target_position(2) = radar_target.z;
					get_instance()->_target_velocity(0) = radar_target.vx;
					get_instance()->_target_velocity(1) = radar_target.vy;
					get_instance()->_target_velocity(2) = radar_target.vz;

					get_instance()->_radar_filtered_position = get_instance()->_target_position;
					get_instance()->_radar_filtered_velocity = get_instance()->_target_velocity;
					get_instance()->_last_radar_update_time = hrt_absolute_time();

					radar_locked = true;

					PX4_INFO("Radar lock acquired!");
					PX4_INFO("Target position: (%.1f, %.1f, %.1f) m",
					         (double)radar_target.x, (double)radar_target.y, (double)radar_target.z);
					PX4_INFO("Target velocity: (%.1f, %.1f, %.1f) m/s",
					         (double)radar_target.vx, (double)radar_target.vy, (double)radar_target.vz);
				}
			}

			if (!radar_locked) {
				PX4_WARN("No radar data received yet, will continue tracking when available");
			}

			// 初始化任务状态
			get_instance()->initializeEngagement();
			get_instance()->_commanded_velocity = get_instance()->_current_velocity;

			// 开始拦截任务
			get_instance()->_mission_state = MissionState::ENGAGING;
			get_instance()->_mission_start_time = hrt_absolute_time();

			PX4_INFO("=== Air Intercept Mission Started ===");
			PX4_INFO("Radar target ID: %d", get_instance()->_radar_target_id);
			PX4_INFO("Current altitude: %.1f m", (double)(-get_instance()->_current_position(2)));
			PX4_INFO("Guidance: True Proportional Navigation (TPN)");

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation intercept_radar_airborne <target_id>");
			PX4_INFO("Example: intercept_radar_airborne 1");
			return 1;
		}
	}

	// ========== 地面俯冲打击命令 ==========

	if (!strcmp(argv[0], "ground_strike_gps")) {
		// 地面俯冲打击（GPS坐标）
		if (argc >= 3) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// GPS目标位置（地面固定点）
			double target_lat = atof(argv[1]);
			double target_lon = atof(argv[2]);
			float target_alt = 0.0f;  // 地面高度（AMSL）

			if (argc >= 4) {
				target_alt = atof(argv[3]);  // 可选：指定目标高度
			}

			get_instance()->_target_lat = target_lat;
			get_instance()->_target_lon = target_lon;
			get_instance()->_target_alt = target_alt;
			get_instance()->_use_gps_target = true;

			// 地面目标速度为0
			get_instance()->_target_velocity.zero();

			// 起飞高度（可选参数）
			if (argc >= 5) {
				get_instance()->_takeoff_altitude = atof(argv[4]);
			} else {
				get_instance()->_takeoff_altitude = 50.0f;  // 默认50米（俯冲需要更高）
			}

			// 地面俯冲打击：自动解除速度限制，实现最大速度撞击
			get_instance()->_param_pn_no_vel_limit.set(1);
			PX4_INFO("Speed limit disabled for maximum impact velocity");

			// 检查地图投影
			if (!get_instance()->_map_projection_initialized) {
				PX4_ERR("Map projection not initialized! Wait for GPS lock");
				return 1;
			}

			// 初始化速度为小幅上升
			get_instance()->_commanded_velocity(0) = 0.0f;
			get_instance()->_commanded_velocity(1) = 0.0f;
			get_instance()->_commanded_velocity(2) = -0.5f;

			// 进入起飞状态
			get_instance()->_mission_state = MissionState::TAKEOFF;
			get_instance()->_takeoff_start_time = hrt_absolute_time();
			get_instance()->_auto_takeoff = true;

			PX4_INFO("=== Ground Strike Mission ===");
			PX4_INFO("Target: lat=%.7f, lon=%.7f, alt=%.1f m (AMSL)",
			         target_lat, target_lon, (double)target_alt);
			PX4_INFO("Takeoff altitude: %.1f m", (double)get_instance()->_takeoff_altitude);
			PX4_INFO("Impact angle: %.1f° (vertical)", (double)get_instance()->_param_pn_impact_angle_v.get());
			PX4_INFO("Pullup distance: %.1f m", (double)get_instance()->_param_pn_pullup_dist.get());
			PX4_INFO("");
			PX4_INFO("Wait 2 seconds, then: commander arm && commander mode offboard");

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation ground_strike_gps <lat> <lon> [alt_amsl] [takeoff_altitude]");
			PX4_INFO("Example: ground_strike_gps 47.397742 8.545594 0 50");
			return 1;
		}
	}

	if (!strcmp(argv[0], "ground_strike")) {
		// 地面俯冲打击（NED坐标）
		if (argc >= 3) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// 目标位置（NED坐标）
			float target_x = atof(argv[1]);
			float target_y = atof(argv[2]);
			float target_z = 0.0f;  // 地面（NED坐标，0 = home高度）

			if (argc >= 4) {
				target_z = atof(argv[3]);  // 可选：指定目标高度
			}

			get_instance()->_target_position(0) = target_x;
			get_instance()->_target_position(1) = target_y;
			get_instance()->_target_position(2) = target_z;

			// 地面目标速度为0
			get_instance()->_target_velocity.zero();

			// 起飞高度（可选参数）
			if (argc >= 5) {
				get_instance()->_takeoff_altitude = atof(argv[4]);
			} else {
				get_instance()->_takeoff_altitude = 50.0f;
			}

			// 地面俯冲打击：自动解除速度限制，实现最大速度撞击
			get_instance()->_param_pn_no_vel_limit.set(1);
			PX4_INFO("Speed limit disabled for maximum impact velocity");

			// 检查是否在空中
			if (!get_instance()->isAirborne()) {
				// 从地面起飞
				get_instance()->_commanded_velocity(0) = 0.0f;
				get_instance()->_commanded_velocity(1) = 0.0f;
				get_instance()->_commanded_velocity(2) = -0.5f;

				get_instance()->sendTakeoffCommand();
				get_instance()->_mission_state = MissionState::TAKEOFF;
				get_instance()->_takeoff_start_time = hrt_absolute_time();
				get_instance()->_auto_takeoff = true;

				PX4_INFO("Ground strike: Taking off to %.1f m", (double)get_instance()->_takeoff_altitude);
			} else {
				// 已在空中，直接开始打击
				get_instance()->initializeEngagement();
				get_instance()->_commanded_velocity = get_instance()->_current_velocity;
				get_instance()->_mission_state = MissionState::GROUND_STRIKE;
				get_instance()->_mission_start_time = hrt_absolute_time();

				PX4_INFO("Ground strike: Engaging from current altitude");
			}

			PX4_INFO("=== Ground Strike Mission ===");
			PX4_INFO("Target NED: (%.1f, %.1f, %.1f) m",
			         (double)target_x, (double)target_y, (double)target_z);
			PX4_INFO("Impact angle: %.1f° (vertical)", (double)get_instance()->_param_pn_impact_angle_v.get());

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation ground_strike <x> <y> [z] [takeoff_altitude]");
			PX4_INFO("Example: ground_strike 100 0 0 50");
			return 1;
		}
	}

	if (!strcmp(argv[0], "engage_gps")) {
		if (argc >= 4) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// GPS目标位置（度）
			double target_lat = atof(argv[1]);
			double target_lon = atof(argv[2]);
			float target_alt = atof(argv[3]);  // AMSL高度（米）

			get_instance()->_target_lat = target_lat;
			get_instance()->_target_lon = target_lon;
			get_instance()->_target_alt = target_alt;
			get_instance()->_use_gps_target = true;

			// 目标速度为0（地面固定点）
			get_instance()->_target_velocity.zero();

			// 检查地图投影是否初始化
			if (!get_instance()->_map_projection_initialized) {
				PX4_ERR("Map projection not initialized! Wait for GPS lock");
				return 1;
			}

			// 检查是否在空中
			if (!get_instance()->isAirborne()) {
				PX4_WARN("Not airborne! Use 'engage_gps_takeoff' for ground start");
				return 1;
			}

			// 转换GPS到NED坐标
			float target_x, target_y;
			get_instance()->_map_projection.project(target_lat, target_lon, target_x, target_y);

			// 计算目标高度（NED坐标，负值=向上）
			vehicle_global_position_s global_pos;
			if (get_instance()->_vehicle_global_position_sub.copy(&global_pos)) {
				float current_amsl = global_pos.alt;
				float target_z = -((target_alt - current_amsl) + (-get_instance()->_current_position(2)));

				get_instance()->_target_position(0) = target_x;
				get_instance()->_target_position(1) = target_y;
				get_instance()->_target_position(2) = target_z;
			}

			// 初始化任务状态
			get_instance()->initializeEngagement();
			get_instance()->_commanded_velocity = get_instance()->_current_velocity;

			// 开始任务
			get_instance()->_mission_state = MissionState::ENGAGING;
			get_instance()->_mission_start_time = hrt_absolute_time();

			PX4_INFO("Engaging GPS target: lat=%.7f, lon=%.7f, alt=%.1f m (AMSL)",
			         target_lat, target_lon, (double)target_alt);
			PX4_INFO("NED target: x=%.1f, y=%.1f, z=%.1f m",
			         (double)target_x, (double)target_y, (double)get_instance()->_target_position(2));

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation engage_gps <lat> <lon> <alt_amsl>");
			return 1;
		}
	}

	if (!strcmp(argv[0], "engage_gps_takeoff")) {
		if (argc >= 4) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// GPS目标位置
			double target_lat = atof(argv[1]);
			double target_lon = atof(argv[2]);
			float target_alt = atof(argv[3]);

			get_instance()->_target_lat = target_lat;
			get_instance()->_target_lon = target_lon;
			get_instance()->_target_alt = target_alt;
			get_instance()->_use_gps_target = true;
			get_instance()->_target_velocity.zero();

			// 起飞高度（可选参数）
			if (argc >= 5) {
				get_instance()->_takeoff_altitude = atof(argv[4]);
			} else {
				get_instance()->_takeoff_altitude = 30.0f;  // 默认30米（高速俯冲需要更高）
			}

			// 检查地图投影
			if (!get_instance()->_map_projection_initialized) {
				PX4_ERR("Map projection not initialized! Wait for GPS lock");
				return 1;
			}

			// 初始化速度为小幅上升（Offboard 需要非零速度）
			get_instance()->_commanded_velocity(0) = 0.0f;
			get_instance()->_commanded_velocity(1) = 0.0f;
			get_instance()->_commanded_velocity(2) = -0.5f;  // 0.5m/s 上升

			// 进入起飞状态（这会开始发布 setpoint）
			get_instance()->_mission_state = MissionState::TAKEOFF;
			get_instance()->_takeoff_start_time = hrt_absolute_time();
			get_instance()->_auto_takeoff = true;

			PX4_INFO("Offboard setpoint stream started");
			PX4_INFO("Wait 2 seconds, then: commander arm && commander mode offboard");
			PX4_INFO("Target: lat=%.7f, lon=%.7f, alt=%.1f m, takeoff=%.1f m",
			         target_lat, target_lon, (double)target_alt, (double)get_instance()->_takeoff_altitude);

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation engage_gps_takeoff <lat> <lon> <alt_amsl> [takeoff_altitude]");
			return 1;
		}
	}

	if (!strcmp(argv[0], "engage")) {
		if (argc >= 4) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// 目标位置
			float target_x = atof(argv[1]);
			float target_y = atof(argv[2]);
			float target_z = atof(argv[3]);

			get_instance()->_target_position(0) = target_x;
			get_instance()->_target_position(1) = target_y;
			get_instance()->_target_position(2) = target_z;

			// 目标速度（可选参数，支持运动目标）
			if (argc >= 7) {
				float target_vx = atof(argv[4]);
				float target_vy = atof(argv[5]);
				float target_vz = atof(argv[6]);

				get_instance()->_target_velocity(0) = target_vx;
				get_instance()->_target_velocity(1) = target_vy;
				get_instance()->_target_velocity(2) = target_vz;

				PX4_INFO("Target velocity: (%.1f, %.1f, %.1f) m/s",
				         (double)target_vx, (double)target_vy, (double)target_vz);
			} else {
				// 静止目标
				get_instance()->_target_velocity.zero();
			}

			// 检查是否在空中
			if (!get_instance()->isAirborne()) {
				PX4_WARN("Not airborne! Use 'engage_takeoff' for ground start");
				return 1;
			}

			// 初始化任务状态（关键步骤！）
			get_instance()->initializeEngagement();

			// 初始化速度
			get_instance()->_commanded_velocity = get_instance()->_current_velocity;

			// 开始任务
			get_instance()->_mission_state = MissionState::ENGAGING;
			get_instance()->_mission_start_time = hrt_absolute_time();

			PX4_INFO("Engaging target at (%.1f, %.1f, %.1f)",
			         (double)target_x, (double)target_y, (double)target_z);

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation engage <x> <y> <z> [vx] [vy] [vz]");
			return 1;
		}
	}

	if (!strcmp(argv[0], "engage_takeoff")) {
		if (argc >= 4) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// 目标位置
			float target_x = atof(argv[1]);
			float target_y = atof(argv[2]);
			float target_z = atof(argv[3]);

			get_instance()->_target_position(0) = target_x;
			get_instance()->_target_position(1) = target_y;
			get_instance()->_target_position(2) = target_z;

			// 目标速度（可选参数）
			if (argc >= 7) {
				float target_vx = atof(argv[4]);
				float target_vy = atof(argv[5]);
				float target_vz = atof(argv[6]);

				get_instance()->_target_velocity(0) = target_vx;
				get_instance()->_target_velocity(1) = target_vy;
				get_instance()->_target_velocity(2) = target_vz;
			} else {
				get_instance()->_target_velocity.zero();
			}

			// 起飞高度（可选参数）
			if (argc >= 8) {
				get_instance()->_takeoff_altitude = atof(argv[7]);
			} else {
				get_instance()->_takeoff_altitude = 20.0f;  // 默认 20 米
			}

			// 发送起飞指令
			get_instance()->sendTakeoffCommand();

			// 进入起飞状态
			get_instance()->_mission_state = MissionState::TAKEOFF;
			get_instance()->_takeoff_start_time = hrt_absolute_time();
			get_instance()->_auto_takeoff = true;

			PX4_INFO("Takeoff and engage target at (%.1f, %.1f, %.1f), altitude: %.1f m",
			         (double)target_x, (double)target_y, (double)target_z,
			         (double)get_instance()->_takeoff_altitude);

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation engage_takeoff <x> <y> <z> [vx] [vy] [vz] [altitude]");
			return 1;
		}
	}

	if (!strcmp(argv[0], "engage_polar")) {
		// 极坐标打击：指定目标的初始位置和运动参数
		if (argc >= 5) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			// 参数：距离(m) 方位角(度) 高度(m) 速度(m/s) 速度方向角(度) [起飞高度(m)]
			float range = atof(argv[1]);           // 目标初始距离（米）
			float azimuth_deg = atof(argv[2]);     // 目标方位角（度，0=北，90=东）
			float altitude = atof(argv[3]);        // 目标飞行高度（米，正值）
			float target_speed = atof(argv[4]);    // 目标速度（m/s）
			float velocity_azimuth_deg = atof(argv[5]); // 目标飞行方向（度）

			// 起飞高度（可选，默认 20 米）
			float takeoff_alt = 20.0f;
			if (argc >= 7) {
				takeoff_alt = atof(argv[6]);
			}

			// 转换为弧度
			float azimuth_rad = math::radians(azimuth_deg);
			float velocity_azimuth_rad = math::radians(velocity_azimuth_deg);

			// 计算目标初始位置（NED坐标系）
			float target_x = range * cosf(azimuth_rad);  // 北向
			float target_y = range * sinf(azimuth_rad);  // 东向
			float target_z = -altitude;                   // 下向（负值=高度）

			get_instance()->_target_position(0) = target_x;
			get_instance()->_target_position(1) = target_y;
			get_instance()->_target_position(2) = target_z;

			// 计算目标速度向量
			float target_vx = target_speed * cosf(velocity_azimuth_rad);  // 北向速度
			float target_vy = target_speed * sinf(velocity_azimuth_rad);  // 东向速度
			float target_vz = 0.0f;  // 水平飞行

			get_instance()->_target_velocity(0) = target_vx;
			get_instance()->_target_velocity(1) = target_vy;
			get_instance()->_target_velocity(2) = target_vz;

			get_instance()->_takeoff_altitude = takeoff_alt;

			// 发送起飞指令
			get_instance()->sendTakeoffCommand();

			// 进入起飞状态
			get_instance()->_mission_state = MissionState::TAKEOFF;
			get_instance()->_takeoff_start_time = hrt_absolute_time();
			get_instance()->_auto_takeoff = true;

			PX4_INFO("=== Polar Engagement ===");
			PX4_INFO("Target initial position: %.1fm away, %.1f° azimuth, %.1fm altitude",
			         (double)range, (double)azimuth_deg, (double)altitude);
			PX4_INFO("Target NED position: (%.1f, %.1f, %.1f)",
			         (double)target_x, (double)target_y, (double)target_z);
			PX4_INFO("Target moving: %.1f m/s toward %.1f°",
			         (double)target_speed, (double)velocity_azimuth_deg);
			PX4_INFO("Target velocity: (%.1f, %.1f, %.1f) m/s",
			         (double)target_vx, (double)target_vy, (double)target_vz);
			PX4_INFO("Takeoff altitude: %.1f m", (double)takeoff_alt);

			// 计算预测拦截点
			float time_to_intercept = range / 15.0f;  // 粗略估计（假设攻击机速度15m/s）
			float predicted_x = target_x + target_vx * time_to_intercept;
			float predicted_y = target_y + target_vy * time_to_intercept;
			PX4_INFO("Predicted intercept point: (%.1f, %.1f) after %.1fs",
			         (double)predicted_x, (double)predicted_y, (double)time_to_intercept);

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation engage_polar <range> <azimuth> <altitude> <speed> <vel_azimuth> [takeoff_alt]");
			PX4_INFO("Example: engage_polar 100 0 30 10 0");
			PX4_INFO("  Target: 100m north, 30m altitude, moving 10m/s toward north");
			PX4_INFO("");
			PX4_INFO("Parameters:");
			PX4_INFO("  range: Initial distance to target (meters)");
			PX4_INFO("  azimuth: Target bearing (degrees, 0=North, 90=East, 180=South, 270=West)");
			PX4_INFO("  altitude: Target flight altitude (meters, positive)");
			PX4_INFO("  speed: Target speed (m/s)");
			PX4_INFO("  vel_azimuth: Target heading (degrees, direction it's flying toward)");
			PX4_INFO("  takeoff_alt: Takeoff altitude (meters, optional, default=20)");
			return 1;
		}
	}

	if (!strcmp(argv[0], "abort")) {
		get_instance()->emergencyAbort();
		PX4_INFO("Mission aborted");
		return 0;
	}

	if (!strcmp(argv[0], "track_radar")) {
		if (argc >= 2) {
			if (get_instance()->_mission_state != MissionState::IDLE) {
				PX4_WARN("Mission already active");
				return 1;
			}

			get_instance()->_radar_target_id = atoi(argv[1]);
			get_instance()->_auto_track_radar = true;

			get_instance()->_target_position.zero();
			get_instance()->_target_velocity.zero();

			get_instance()->initializeEngagement();
			get_instance()->_commanded_velocity = get_instance()->_current_velocity;
			get_instance()->_mission_state = MissionState::ENGAGING;
			get_instance()->_mission_start_time = hrt_absolute_time();

			PX4_INFO("Tracking radar target ID: %d", get_instance()->_radar_target_id);
			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation track_radar <target_id>");
			return 1;
		}
	}

	if (!strcmp(argv[0], "abort")) {
		get_instance()->emergencyAbort();
		PX4_INFO("Mission aborted");
		return 0;
	}

	if (!strcmp(argv[0], "update_target")) {
		// 实时更新目标位置和速度（用于持续跟踪）
		if (argc >= 4) {
			float target_x = atof(argv[1]);
			float target_y = atof(argv[2]);
			float target_z = atof(argv[3]);

			get_instance()->_target_position(0) = target_x;
			get_instance()->_target_position(1) = target_y;
			get_instance()->_target_position(2) = target_z;

			if (argc >= 7) {
				float target_vx = atof(argv[4]);
				float target_vy = atof(argv[5]);
				float target_vz = atof(argv[6]);

				get_instance()->_target_velocity(0) = target_vx;
				get_instance()->_target_velocity(1) = target_vy;
				get_instance()->_target_velocity(2) = target_vz;
			}

			PX4_INFO("Target updated: pos(%.1f, %.1f, %.1f) vel(%.1f, %.1f, %.1f)",
			         (double)target_x, (double)target_y, (double)target_z,
			         (double)get_instance()->_target_velocity(0),
			         (double)get_instance()->_target_velocity(1),
			         (double)get_instance()->_target_velocity(2));

			return 0;
		} else {
			PX4_ERR("Usage: proportional_navigation update_target <x> <y> <z> [vx] [vy] [vz]");
			return 1;
		}
	}

	if (!strcmp(argv[0], "status")) {
		PX4_INFO("=== PPNIACG Status ===");
		PX4_INFO("State: %d (%s)", (int)get_instance()->_mission_state,
		         get_instance()->_mission_state == MissionState::IDLE ? "IDLE" :
		         get_instance()->_mission_state == MissionState::TAKEOFF ? "TAKEOFF" :
		         get_instance()->_mission_state == MissionState::ENGAGING ? "ENGAGING" :
		         get_instance()->_mission_state == MissionState::GROUND_STRIKE ? "GROUND_STRIKE" :
		         get_instance()->_mission_state == MissionState::PULLUP ? "PULLUP" : "COMPLETE");
		PX4_INFO("Range: %.2f m", (double)get_instance()->_r);

		if (get_instance()->_use_gps_target) {
			PX4_INFO("GPS target: lat=%.7f, lon=%.7f, alt=%.1f m (AMSL)",
			         get_instance()->_target_lat, get_instance()->_target_lon,
			         (double)get_instance()->_target_alt);
		}

		PX4_INFO("Target pos: (%.1f, %.1f, %.1f)",
		         (double)get_instance()->_target_position(0),
		         (double)get_instance()->_target_position(1),
		         (double)get_instance()->_target_position(2));
		PX4_INFO("Target vel: (%.1f, %.1f, %.1f) m/s",
		         (double)get_instance()->_target_velocity(0),
		         (double)get_instance()->_target_velocity(1),
		         (double)get_instance()->_target_velocity(2));
		PX4_INFO("Altitude: %.2f m", (double)(-get_instance()->_current_position(2)));
		PX4_INFO("LOS angle: %.1f°", (double)math::degrees(get_instance()->_q));
		PX4_INFO("Lead angle: %.1f°", (double)math::degrees(get_instance()->_theta));
		PX4_INFO("LOS rate: %.4f rad/s", (double)get_instance()->_q_dot);
		PX4_INFO("Final LOS: %.1f°", (double)math::degrees(get_instance()->_q_f));
		PX4_INFO("Current N: %.2f", (double)get_instance()->_current_N);
		PX4_INFO("PPN mode: %s", get_instance()->_switched_to_ppn ? "YES" : "NO");
		PX4_INFO("Pullup enabled: %s", get_instance()->_param_pn_enable_pullup.get() ? "YES" : "NO");
		PX4_INFO("Velocity: %.2f m/s", (double)get_instance()->_current_velocity.norm());
		PX4_INFO("Accel cmd: %.2f m/s²", (double)get_instance()->_commanded_acceleration.norm());

		// 雷达状态
		if (get_instance()->_auto_track_radar) {
			PX4_INFO("Radar tracking: target_id=%d, loss_count=%d",
			         get_instance()->_radar_target_id,
			         get_instance()->_radar_loss_count);
		}

		return 0;
	}

	return print_usage("unknown command");
}

int ProportionalNavigation::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Pure Proportional Navigation with Impact Angle Constraint Guidance (PPNIACG)

Implements analytical solution for PPN intercepting stationary or moving targets with impact angle constraints.

Key features:
- Supports moving targets (TPN - True Proportional Navigation)
- Uses analytical solutions (formulas 11, 18, 28, 32)
- No time-to-go estimation required
- Adaptive navigation gain
- Vertical and horizontal impact angle constraints
- Robust against disturbances

Based on missile guidance theory.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("proportional_navigation", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("intercept_radar", "Radar-guided air intercept from ground");
	PRINT_MODULE_USAGE_ARG("<target_id> [takeoff_altitude]", "Radar target ID and takeoff altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("intercept_radar_airborne", "Radar-guided air intercept (already airborne)");
	PRINT_MODULE_USAGE_ARG("<target_id>", "Radar target ID to track", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ground_strike_gps", "Ground dive strike using GPS coordinates");
	PRINT_MODULE_USAGE_ARG("<lat> <lon> [alt_amsl] [takeoff_alt]", "Target GPS and takeoff altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ground_strike", "Ground dive strike using NED coordinates");
	PRINT_MODULE_USAGE_ARG("<x> <y> [z] [takeoff_altitude]", "Target NED position and takeoff altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("engage_polar", "Takeoff and engage target using polar coordinates");
	PRINT_MODULE_USAGE_ARG("<range> <azimuth> <altitude> <speed> [vel_azimuth] [takeoff_alt]", "Range(m), direction(deg), altitude(m), speed(m/s)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("engage_gps", "Engage GPS target (requires airborne)");
	PRINT_MODULE_USAGE_ARG("<lat> <lon> <alt_amsl>", "Target GPS position (degrees, AMSL meters)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("engage_gps_takeoff", "Takeoff and engage GPS target from ground");
	PRINT_MODULE_USAGE_ARG("<lat> <lon> <alt_amsl> [takeoff_altitude]", "Target GPS and takeoff altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("engage", "Engage target (requires airborne)");
	PRINT_MODULE_USAGE_ARG("<x> <y> <z> [vx] [vy] [vz]", "Target position and velocity in NED (meters, m/s)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("engage_takeoff", "Takeoff and engage target from ground");
	PRINT_MODULE_USAGE_ARG("<x> <y> <z> [vx] [vy] [vz] [altitude]", "Target position, velocity, and takeoff altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("track_radar", "Track radar target automatically");
	PRINT_MODULE_USAGE_ARG("<target_id>", "Target ID to track", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("update_target", "Update target position and velocity during tracking");
	PRINT_MODULE_USAGE_ARG("<x> <y> <z> [vx] [vy] [vz]", "Target position and velocity in NED (meters, m/s)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("abort", "Abort mission");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show detailed status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int proportional_navigation_main(int argc, char *argv[])
{
	return ProportionalNavigation::main(argc, argv);
}


void ProportionalNavigation::executePullup()
{
	// 拉升机动：仰头上升到安全高度

	// 计算到目标高度的距离
	float altitude_error = _pullup_target_altitude - _current_position(2);  // NED坐标，负值=向上

	// 拉升加速度（向上）
	float pullup_accel = _param_pn_pullup_accel.get();

	// 计算垂直加速度指令
	matrix::Vector3f accel_cmd;
	accel_cmd.zero();

	if (altitude_error < 0.0f) {
		// 需要上升
		accel_cmd(2) = -pullup_accel;  // NED坐标，负值=向上加速

		// 保持水平速度，逐渐减小
		float horizontal_decel = 5.0f;  // 水平减速度
		matrix::Vector2f horizontal_vel(_current_velocity(0), _current_velocity(1));
		float h_speed = horizontal_vel.norm();

		if (h_speed > 1.0f) {
			matrix::Vector2f h_decel_dir = -horizontal_vel.normalized();
			accel_cmd(0) = h_decel_dir(0) * horizontal_decel;
			accel_cmd(1) = h_decel_dir(1) * horizontal_decel;
		}

	} else {
		// 已到达目标高度，悬停
		accel_cmd.zero();
		_commanded_velocity.zero();
	}

	// 限制加速度
	float max_accel = _param_pn_max_accel.get();
	if (accel_cmd.norm() > max_accel) {
		accel_cmd = accel_cmd.normalized() * max_accel;
	}

	_commanded_acceleration = accel_cmd;

	// 积分得到速度
	_commanded_velocity += _commanded_acceleration * _dt;

	// 限制速度（拉升阶段保持限制以确保安全）
	float max_vel = _param_pn_max_vel.get();
	if (_commanded_velocity.norm() > max_vel) {
		_commanded_velocity = _commanded_velocity.normalized() * max_vel;
	}

	// 限制垂直速度（安全考虑）
	float max_vertical_vel = 15.0f;  // 最大上升速度 15 m/s（提高以适应高速拉升）
	if (fabsf(_commanded_velocity(2)) > max_vertical_vel) {
		_commanded_velocity(2) = math::constrain(_commanded_velocity(2), -max_vertical_vel, max_vertical_vel);
	}
}

void ProportionalNavigation::executeTakeoff()
{
	// 起飞机动：只垂直上升，不水平移动

	float current_altitude = -_current_position(2);  // NED坐标，负值=高度
	float altitude_error = _takeoff_altitude - current_altitude;

	// 起飞垂直速度：5 m/s（提高速度）
	float takeoff_vertical_speed = 5.0f;

	// 计算速度指令 - 只垂直上升
	_commanded_velocity(0) = 0.0f;
	_commanded_velocity(1) = 0.0f;

	if (altitude_error > 0.5f) {
		// 只垂直上升
		_commanded_velocity(2) = -takeoff_vertical_speed;  // NED坐标，负值=向上
	} else {
		// 到达高度，停止垂直运动
		_commanded_velocity(2) = 0.0f;
	}
}

// ============================================================
// 雷达数据处理功能（新增）
// ============================================================

void ProportionalNavigation::updateRadarTarget()
{
	radar_target_s radar_target;

	if (_radar_target_sub.update(&radar_target)) {
		if (radar_target.valid && radar_target.target_id == _radar_target_id) {
			// 雷达数据有效
			hrt_abstime now = hrt_absolute_time();

			// 原始测量值
			matrix::Vector3f measured_position(
				radar_target.x,
				radar_target.y,
				radar_target.z
			);

			matrix::Vector3f measured_velocity(
				radar_target.vx,
				radar_target.vy,
				radar_target.vz
			);

			// 首次接收雷达数据
			if (_last_radar_update_time == 0) {
				_radar_filtered_position = measured_position;
				_radar_filtered_velocity = measured_velocity;
				_radar_predicted_position = measured_position;
				_radar_predicted_velocity = measured_velocity;

				PX4_INFO("Radar lock acquired: target_id=%d", _radar_target_id);
			} else {
				// α-β滤波器（平滑噪声）
				filterRadarData();

				// 计算时间步长
				float dt = (now - _last_radar_update_time) / 1e6f;
				dt = math::constrain(dt, 0.001f, 0.5f);

				// 预测步骤
				_radar_predicted_position = _radar_filtered_position + _radar_filtered_velocity * dt;

				// 更新步骤（α-β滤波）
				matrix::Vector3f innovation = measured_position - _radar_predicted_position;

				_radar_filtered_position = _radar_predicted_position + innovation * _radar_alpha;
				_radar_filtered_velocity = _radar_filtered_velocity + (innovation / dt) * _radar_beta;
			}

			// 使用滤波后的数据
			_target_position = _radar_filtered_position;
			_target_velocity = _radar_filtered_velocity;

			// 更新时间戳
			_last_radar_update_time = now;
			_radar_loss_count = 0;  // 重置丢失计数

		} else {
			// 雷达数据无效
			handleRadarLoss();
		}
	} else {
		// 没有新的雷达数据
		handleRadarLoss();
	}
}

void ProportionalNavigation::filterRadarData()
{
	// α-β滤波器已在 updateRadarTarget() 中实现
	// 这里可以添加更复杂的滤波算法（如卡尔曼滤波）
}

void ProportionalNavigation::handleRadarLoss()
{
	_radar_loss_count++;

	if (_radar_loss_count < RADAR_LOSS_THRESHOLD) {
		// 短时间丢失：使用预测值
		hrt_abstime now = hrt_absolute_time();
		float dt = (now - _last_radar_update_time) / 1e6f;
		dt = math::constrain(dt, 0.0f, 1.0f);

		// 匀速直线运动预测
		_target_position = _radar_predicted_position + _radar_predicted_velocity * dt;
		_target_velocity = _radar_predicted_velocity;

		if (_radar_loss_count == 1) {
			PX4_WARN("Radar signal lost, using prediction");
		}

	} else if (_radar_loss_count == RADAR_LOSS_THRESHOLD) {
		// 长时间丢失：中止任务
		PX4_ERR("Radar signal lost for too long, aborting mission");
		emergencyAbort();
	}
}

bool ProportionalNavigation::isRadarDataValid()
{
	return (_radar_loss_count < RADAR_LOSS_THRESHOLD);
}

// ============================================================
// 地面俯冲打击功能（新增）
// ============================================================

void ProportionalNavigation::executeGroundStrike()
{
	// 地面俯冲打击：针对地面固定目标的高速俯冲攻击
	// 特点：
	// 1. 目标速度为0（地面固定点）
	// 2. 强制使用铅垂面内落角约束（垂直或大角度俯冲）
	// 3. 高速接近（不限制速度）
	// 4. 必须启用拉升（安全考虑）

	// 计算极坐标系状态
	_r = computeRange();
	_q = computeLOSAngle();
	_theta = computeLeadAngle();
	_q_dot = computeLOSRate();
	_v_r = computeRadialVelocity();
	_v_theta = computeNormalVelocity();

	// 地面打击使用铅垂面内落角约束
	float q_f_desired = math::radians(_param_pn_impact_angle_v.get());

	// 计算PPNIACG加速度（铅垂面内）
	float accel_magnitude = computeVerticalIACG(_r, _q, _theta, q_f_desired);

	// 加速度方向：在铅垂面内，垂直于速度方向
	matrix::Vector2f velocity_2d(
		_current_velocity(0),  // vx (北)
		_current_velocity(2)   // vz (下)
	);

	matrix::Vector3f accel_cmd;
	accel_cmd.zero();

	float v_norm = velocity_2d.norm();
	if (v_norm > 0.1f) {
		// 垂直于速度的方向（铅垂面内）
		matrix::Vector2f accel_dir_2d(
			-velocity_2d(1),  // -vz
			velocity_2d(0)    // vx
		);

		accel_dir_2d = accel_dir_2d.normalized();

		// 转换回3D（y方向为0）
		accel_cmd(0) = accel_dir_2d(0) * accel_magnitude;
		accel_cmd(1) = 0.0f;
		accel_cmd(2) = accel_dir_2d(1) * accel_magnitude;
	}

	// 限制加速度
	float max_accel = _param_pn_max_accel.get();
	if (accel_cmd.norm() > max_accel) {
		accel_cmd = accel_cmd.normalized() * max_accel;
	}

	// 对加速度方向进行滤波，保持大小
	float cmd_accel_mag = accel_cmd.norm();
	if (cmd_accel_mag > 0.1f) {
		matrix::Vector3f accel_direction = accel_cmd.normalized();
		matrix::Vector3f prev_direction = _commanded_acceleration.norm() > 0.1f ?
			_commanded_acceleration.normalized() : accel_direction;

		// 方向滤波
		float dir_alpha = 0.6f;
		matrix::Vector3f filtered_direction = prev_direction * (1.0f - dir_alpha) + accel_direction * dir_alpha;
		filtered_direction = filtered_direction.normalized();

		_commanded_acceleration = filtered_direction * cmd_accel_mag;
	} else {
		_commanded_acceleration = accel_cmd;
	}

	// 快速加速，不限制速度
	float accel_multiplier = 100.0f;
	_commanded_velocity += _commanded_acceleration * _dt * accel_multiplier;

	// 安全检查：高度过低时强制拉升
	float current_altitude = -_current_position(2);
	float safety_altitude = _param_pn_pullup_dist.get() + 5.0f;  // 安全高度

	if (current_altitude < safety_altitude || _r < _param_pn_pullup_dist.get()) {
		PX4_WARN("Ground strike: Safety altitude reached, initiating pullup");
		_mission_state = MissionState::PULLUP;
		_pullup_start_time = hrt_absolute_time();
		_pullup_target_altitude = -(_param_pn_pullup_alt.get());  // NED坐标
	}

	// 打印调试信息
	static hrt_abstime last_print = 0;
	if (hrt_absolute_time() - last_print > 500_ms) {
		PX4_INFO("Ground Strike: r=%.1fm, alt=%.1fm, vel=%.1fm/s, q=%.1f°, q_f=%.1f°",
		         (double)_r,
		         (double)current_altitude,
		         (double)_commanded_velocity.norm(),
		         (double)math::degrees(_q),
		         (double)math::degrees(_q_f));
		last_print = hrt_absolute_time();
	}
}
