#include "swarm_node.hpp"

using namespace matrix;

Swarm_Node::Swarm_Node() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_sp_position(0.f, 0.f, 0.f),
	_sp_vel(0.f, 0.f, 0.f),
	_sp_yaw(0.f),
	_sp_yaw_speed(0.f)
{
}

Swarm_Node::~Swarm_Node()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


void
Swarm_Node::set_swarm_offset(const matrix::Vector3f &sp_offset)
{
	_sp_offset = sp_offset;
}

void
Swarm_Node::set_swarm_info(const int &set_as_leader, const int &group_id)
{
	_set_as_leader = set_as_leader;
	_group_id = group_id;


	if (_set_as_leader) {
		_leader_group_id.group_id = _group_id;
		_leader_group_id.leader = 1;

	} else {
		_leader_group_id.group_id = _group_id;
		_leader_group_id.leader = 0;

	}

	_group_id_pub.publish(_leader_group_id);
}


bool
Swarm_Node::init()
{
	ScheduleOnInterval(20000_us); // 50 Hz rate
	return true;
}


bool Swarm_Node::swarm_node_init()
{
	if (_init_done) {
		return true;
	}

	// 获取MAV_SYS_ID参数
	int32_t mav_sys_id = -1;
	param_t param_id = param_find("MAV_SYS_ID");

	if (param_id != PARAM_INVALID) {
		param_get(param_id, &mav_sys_id);
		vehicle_id = mav_sys_id;

	} else {
		PX4_WARN("无法获取MAV_SYS_ID参数，使用默认vehicle_id=%d", vehicle_id);
	}

	_last_sp_yaw = NAN;

	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.copy(&_vehicle_local_position);
	}

	_swarm_start_flag_sub.copy(&_swarm_start_flag);

	// 设置组号、偏移量等
	set_swarm_info(_param_swarm_set_leader.get(), _param_swarm_group_id.get());
	set_swarm_offset(Vector3f(_param_swarm_X_offset.get(), _param_swarm_Y_offset.get(), _param_swarm_Z_offset.get()));

	bool swarm_active = (_swarm_start_flag.start_swarm || _swarm_start_flag.start_swarm_auto);

	if ((_vehicle_local_position.xy_valid) && swarm_active) {
		// 初始化参考坐标
		if (!_map_ref_initialized) {
			_global_local_proj_ref.initReference(_vehicle_local_position.ref_lat,
							    _vehicle_local_position.ref_lon,
							    hrt_absolute_time());
			_map_ref_initialized = true;
		}

		_init_done = true;
		return true;
	}

	return false;
}



/**
 * @brief 更新主机信息
 * @param selected_leader_id 选定的主机ID（0表示自动选择）
 * @return true: 主机信息有效，可以继续跟随; false: 主机信息无效
 */
bool Swarm_Node::update_leader_info(int32_t selected_leader_id)
{
	if (!_leader_info_sub.updated()) {
		// 没有新数据，检查现有数据是否有效
		return PX4_ISFINITE(_leader_sp_glo_pos.lat) && PX4_ISFINITE(_leader_sp_glo_pos.lon);
	}

	leader_info_s temp_info{};
	_leader_info_sub.copy(&temp_info);

	if (selected_leader_id == 0) {
		// 自动选择模式：接受第一个有效的主机
		if (!PX4_ISFINITE(_leader_sp_glo_pos.lat) || _leader_sp_glo_pos.mavid == 0) {
			_leader_sp_glo_pos = temp_info;
		}

	} else {
		// 指定主机模式
		if (temp_info.mavid == (uint32_t)selected_leader_id) {
			// 检测主机切换
			if (_leader_sp_glo_pos.mavid != 0 && _leader_sp_glo_pos.mavid != (uint32_t)selected_leader_id) {
				PX4_WARN("[主机切换] 从机%d: 从主机ID=%d切换到主机ID=%d",
					 vehicle_id, _leader_sp_glo_pos.mavid, selected_leader_id);
			}

			_leader_sp_glo_pos = temp_info;

		} else {
			// 收到的不是目标主机的信息，清除旧数据
			if (_leader_sp_glo_pos.mavid != 0 && _leader_sp_glo_pos.mavid != (uint32_t)selected_leader_id) {
				_leader_sp_glo_pos = leader_info_s{};
			}
		}
	}

	return PX4_ISFINITE(_leader_sp_glo_pos.lat) && PX4_ISFINITE(_leader_sp_glo_pos.lon);
}

/**
 * @brief 计算目标位置并检测编队切换
 * @param formation_switching [out] 是否发生编队切换（目标位置突变）
 * @return true: 计算成功; false: 计算失败
 */
bool Swarm_Node::calculate_target_position(bool &formation_switching)
{
	formation_switching = false;

	// 将主机GPS位置投影到本地坐标系
	_global_local_proj_ref.project(_leader_sp_glo_pos.lat, _leader_sp_glo_pos.lon, target_x, target_y);

	_final_target(0) = target_x;
	_final_target(1) = target_y;

	// 高度处理：主机发送的是 MSL 高度，需要转换为本地 NED z 坐标
	float leader_ned_z = _vehicle_local_position.ref_alt - static_cast<float>(_leader_sp_glo_pos.alt);
	_final_target(2) = leader_ned_z;

	// 检测编队切换（目标位置突变）
	if (_last_target_valid) {
		matrix::Vector3f target_delta = _final_target - _last_target_position;
		float target_distance = target_delta.norm();

		if (target_distance > _param_formation_switch_threshold.get()) {
			formation_switching = true;
			PX4_WARN("[编队切换检测] 从机%d: 目标位置突变%.2fm", vehicle_id, (double)target_distance);
		}
	}

	return true;
}

/**
 * @brief 处理路径规划和绕行航点
 */
matrix::Vector3f Swarm_Node::handle_path_planning(const matrix::Vector3f &current_position,
						   bool formation_switching,
						   float distance_to_target,
						   const OtherVehiclePosition *other_aircraft)
{
	matrix::Vector3f actual_target = _final_target;

	// 路径规划只在编队切换且距离较远时启用
	bool enable_path_planning = formation_switching && (distance_to_target > PATH_PLANNING_DISTANCE);

	if (enable_path_planning) {
		FormationPlanner::PlannerConfig planner_config;
		planner_config.enable_planning = true;
		planner_config.prediction_time = 2.0f;
		planner_config.collision_threshold = 2.0f;
		planner_config.detour_distance = 3.0f;
		planner_config.max_detection_distance = 7.0f;
		_formation_planner.set_config(planner_config);

		matrix::Vector3f current_velocity(
			_vehicle_local_position.vx,
			_vehicle_local_position.vy,
			_vehicle_local_position.vz
		);

		FormationPlanner::PlanResult plan_result = _formation_planner.plan_formation_path(
			current_position, _final_target, current_velocity,
			other_aircraft, MAX_SWARM_SIZE, vehicle_id
		);

		if (plan_result.needs_detour && !plan_result.direct_path_safe) {
			actual_target = plan_result.waypoint;
			_using_detour_waypoint = true;
			_current_waypoint = plan_result.waypoint;
			PX4_WARN("[路径规划] 从机%d: 启用绕行路径", vehicle_id);
		}

	} else {
		// 正常编队跟随：检查是否需要继续使用绕行航点
		if (_using_detour_waypoint) {
			if (_current_waypoint.norm() < 0.1f) {
				_using_detour_waypoint = false;

			} else {
				matrix::Vector3f to_waypoint = _current_waypoint - current_position;

				if (to_waypoint.norm() < WAYPOINT_REACH_DISTANCE) {
					_using_detour_waypoint = false;
					actual_target = _final_target;

				} else {
					actual_target = _current_waypoint;
				}
			}
		}
	}

	return actual_target;
}

/**
 * @brief 配置速度障碍控制器参数
 * @param current_speed_xy 当前水平速度
 */
void Swarm_Node::configure_vo_controller(float current_speed_xy)
{
	bool is_high_speed_mode = current_speed_xy > _param_high_speed_threshold.get();
	bool is_low_speed_mode = current_speed_xy < _param_low_speed_threshold.get();

	VelocityObstacleController::VOConfig vo_config;
	vo_config.enable_avoidance = true;

	float speed_scale = is_high_speed_mode ? 1.5f : (is_low_speed_mode ? 1.0f : 1.2f);
	vo_config.safety_radius = _param_apf_safety_radius.get() * speed_scale;
	vo_config.danger_radius = _param_apf_danger_radius.get() * speed_scale;
	vo_config.max_avoidance_distance = _param_apf_max_distance.get() * 1.5f;
	vo_config.repulsive_gain = _param_apf_repulsive_gain.get() * speed_scale;
	vo_config.tangential_gain = _param_apf_tangential_gain.get() * speed_scale;
	vo_config.max_avoidance_force = _param_apf_max_force.get() * (is_high_speed_mode ? 2.0f : 1.5f);
	vo_config.max_safe_velocity = is_high_speed_mode ? (8.0f + current_speed_xy) : 5.0f;
	vo_config.velocity_blend_factor = is_high_speed_mode ? 0.9f : 0.7f;
	vo_config.enable_leader_avoidance = true;
	vo_config.clockwise_tangential = true;

	_vo_controller.set_config(vo_config);
}

/**
 * @brief 预测性轨迹碰撞检测
 * @param current_position 当前位置
 * @param current_speed_xy 当前水平速度
 * @param other_aircraft 其他飞机位置信息
 * @param result [out] 碰撞检测结果
 */
void Swarm_Node::predict_trajectory_collision(const matrix::Vector3f &current_position,
					      float current_speed_xy,
					      const OtherVehiclePosition *other_aircraft,
					      CollisionAvoidanceResult &result)
{
	bool is_low_speed_mode = current_speed_xy < _param_low_speed_threshold.get();
	uint64_t current_time_check = hrt_absolute_time();

	for (int i = 0; i < MAX_SWARM_SIZE; i++) {
		if (!other_aircraft[i].valid || other_aircraft[i].mavid == vehicle_id) {
			continue;
		}

		if ((current_time_check - other_aircraft[i].timestamp) >= DATA_VALID_TIMEOUT) {
			continue;
		}

		matrix::Vector3f my_pos(current_position(0), current_position(1), 0.0f);
		matrix::Vector3f my_vel(_vehicle_local_position.vx, _vehicle_local_position.vy, 0.0f);
		matrix::Vector3f other_pos(other_aircraft[i].x, other_aircraft[i].y, 0.0f);
		matrix::Vector3f other_vel(other_aircraft[i].vx, other_aircraft[i].vy, 0.0f);

		bool is_leader_aircraft = other_aircraft[i].is_leader;

		// 使用目标点方向作为预测方向
		matrix::Vector3f my_target_dir = my_vel;
		matrix::Vector3f target_direction(_sp_position_filtered(0) - current_position(0),
						  _sp_position_filtered(1) - current_position(1), 0.0f);

		if (target_direction.norm() > 0.1f) {
			my_target_dir = target_direction.normalized() * 2.0f;
		}

		matrix::Vector3f relative_pos = other_pos - my_pos;
		matrix::Vector3f relative_vel = other_vel - my_vel;
		float distance = relative_pos.norm();

		// 轨迹预测
		float min_predicted_distance = distance;
		float critical_time = 0.0f;

		for (int t = 1; t <= PREDICTION_STEPS; t++) {
			float dt_pred = t * (PREDICTION_HORIZON / PREDICTION_STEPS);
			matrix::Vector3f my_future = my_pos + my_target_dir * dt_pred;
			matrix::Vector3f other_future = other_pos + other_vel * dt_pred;
			float predicted_distance = (other_future - my_future).norm();

			if (predicted_distance < min_predicted_distance) {
				min_predicted_distance = predicted_distance;
				critical_time = dt_pred;
			}
		}

		// 计算接近速度
		float closing_speed = (distance > 0.1f) ? -relative_pos.normalized().dot(relative_vel) : 0.0f;

		// 碰撞检测阈值
		float collision_distance_threshold = is_leader_aircraft ? _param_collision_dist_leader.get() : _param_collision_dist_follower.get();
		float collision_dot_threshold = is_leader_aircraft ? -0.1f : -0.2f;
		float close_distance_threshold = is_leader_aircraft ? _param_close_dist_leader.get() : _param_close_dist_follower.get();
		float close_normalized_threshold = is_leader_aircraft ? 0.1f : 0.0f;

		// 对冲检测
		if (distance > 0.1f && distance < collision_distance_threshold) {
			float normalized_dot_product = 0.0f;
			if (relative_vel.norm() > 0.1f) {
				normalized_dot_product = relative_pos.normalized().dot(relative_vel.normalized());
			}

			float normalized_threshold = fmaxf(-1.0f, collision_dot_threshold * 1.5f);
			float closing_speed_threshold = is_low_speed_mode ? 0.3f : 0.1f;

			if (normalized_dot_product < normalized_threshold && closing_speed > closing_speed_threshold) {
				result.critical_collision_risk = true;
			}

			if (distance < close_distance_threshold &&
			    normalized_dot_product < close_normalized_threshold &&
			    closing_speed > closing_speed_threshold) {
				result.critical_collision_risk = true;
			}
		}

		// 轨迹交叉检测
		float critical_distance_threshold = is_leader_aircraft ? _param_critical_dist_leader.get() : _param_critical_dist_follower.get();
		float warning_distance_threshold = is_leader_aircraft ? _param_warning_dist_leader.get() : _param_warning_dist_follower.get();

		if (min_predicted_distance < critical_distance_threshold && critical_time < PREDICTION_HORIZON) {
			result.critical_collision_risk = true;
		} else if (min_predicted_distance < warning_distance_threshold && critical_time < PREDICTION_HORIZON) {
			result.collision_risk = true;
		}
	}
}

/**
 * @brief 执行避撞检测
 */
Swarm_Node::CollisionAvoidanceResult Swarm_Node::perform_collision_avoidance(
	const matrix::Vector3f &current_position,
	float current_speed_xy,
	bool need_avoidance,
	bool in_takeoff_phase,
	const OtherVehiclePosition *other_aircraft)
{
	CollisionAvoidanceResult result;

	// 只有在需要避撞时才启用避撞检查
	bool enable_avoidance_check = _param_apf_enable.get() > 0.5f && !in_takeoff_phase && need_avoidance;

	if (!enable_avoidance_check) {
		return result;
	}

	// 配置速度障碍控制器
	configure_vo_controller(current_speed_xy);

	// 获取当前速度和期望速度
	matrix::Vector3f current_vel(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
	matrix::Vector3f desired_vel = _sp_vel_filtered;

	// 计算时间步长
	static hrt_abstime last_time = 0;
	hrt_abstime current_time = hrt_absolute_time();
	float dt = (last_time > 0) ? (current_time - last_time) * 1e-6f : 0.1f;
	dt = math::constrain(dt, 0.01f, 0.2f);
	last_time = current_time;

	// 使用速度障碍方法计算安全目标速度和位置修正
	VelocityObstacleController::AvoidanceResult vo_result =
		_vo_controller.calculate_safe_velocity(
			current_position, current_vel, desired_vel,
			other_aircraft, MAX_SWARM_SIZE, vehicle_id, dt
		);

	result.position_correction = vo_result.position_correction;
	result.collision_risk = (vo_result.avoided_aircraft_count > 0);
	result.critical_collision_risk = vo_result.emergency_avoidance;

	// 预测性轨迹碰撞检测
	predict_trajectory_collision(current_position, current_speed_xy, other_aircraft, result);

	return result;
}

/**
 * @brief 应用避撞修正到位置和速度
 * @param current_position 当前位置
 * @param current_speed_xy 当前水平速度
 * @param position_correction 位置修正量
 * @param other_aircraft 其他飞机位置信息
 */
void Swarm_Node::apply_avoidance_correction(const matrix::Vector3f &current_position,
					    float current_speed_xy,
					    const matrix::Vector3f &position_correction,
					    const OtherVehiclePosition *other_aircraft)
{
	bool is_high_speed_mode = current_speed_xy > _param_high_speed_threshold.get();

	const VelocityObstacleController::AvoidanceResult& vo_result = _vo_controller.get_last_result();

	// 速度替换逻辑
	float ttc_threshold = is_high_speed_mode ? 4.0f : 3.0f;
	float min_avoidance_speed = is_high_speed_mode ? (6.0f + current_speed_xy * 0.5f) : 3.0f;

	if (vo_result.time_to_collision < ttc_threshold) {
		matrix::Vector3f avoidance_vel = vo_result.safe_target_velocity;
		float avoidance_speed = avoidance_vel.norm();

		if (avoidance_speed < min_avoidance_speed && avoidance_speed > 0.1f) {
			avoidance_vel = avoidance_vel.normalized() * min_avoidance_speed;
		}
		_sp_vel_filtered(0) = avoidance_vel(0);
		_sp_vel_filtered(1) = avoidance_vel(1);
	}

	// 位置修正TTC阈值：根据速度动态调整
	float ttc_threshold_pos = (current_speed_xy > 5.0f) ? 3.0f :
				  (current_speed_xy > 1.0f) ? 3.5f : 4.0f;

	// 检查是否有主机在避撞范围内
	bool has_leader_collision = false;
	uint64_t current_time_check = hrt_absolute_time();
	for (int i = 0; i < MAX_SWARM_SIZE; i++) {
		if (other_aircraft[i].valid && other_aircraft[i].is_leader &&
		    (current_time_check - other_aircraft[i].timestamp) < DATA_VALID_TIMEOUT) {
			matrix::Vector3f leader_pos(other_aircraft[i].x, other_aircraft[i].y, 0.0f);
			matrix::Vector3f my_pos(current_position(0), current_position(1), 0.0f);
			if ((leader_pos - my_pos).norm() < _param_apf_max_distance.get() * 2.0f) {
				has_leader_collision = true;
				break;
			}
		}
	}

	// 计算位置修正系数
	float correction_scale = 1.0f;
	if (vo_result.time_to_collision < ttc_threshold_pos) {
		float ttc = vo_result.time_to_collision;
		float base_scale = is_high_speed_mode ? 3.0f : 1.5f;
		float ttc_scale = is_high_speed_mode ? 3.0f : 1.5f;
		if (has_leader_collision) {
			correction_scale = (base_scale + 1.0f) + (ttc_scale + 1.0f) * (ttc_threshold_pos - ttc) / ttc_threshold_pos;
		} else {
			correction_scale = base_scale + ttc_scale * (ttc_threshold_pos - ttc) / ttc_threshold_pos;
		}
	}

	// 应用位置修正
	_sp_position_filtered(0) += position_correction(0) * correction_scale;
	_sp_position_filtered(1) += position_correction(1) * correction_scale;
}

/**
 * @brief 计算最终速度指令
 * @param current_position 当前位置
 * @param formation_switching 是否发生编队切换
 * @param collision_risk 是否存在碰撞风险
 * @param enable_avoidance_check 是否启用避撞检查
 * @return 最终速度指令
 */
matrix::Vector3f Swarm_Node::calculate_final_velocity(const matrix::Vector3f &current_position,
						      bool formation_switching,
						      bool collision_risk,
						      bool enable_avoidance_check)
{
	matrix::Vector3f final_vel = _sp_vel_filtered;

	// 编队切换时限制速度并计算指向目标的速度指令
	if (formation_switching && (!collision_risk || !enable_avoidance_check)) {
		matrix::Vector3f target_position = _sp_position + _sp_offset;
		matrix::Vector3f to_target = target_position - current_position;
		float distance_to_target = sqrtf(to_target(0) * to_target(0) + to_target(1) * to_target(1));

		if (distance_to_target > 0.1f) {
			// 计算指向目标位置的方向
			matrix::Vector3f direction_to_target = to_target.normalized();
			float max_speed = _param_max_formation_speed.get();
			final_vel(0) = direction_to_target(0) * max_speed;
			final_vel(1) = direction_to_target(1) * max_speed;
		} else {
			// 已经接近目标位置，限制速度大小
			float vel_magnitude = sqrtf(final_vel(0) * final_vel(0) + final_vel(1) * final_vel(1));
			float max_speed = _param_max_formation_speed.get();
			if (vel_magnitude > max_speed) {
				final_vel(0) = final_vel(0) / vel_magnitude * max_speed;
				final_vel(1) = final_vel(1) / vel_magnitude * max_speed;
			}
		}
	}

	return final_vel;
}

/**
 * @brief 更新位置和速度滤波
 * @param current_speed_xy 当前水平速度
 * @param need_avoidance 是否需要避撞
 * @param formation_switching 是否发生编队切换
 * @param collision_risk 是否存在碰撞风险
 * @param critical_collision_risk 是否存在紧急碰撞风险
 * @param enable_avoidance_check 是否启用避撞检查
 */
void Swarm_Node::update_setpoint_filter(float current_speed_xy, bool need_avoidance, bool formation_switching,
					bool collision_risk, bool critical_collision_risk, bool enable_avoidance_check)
{
	// 根据当前速度判断速度模式
	bool is_high_speed_mode = current_speed_xy > _param_high_speed_threshold.get();
	bool is_low_speed_mode = current_speed_xy < _param_low_speed_threshold.get();
	bool is_formation_following = !need_avoidance;

	// 根据模式和碰撞风险动态调整滤波系数
	float alpha;
	if (is_formation_following) {
		alpha = _param_filter_alpha_formation.get();  // 编队跟随：平滑滤波
	} else if (is_high_speed_mode) {
		alpha = critical_collision_risk ? 0.9f : (collision_risk ? 0.7f : 0.5f);
	} else if (is_low_speed_mode) {
		alpha = critical_collision_risk ? 0.5f : (collision_risk ? 0.3f : 0.2f);
	} else {
		alpha = critical_collision_risk ? 0.7f : (collision_risk ? 0.5f : 0.3f);
	}

	// 计算期望位置并进行滤波
	matrix::Vector3f desired_position = _sp_position + _sp_offset;

	if (!_filter_initialized) {
		_sp_position_filtered = desired_position;
		_sp_vel_filtered = _sp_vel;
		_filter_initialized = true;
	} else {
		float pos_alpha = alpha;
		if (formation_switching && (!collision_risk || !enable_avoidance_check)) {
			pos_alpha = _param_filter_alpha_switching.get();  // 编队切换时平滑过渡
		}
		_sp_position_filtered = pos_alpha * desired_position + (1.f - pos_alpha) * _sp_position_filtered;
		_sp_vel_filtered = alpha * _sp_vel + (1.f - alpha) * _sp_vel_filtered;
	}
}

/**
 * @brief YAW角度展开处理，避免角度跳变
 */
void Swarm_Node::unwrap_yaw()
{
	if (PX4_ISFINITE(_last_sp_yaw)) {
		float yaw_diff = _sp_yaw - _last_sp_yaw;
		if (yaw_diff > M_PI_F) {
			_sp_yaw -= 2.f * M_PI_F;
		} else if (yaw_diff < -M_PI_F) {
			_sp_yaw += 2.f * M_PI_F;
		}
	}
	_last_sp_yaw = _sp_yaw;
}

void Swarm_Node::start_swarm_node()
{
	int32_t selected_leader_id = _param_swarm_leader_id.get();

	if (!_set_as_leader) {
		_swarm_start_flag_sub.copy(&_swarm_start_flag);

		if (_swarm_start_flag.stop_swarm) {
			STATE = state::LAND;
			return;
		}

		_vehicle_local_position_sub.copy(&_vehicle_local_position);

		// 更新主机信息
		if (!update_leader_info(selected_leader_id)) {
			PX4_WARN("主从跟随问题: _leader_sp_glo_pos无效 lat=%.6f lon=%.6f mav_id=%u",
				 (double)_leader_sp_glo_pos.lat, (double)_leader_sp_glo_pos.lon, _leader_sp_glo_pos.mavid);
			return;
		}

		_sensor_gps_sub.copy(&_sensor_gps);

		matrix::Vector3f vehicle_own_position;
		vehicle_own_position(0) = _vehicle_local_position.x;
		vehicle_own_position(1) = _vehicle_local_position.y;
		vehicle_own_position(2) = _vehicle_local_position.z;
		float takeoff_altitude = _param_take_altitude.get();

		// 计算目标位置并检测编队切换
		bool formation_switching = false;
		calculate_target_position(formation_switching);

		_position_sharing.update_other_positions(vehicle_id, _global_local_proj_ref, selected_leader_id);
		const OtherVehiclePosition* other_aircraft = _position_sharing.get_other_vehicles();

		matrix::Vector3f to_target = _final_target - vehicle_own_position;
		to_target(2) = 0.0f;
		float distance_to_target = to_target.norm();

		// 判断是否已到达目标位置（用于避撞优先级）
		float current_speed_xy = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
					       _vehicle_local_position.vy * _vehicle_local_position.vy);
		bool at_target_position = (distance_to_target < _param_at_target_distance.get()) && (current_speed_xy < _param_at_target_speed.get());

		// 发布位置信息，包含at_target状态
		_position_sharing.publish_position(vehicle_id, _vehicle_local_position, _sensor_gps, at_target_position);

		// 判断是否需要避撞：编队切换或距离目标较远（正在移动中）
		bool need_avoidance = formation_switching || (distance_to_target > _param_need_avoidance_distance.get());

		// 路径规划处理
		matrix::Vector3f actual_target = handle_path_planning(vehicle_own_position, formation_switching,
								      distance_to_target, other_aircraft);

		// 设置目标位置、速度和航向
		_sp_position = actual_target;
		_sp_vel(0) = _leader_sp_glo_pos.vx;
		_sp_vel(1) = _leader_sp_glo_pos.vy;
		_sp_vel(2) = _leader_sp_glo_pos.vz;
		_sp_yaw = _leader_sp_glo_pos.yaw;
		_sp_yaw_speed = _leader_sp_glo_pos.yawspeed;

		// 记录当前目标位置并处理YAW角度
		_last_target_position = _final_target;
		_last_target_valid = true;
		unwrap_yaw();

		// 检查是否处于起飞阶段（高度未达标）
		bool in_takeoff_phase = (vehicle_own_position(2) > -takeoff_altitude);

		// 起飞阶段日志
		static uint64_t last_takeoff_log = 0;
		uint64_t now_takeoff = hrt_absolute_time();
		if (in_takeoff_phase && (now_takeoff - last_takeoff_log > LOG_INTERVAL_US)) {
			PX4_INFO("[起飞阶段] 从机%d: 当前高度%.2fm 目标高度%.2fm 避撞已禁用",
				 vehicle_id, (double)(-vehicle_own_position(2)), (double)takeoff_altitude);
			last_takeoff_log = now_takeoff;
		}

		// 执行避撞检测
		CollisionAvoidanceResult avoidance_result = perform_collision_avoidance(
			vehicle_own_position, current_speed_xy, need_avoidance, in_takeoff_phase, other_aircraft);

		bool collision_risk = avoidance_result.collision_risk;
		bool critical_collision_risk = avoidance_result.critical_collision_risk;
		matrix::Vector3f position_correction = avoidance_result.position_correction;

		// 避撞检查开关
		bool enable_avoidance_check = _param_apf_enable.get() > 0.5f && !in_takeoff_phase && need_avoidance;

		// 更新位置和速度滤波
		update_setpoint_filter(current_speed_xy, need_avoidance, formation_switching,
				       collision_risk, critical_collision_risk, enable_avoidance_check);

		// 起飞逻辑：先爬升到设定高度，再执行编队
		if (uav_takeoff_altitude(vehicle_own_position, _sp_position_filtered, _sp_vel_filtered,
					 takeoff_altitude, _sp_yaw, _sp_yaw_speed)) {
			return;
		}

		// 只有在需要避撞时才应用避撞修正
		bool should_apply_avoidance = enable_avoidance_check && collision_risk;

		if (should_apply_avoidance) {
			apply_avoidance_correction(vehicle_own_position, current_speed_xy, position_correction, other_aircraft);
		}

		// 计算最终速度并发送控制指令
		matrix::Vector3f final_vel = calculate_final_velocity(vehicle_own_position, formation_switching,
								      collision_risk, enable_avoidance_check);

		control_instance::getInstance()->Control_pos_vel_yaw(
			_sp_position_filtered, final_vel, _sp_yaw, _sp_yaw_speed);
	}
}

void Swarm_Node::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// 处理参数更新
	handle_parameter_update();

	// 处理数据订阅
	swarm_start_flag_s _start_flag{};
	vehicle_status_s _state{};
	leader_info_s _leader_info{};

	_swarm_start_flag_sub.copy(&_start_flag);
	_vehicle_status_sub.copy(&_state);

	// 根据当前状态进行处理
	switch (STATE) {
	case state::INIT:
		handle_init_state();
		break;

	case state::ARM_AUTO:
		handle_arm_auto_state();
		break;

	case state::ARM_LEADER:
		handle_arm_leader_state();
		break;

	case state::ARM_OFFBOARD:
		handle_arm_offboard_state();
		break;

	case state::CONTROL:
		handle_control_state(_state, _leader_info);
		break;

	case state::LAND:
		handle_land_state();
		break;

	case state::DISARM:
		handle_disarm_state(_state);
		break;

	case state::IDLE:
		handle_idle_state(_start_flag);
		break;

	default:
		PX4_WARN("Unknown state: %d", STATE);
		break;
	}

	perf_end(_loop_perf);
}

// 处理参数更新
void Swarm_Node::handle_parameter_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		//  保存旧的_set_as_leader值，用于检测主机切换
		bool old_set_as_leader = _set_as_leader;

		updateParams();

		//  关键：参数更新后，重新设置swarm信息（包括_set_as_leader和_group_id）
		// 这样当QGC动态改变主机选择时，follower能够及时响应
		set_swarm_info(_param_swarm_set_leader.get(), _param_swarm_group_id.get());

		//  关键修复：如果主机角色发生变化（从机变主机或主机变从机），需要重新初始化状态机
		if (old_set_as_leader != _set_as_leader) {
			PX4_WARN("[主机切换] 飞机%d: 角色从%s切换为%s，重新初始化状态机",
				 vehicle_id, old_set_as_leader ? "主机" : "从机", _set_as_leader ? "主机" : "从机");

			// 重置初始化标志，强制重新初始化
			_init_done = false;

			// 重置状态机到INIT状态，让它重新根据新的角色进入正确的状态
			STATE = state::INIT;

			// 重新设置偏移量
			set_swarm_offset(Vector3f(_param_swarm_X_offset.get(), _param_swarm_Y_offset.get(), _param_swarm_Z_offset.get()));
			_offset_initialized = false;
		} else {
			// 角色未变化，只重新设置偏移量
			set_swarm_offset(Vector3f(_param_swarm_X_offset.get(), _param_swarm_Y_offset.get(), _param_swarm_Z_offset.get()));
			_offset_initialized = false;
		}

		// 调试输出：显示参数更新
		static uint64_t last_param_log = 0;
		uint64_t now = hrt_absolute_time();
		if (now - last_param_log > 2000000) {  // 每2秒最多输出一次
			PX4_INFO("[参数更新] 飞机%d: SWARM_SET_LEADER=%d, SWARM_GROUP_ID=%d, SWARM_LEADER_ID=%d",
				 vehicle_id, _param_swarm_set_leader.get(), _param_swarm_group_id.get(), _param_swarm_leader_id.get());
			last_param_log = now;
		}
	}
}


// 处理 INIT 状态
void Swarm_Node::handle_init_state()
{
	if (swarm_node_init()) {
		if (!_set_as_leader) {
			STATE = state::ARM_OFFBOARD;
			PX4_INFO("STATE = state::ARM_OFFBOARD;");
		} else if (_set_as_leader) {
			if (_swarm_start_flag.start_swarm_auto) {
				STATE = state::ARM_AUTO;
				PX4_INFO("STATE = state::ARM_AUTO;");
			} else if (_swarm_start_flag.start_swarm) {
				STATE = state::ARM_LEADER;
				PX4_INFO("STATE = state::ARM_LEADER;");
			}
		}
	}
}

// 处理 ARM_AUTO 状态
void Swarm_Node::handle_arm_auto_state()
{
	if (control_instance::getInstance()->Change_Auto() && control_instance::getInstance()->Arm_vehicle()) {
		STATE = state::CONTROL;
	} else {
		STATE = state::IDLE;
	}
}

// 处理 ARM_LEADER 状态
void Swarm_Node::handle_arm_leader_state()
{
	if (control_instance::getInstance()->Change_position_mode() && control_instance::getInstance()->Arm_vehicle()) {
		STATE = state::CONTROL;
	} else {
		STATE = state::IDLE;
	}
}

// 处理 ARM_OFFBOARD 状态
void Swarm_Node::handle_arm_offboard_state()
{
	if (control_instance::getInstance()->Change_offborad() && control_instance::getInstance()->Arm_vehicle()) {
		STATE = state::CONTROL;
		PX4_INFO("STATE = state::ARM_OFFBOARD;->state::CONTROL;");
	} else {
		STATE = state::IDLE;
		PX4_INFO("STATE = state::ARM_OFFBOARD;->state::IDLE;");
	}
}

// 处理 CONTROL 状态
void Swarm_Node::handle_control_state(vehicle_status_s _state, leader_info_s _leader_info)
{
	//  主机：只发布位置信息，不执行跟随逻辑
	if (_set_as_leader) {
		// 主机：发布自己的位置信息，让从机能够接收到
		vehicle_local_position_s vehicle_local_pos{};
		sensor_gps_s sensor_gps{};

		if (_vehicle_local_position_sub.copy(&vehicle_local_pos) && _sensor_gps_sub.copy(&sensor_gps)) {
			// 主机始终设置at_target=false，因为主机不需要避让其他飞机
			_position_sharing.publish_position(vehicle_id, vehicle_local_pos, sensor_gps, false);

			// 调试输出（每2秒一次）
			static uint64_t last_leader_publish_log = 0;
			uint64_t now = hrt_absolute_time();
			if (now - last_leader_publish_log > 2000000) {
				PX4_INFO("[主机发布] 主机%d: 发布位置信息 GPS(%.6f, %.6f) 给从机",
					 vehicle_id, (double)sensor_gps.latitude_deg, (double)sensor_gps.longitude_deg);
				last_leader_publish_log = now;
			}
		}

		//  关键：主机只需要发布位置信息，不需要执行跟随逻辑
		// 主机的控制完全由PX4的任务系统（AUTO模式）或用户手动控制
		// 不调用start_swarm_node()，避免干扰主机控制
		return;  // 主机提前返回，不执行后续的跟随控制逻辑
	}

	if (_leader_info.land == 1 ) {
		// 主机进入 MISSION 模式且准备降落，从机也应该进入降落

		// 同步从机目标高度为主机当前高度
		// 高度处理：主机发送的是 MSL 高度，需要转换为本地 NED z 坐标
		float leader_land_z = _vehicle_local_position.ref_alt - static_cast<float>(_leader_sp_glo_pos.alt);
		_sp_position(2) = leader_land_z;

		// 设置垂直速度为负值，确保从机开始降落
		_sp_vel(2) = 0.2f;  // 设置合适的下降速度（可以根据需要调整）

		// 确保水平速度为 0，避免水平运动
		_sp_vel(0) = 0.0f;
		_sp_vel(1) = 0.0f;

		// 发送控制命令，确保从机按目标位置、速度和航向进行降落
		control_instance::getInstance()->Control_pos_vel_yaw(
			_sp_position_filtered,
			_sp_vel_filtered,
			_sp_yaw,
			_sp_yaw_speed
		);

		// 如果从机已经接近降落目标，可以切换状态为 DISARM
		STATE = state::DISARM;  // 降落完成后停止控制
		return;
	}

	// 从机：检查是否处于 OFFBOARD 模式（主机已在上面 return）
	if (_state.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		// 判断是否人为接管或被切换到其他模式执行独立任务
		if (_state.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL ||
		    _state.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL ||
		    _state.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
		    _state.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB ||
		    _state.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) {
			PX4_WARN("Manual takeover detected, not switching back to OFFBOARD");
			return;

		} else if (_state.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
			   _state.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
			   _state.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
			   _state.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
			// 从机被切换到AUTO模式执行独立任务，不再跟随但保持避撞
			static uint64_t last_independent_log = 0;
			uint64_t now = hrt_absolute_time();
			if (now - last_independent_log > 2000000) {
				PX4_INFO("[独立模式] 从机%d: 执行独立任务(nav_state=%d)，保持避撞", vehicle_id, _state.nav_state);
				last_independent_log = now;
			}

			// 继续发布位置信息，供其他飞机避撞使用
			_vehicle_local_position_sub.copy(&_vehicle_local_position);
			_sensor_gps_sub.copy(&_sensor_gps);
			_position_sharing.publish_position(vehicle_id, _vehicle_local_position, _sensor_gps, true);

			// 持续发送OFFBOARD心跳信号，保持切回OFFBOARD的能力
			// 发送当前位置作为目标，不会影响AUTO模式的任务执行
			matrix::Vector3f current_pos(_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
			matrix::Vector3f zero_vel(0.f, 0.f, 0.f);
			control_instance::getInstance()->Control_pos_vel_yaw(current_pos, zero_vel, _vehicle_local_position.heading, 0.f);
			return;

		} else {
			PX4_WARN("Follower accidentally left OFFBOARD, attempting to re-enter...");
			control_instance::getInstance()->Change_offborad();
			return;
		}
	}

	//  2. 初始化偏移
	if (!_offset_initialized) {
		set_swarm_offset(Vector3f(
			_param_swarm_X_offset.get(),
			_param_swarm_Y_offset.get(),
			_param_swarm_Z_offset.get()));
		_offset_initialized = true;
	}

	//  3. 控制起始延迟
	if (first_loop == 1) {
		px4_usleep(1_s);
		first_loop = 0;
	}

	//  6. 暂停集群控制
	if (_swarm_start_flag.pause_swarm == 1) {
		STATE = _set_as_leader ? state::ARM_AUTO : state::ARM_OFFBOARD;
		return;
	}

	//  7. 从机执行跟随控制逻辑
	//  注意：主机已经在上面提前return了，这里只会执行从机的跟随逻辑
	start_swarm_node();

	//  8. 自动降落
	if (_state.arming_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		STATE = state::IDLE;
		return;
	}

	//  9. 主动触发 LAND
	if (_swarm_start_flag.stop_swarm == 1 || _leader_info.land == 1) {
		PX4_WARN("Swarm termination triggered, landing...");
		STATE = state::LAND;
		return;
	}
}

// 处理 LAND 状态
void Swarm_Node::handle_land_state()
{
	if (_swarm_start_flag.start_swarm || _swarm_start_flag.start_swarm_auto) {
		PX4_INFO("[Swarm] Received auto command in LAND state, switching to ARM state");

		//  关键修复：根据start_swarm_auto或start_swarm选择正确的状态
		if (_set_as_leader) {
			if (_swarm_start_flag.start_swarm_auto) {
				STATE = state::ARM_AUTO;  // 自动任务：进入ARM_AUTO
				PX4_INFO("[主机] 切换到ARM_AUTO状态，准备执行自动任务");
			} else if (_swarm_start_flag.start_swarm) {
				STATE = state::ARM_LEADER;  // 手动任务：进入ARM_LEADER
				PX4_INFO("[主机] 切换到ARM_LEADER状态");
			}
		} else {
			STATE = state::ARM_OFFBOARD;  // 从机：进入ARM_OFFBOARD
			PX4_INFO("[从机] 切换到ARM_OFFBOARD状态");
		}

		// 清除 stop_swarm 标志，确保后续状态判断正确
		swarm_start_flag_s clear_flag{};
		clear_flag.start_swarm = _swarm_start_flag.start_swarm;
		clear_flag.start_swarm_auto = _swarm_start_flag.start_swarm_auto;
		clear_flag.stop_swarm = 0;
		_start_flag_pub.publish(clear_flag);
		return;
	}

	if (control_instance::getInstance()->Change_land()) {
		STATE = state::DISARM;
	}
}

// 处理 DISARM 状态
void Swarm_Node::handle_disarm_state(vehicle_status_s _state)
{
	if (_state.arming_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		STATE = state::IDLE;
	}
}

// 处理 IDLE 状态
void Swarm_Node::handle_idle_state(swarm_start_flag_s _start_flag)
{
	_start_flag = {};
	_start_flag_pub.publish(_start_flag);
	STATE = state::INIT;
}

// 处理人工接管
void Swarm_Node::handle_manual_takeover(vehicle_status_s _state)
{
	const char *nav_mode_name = "UNKNOWN";

	switch (_state.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		nav_mode_name = "POSCTL";
		break;
	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		nav_mode_name = "ALTCTL";
		break;
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		nav_mode_name = "STABILIZED";
		break;
	default:
		break;
	}

	PX4_WARN("Remote takeover: OFFBOARD -> %s, exiting swarm control", nav_mode_name);
	STATE = state::IDLE;
}


int Swarm_Node::task_spawn(int argc, char *argv[])
{
	Swarm_Node *instance = new Swarm_Node();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Swarm_Node::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int Swarm_Node::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Swarm_Node::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Swarm node module for multi-UAV formation control.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("swarm_node", "template");
	PRINT_MODULE_USAGE_COMMAND("start");

	return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
	return Swarm_Node::main(argc, argv);
}




bool Swarm_Node::uav_takeoff_altitude(matrix::Vector3f vehicle_own_position, matrix::Vector3f target_position,
				      matrix::Vector3f target_velocity, float takeoff_altitude,
				      float target_yaw, float target_yawspeed)
{
	// 如果从机没有达到目标高度，则控制其爬升
	if (vehicle_own_position(2) > -takeoff_altitude) {
		// 控制从机逐步爬升至目标高度
		_sp_position(0) = vehicle_own_position(0);
		_sp_position(1) = vehicle_own_position(1);
		_sp_position(2) = target_position(2);

		// 设置目标速度，控制从机逐渐爬升
		_sp_vel(0) = 0.0f;
		_sp_vel(1) = 0.0f;
		_sp_vel(2) = target_velocity(2);

		// 调用控制器进行位置、速度和航向控制
		control_instance::getInstance()->Control_pos_vel_yaw(
			_sp_position, _sp_vel, target_yaw, target_yawspeed);

		PX4_INFO("Follower is climbing to %.2f meters... Current altitude: %.2f",
			 (double)takeoff_altitude, (double)_vehicle_local_position.z);
		return true;
	}
	return false;
}

