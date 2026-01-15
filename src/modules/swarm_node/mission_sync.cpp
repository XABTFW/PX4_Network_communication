#include "mission_sync.hpp"
#include <mathlib/mathlib.h>

void MissionSync::init(uint8_t vehicle_id, uint8_t group_id, bool is_leader)
{
	_vehicle_id = vehicle_id;
	_group_id = group_id;
	_is_leader = is_leader;
	_initialized = true;

	// 清空航点存储
	for (int i = 0; i < MAX_MISSION_ITEMS; i++) {
		_waypoints[i] = StoredWaypoint{};
	}

	PX4_INFO("[MissionSync] 初始化: 飞机%d, 组%d, %s",
		 vehicle_id, group_id, is_leader ? "主机" : "从机");
}

void MissionSync::update_role(uint8_t group_id, bool is_leader)
{
	if (_group_id != group_id || _is_leader != is_leader) {
		_group_id = group_id;
		_is_leader = is_leader;

		// 角色变化时重置状态
		reset_mission_state();

		PX4_INFO("[MissionSync] 角色更新: 组%d, %s",
			 group_id, is_leader ? "主机" : "从机");
	}
}

void MissionSync::reset_mission_state()
{
	_mission_complete = false;
	_broadcast_in_progress = false;
	_broadcast_seq = 0;

	// 不清除已存储的航点，保留用于恢复
}

bool MissionSync::load_mission_from_dataman()
{
	mission_s mission{};

	if (!_mission_sub.copy(&mission)) {
		return false;
	}

	if (mission.count == 0 || mission.count > MAX_MISSION_ITEMS) {
		return false;
	}

	PX4_INFO("[MissionSync] 主机%d: 加载任务, %d个航点, mission_id=%u",
		 _vehicle_id, mission.count, mission.mission_id);

	// 从 dataman 读取所有航点
	const dm_item_t dm_item = static_cast<dm_item_t>(mission.mission_dataman_id);
	int valid_count = 0;

	for (uint16_t i = 0; i < mission.count && i < MAX_MISSION_ITEMS; i++) {
		mission_item_s item{};

		bool success = _dataman_client.readSync(dm_item, i,
							reinterpret_cast<uint8_t *>(&item),
							sizeof(mission_item_s));

		if (success) {
			// 只存储位置航点 (NAV_CMD_WAYPOINT, NAV_CMD_LOITER_*, NAV_CMD_LAND 等)
			if (item.nav_cmd == NAV_CMD_WAYPOINT ||
			    item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
			    item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    item.nav_cmd == NAV_CMD_LAND ||
			    item.nav_cmd == NAV_CMD_TAKEOFF) {

				_waypoints[valid_count].seq = i;
				_waypoints[valid_count].nav_cmd = item.nav_cmd;
				_waypoints[valid_count].lat = item.lat;
				_waypoints[valid_count].lon = item.lon;
				_waypoints[valid_count].alt = item.altitude;
				_waypoints[valid_count].yaw = item.yaw;
				_waypoints[valid_count].acceptance_radius = item.acceptance_radius;
				_waypoints[valid_count].loiter_radius = item.loiter_radius;
				_waypoints[valid_count].time_inside = item.time_inside;
				_waypoints[valid_count].autocontinue = item.autocontinue;
				_waypoints[valid_count].valid = true;

				valid_count++;
			}
		}
	}

	_total_count = valid_count;
	_mission_id = mission.mission_id;
	_current_seq = 0;
	_mission_valid = (valid_count > 0);
	_mission_complete = false;

	PX4_INFO("[MissionSync] 主机%d: 加载了%d个有效航点", _vehicle_id, valid_count);

	return _mission_valid;
}

bool MissionSync::leader_broadcast_mission()
{
	if (!_is_leader || !_initialized) {
		return false;
	}

	uint64_t now = hrt_absolute_time();

	// 检查是否需要加载/更新任务
	mission_s mission{};
	if (_mission_sub.copy(&mission)) {
		// 只有当任务ID不同且不在广播中时才加载新任务
		if (mission.count > 0 && mission.mission_id != _last_broadcast_mission_id && !_broadcast_in_progress) {
			PX4_INFO("[主机广播] 检测到新任务: count=%d, mission_id=%u (当前=%u)",
				 mission.count, mission.mission_id, _last_broadcast_mission_id);

			// 先更新 _last_broadcast_mission_id 防止重复加载
			_last_broadcast_mission_id = mission.mission_id;

			// 任务更新，重新加载
			if (load_mission_from_dataman()) {
				_broadcast_seq = 0;
				_broadcast_in_progress = true;

				// 发送任务开始标记
				send_waypoint(0, swarm_mission_item_s::SYNC_START);
				_last_broadcast_time = now;

				PX4_INFO("[MissionSync] 主机%d: 开始广播任务, %d个航点, mission_id=%u",
					 _vehicle_id, _total_count, _last_broadcast_mission_id);
				return true;
			} else {
				// 加载失败，重置
				_last_broadcast_mission_id = 0;
			}
		}
	}

	// 正在广播中，继续发送航点（每次发送多个，加快同步速度）
	if (_broadcast_in_progress && _mission_valid) {
		if (now - _last_broadcast_time >= MISSION_SYNC_INTERVAL_US) {
			// 每次发送最多3个航点
			for (int i = 0; i < 3 && _broadcast_seq < _total_count; i++) {
				send_waypoint(_broadcast_seq, swarm_mission_item_s::SYNC_SINGLE);
				_broadcast_seq++;
			}
			_last_broadcast_time = now;

			if (_broadcast_seq >= _total_count) {
				// 发送任务结束标记
				send_waypoint(_total_count - 1, swarm_mission_item_s::SYNC_END);
				_broadcast_in_progress = false;

				PX4_INFO("[MissionSync] 主机%d: 任务广播完成", _vehicle_id);
			}
			return true;
		}
	}

	// 周期性发送所有航点（轮流发送，确保从机收到完整任务）
	if (_mission_valid && !_broadcast_in_progress) {
		if (now - _last_broadcast_time >= MISSION_SYNC_INTERVAL_US) {
			// 获取主机当前执行的航点序号
			mission_s current_mission{};
			if (_mission_sub.copy(&current_mission) && current_mission.current_seq >= 0) {
				// 找到对应的存储航点索引
				for (uint16_t i = 0; i < _total_count; i++) {
					if (_waypoints[i].seq == (uint16_t)current_mission.current_seq) {
						_current_seq = i;
						break;
					}
				}
			}

			// 每次发送2个航点，加快同步速度
			for (int i = 0; i < 2; i++) {
				uint16_t wp_to_send = _round_robin_idx % _total_count;
				send_waypoint(wp_to_send, swarm_mission_item_s::SYNC_SINGLE);
				_round_robin_idx++;
			}

			_last_broadcast_time = now;
			return true;
		}
	}

	return false;
}

bool MissionSync::follower_relay_mission()
{
	// 从机在主机丢失后，将已存储的航点转发给同组其他从机
	// 这样当次要主机丢失时，组内从机也能收到航点信息
	if (_is_leader || !_initialized || !_mission_valid || _total_count == 0) {
		return false;
	}

	uint64_t now = hrt_absolute_time();

	// 周期性转发航点
	if (now - _last_relay_time >= MISSION_SYNC_INTERVAL_US * 2) {
		// 每次转发2个航点，加快同步速度
		for (int i = 0; i < 2; i++) {
			uint16_t wp_to_relay = _relay_round_robin_idx % _total_count;

			if (_waypoints[wp_to_relay].valid) {
				// 构造转发消息，标记为从机转发
				swarm_mission_item_s msg{};
				msg.timestamp = now;
				msg.group_id = _group_id;
				msg.leader_id = _vehicle_id;  // 使用从机自己的ID
				msg.mission_id = _mission_id;
				msg.total_count = _total_count;
				msg.current_seq = _current_seq;
				msg.sync_type = swarm_mission_item_s::SYNC_SINGLE;
				msg.is_relay = true;  // 标记为转发消息

				const StoredWaypoint &wp = _waypoints[wp_to_relay];
				msg.seq = wp_to_relay;
				msg.nav_cmd = wp.nav_cmd;
				msg.lat = wp.lat;
				msg.lon = wp.lon;
				msg.alt = wp.alt;
				msg.yaw = wp.yaw;
				msg.acceptance_radius = wp.acceptance_radius;
				msg.loiter_radius = wp.loiter_radius;
				msg.time_inside = wp.time_inside;
				msg.autocontinue = wp.autocontinue;

				_mission_item_pub.publish(msg);
			}

			_relay_round_robin_idx++;
		}
		_last_relay_time = now;
		return true;
	}

	return false;
}

void MissionSync::send_waypoint(uint16_t idx, uint8_t sync_type)
{
	if (idx >= _total_count && sync_type == swarm_mission_item_s::SYNC_SINGLE) {
		return;
	}

	swarm_mission_item_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.group_id = _group_id;
	msg.leader_id = _vehicle_id;
	msg.mission_id = _mission_id;
	msg.total_count = _total_count;
	msg.current_seq = _current_seq;
	msg.sync_type = sync_type;
	msg.is_relay = false;  // 主机发送的消息，不是转发

	if (sync_type != swarm_mission_item_s::SYNC_CLEAR && idx < _total_count) {
		const StoredWaypoint &wp = _waypoints[idx];  // idx 是数组索引
		msg.seq = idx;  // 使用数组索引作为序号，保持一致性
		msg.nav_cmd = wp.nav_cmd;
		msg.lat = wp.lat;
		msg.lon = wp.lon;
		msg.alt = wp.alt;
		msg.yaw = wp.yaw;
		msg.acceptance_radius = wp.acceptance_radius;
		msg.loiter_radius = wp.loiter_radius;
		msg.time_inside = wp.time_inside;
		msg.autocontinue = wp.autocontinue;
	}

	_mission_item_pub.publish(msg);
}

bool MissionSync::follower_receive_mission()
{
	if (_is_leader || !_initialized) {
		return false;
	}

	swarm_mission_item_s item{};
	bool received_new = false;

	while (_mission_item_sub.update(&item)) {
		// 只接收同组的消息
		if (item.group_id != _group_id) {
			continue;
		}

		// 跳过自己发送的转发消息
		if (item.is_relay && item.leader_id == _vehicle_id) {
			continue;
		}

		_last_receive_time = hrt_absolute_time();

		switch (item.sync_type) {
		case swarm_mission_item_s::SYNC_START:
			// 新任务开始（只接受主机发送的开始标记，不接受转发的）
			if (!item.is_relay && item.mission_id != _receiving_mission_id) {
				_receiving_mission_id = item.mission_id;
				_received_count = 0;
				_total_count = item.total_count;
				_mission_valid = false;
				_mission_complete = false;

				// 清空旧航点
				for (int i = 0; i < MAX_MISSION_ITEMS; i++) {
					_waypoints[i].valid = false;
				}

				PX4_INFO("[MissionSync] 从机%d: 开始接收任务, 预计%d个航点",
					 _vehicle_id, item.total_count);
			}
			break;

		case swarm_mission_item_s::SYNC_SINGLE:
			// 存储航点（接受主机发送的和从机转发的）
			store_waypoint(item);
			received_new = true;

			// 更新主机当前航点（只从主机消息更新，不从转发消息更新）
			if (!item.is_relay) {
				_current_seq = item.current_seq;
			}

			// 更新任务信息
			if (item.total_count > 0) {
				_total_count = item.total_count;
				_mission_id = item.mission_id;
				// 如果已经收到航点，标记任务有效
				if (_received_count > 0) {
					_mission_valid = true;
				}
			}
			break;

		case swarm_mission_item_s::SYNC_END:
			// 任务接收完成（只接受主机发送的结束标记）
			if (!item.is_relay) {
				store_waypoint(item);
				if (_mission_id != item.mission_id) {
					// 只在新任务完成时输出日志
					_mission_id = item.mission_id;
					_mission_valid = (_received_count > 0);
					PX4_INFO("[MissionSync] 从机%d: 任务接收完成, 共%d个航点",
						 _vehicle_id, _received_count);
				} else {
					_mission_valid = (_received_count > 0);
				}
				received_new = true;
			}
			break;

		case swarm_mission_item_s::SYNC_CLEAR:
			// 清除任务（只接受主机发送的清除标记）
			if (!item.is_relay) {
				_mission_valid = false;
				_total_count = 0;
				_received_count = 0;
				for (int i = 0; i < MAX_MISSION_ITEMS; i++) {
					_waypoints[i].valid = false;
				}
				PX4_INFO("[MissionSync] 从机%d: 任务已清除", _vehicle_id);
			}
			break;
		}
	}

	return received_new;
}

bool MissionSync::follower_receive_waypoints_only()
{
	// 独立模式下只接收航点数据，不更新_current_seq
	// 用于补充可能丢失的航点
	if (_is_leader || !_initialized) {
		return false;
	}

	swarm_mission_item_s item{};
	bool received_new = false;

	while (_mission_item_sub.update(&item)) {
		// 只接收同组的消息
		if (item.group_id != _group_id) {
			continue;
		}

		// 跳过自己发送的转发消息
		if (item.is_relay && item.leader_id == _vehicle_id) {
			continue;
		}

		// 只处理单个航点消息，忽略START/END/CLEAR
		if (item.sync_type == swarm_mission_item_s::SYNC_SINGLE) {
			store_waypoint(item);
			received_new = true;
		}
	}

	return received_new;
}

void MissionSync::store_waypoint(const swarm_mission_item_s &item)
{
	// item.seq 现在是数组索引 (0, 1, 2...)，直接用作存储位置
	uint16_t store_idx = item.seq;

	if (store_idx >= MAX_MISSION_ITEMS) {
		return;
	}

	// 验证GPS数据有效性 - 拒绝无效航点
	if (fabs(item.lat) < 0.0001 && fabs(item.lon) < 0.0001) {
		return;
	}

	StoredWaypoint &wp = _waypoints[store_idx];
	bool is_new = !wp.valid;

	wp.seq = item.seq;
	wp.nav_cmd = item.nav_cmd;
	wp.lat = item.lat;
	wp.lon = item.lon;
	wp.alt = item.alt;
	wp.yaw = item.yaw;
	wp.acceptance_radius = item.acceptance_radius;
	wp.loiter_radius = item.loiter_radius;
	wp.time_inside = item.time_inside;
	wp.autocontinue = item.autocontinue;
	wp.valid = true;

	if (is_new) {
		_received_count++;
	}

	// 更新总数
	if (item.total_count > 0) {
		_total_count = item.total_count;
	}
}

bool MissionSync::is_leader_lost(uint64_t last_leader_timestamp) const
{
	uint64_t now = hrt_absolute_time();
	return (now - last_leader_timestamp) > LEADER_LOST_TIMEOUT_US;
}

bool MissionSync::get_current_waypoint(const matrix::Vector3f &current_pos,
				       MapProjection &proj_ref,
				       matrix::Vector3f &target_pos,
				       float &target_yaw)
{
	if (!_mission_valid || _mission_complete) {
		return false;
	}

	// _current_seq 现在是数组索引 (0, 1, 2...)
	if (_current_seq >= _total_count) {
		_mission_complete = true;
		return false;
	}

	// 直接用 _current_seq 作为数组索引
	const StoredWaypoint &current_wp = _waypoints[_current_seq];

	if (!current_wp.valid) {
		// 当前航点无效，尝试找下一个有效航点
		for (uint16_t i = _current_seq + 1; i < _total_count; i++) {
			if (_waypoints[i].valid) {
				_current_seq = i;
				return get_current_waypoint(current_pos, proj_ref, target_pos, target_yaw);
			}
		}
		_mission_complete = true;
		return false;
	}

	// 检查GPS数据有效性（lat/lon不能都是0）
	if (fabs(current_wp.lat) < 0.0001 && fabs(current_wp.lon) < 0.0001) {
		// GPS数据无效，跳到下一个航点
		_current_seq++;
		if (_current_seq >= _total_count) {
			_mission_complete = true;
			return false;
		}
		return get_current_waypoint(current_pos, proj_ref, target_pos, target_yaw);
	}

	// 将GPS坐标转换为本地坐标
	float local_x, local_y;
	proj_ref.project(current_wp.lat, current_wp.lon, local_x, local_y);

	target_pos(0) = local_x;
	target_pos(1) = local_y;
	target_pos(2) = -(current_wp.alt);  // NED坐标系，高度取负

	target_yaw = PX4_ISFINITE(current_wp.yaw) ? current_wp.yaw : NAN;

	return true;
}

bool MissionSync::advance_waypoint_if_reached(const matrix::Vector3f &current_pos,
					      MapProjection &proj_ref)
{
	if (!_mission_valid || _mission_complete) {
		return false;
	}

	// _current_seq 现在是数组索引 (0, 1, 2...)
	if (_current_seq >= _total_count) {
		_mission_complete = true;
		return false;
	}

	const StoredWaypoint &current_wp = _waypoints[_current_seq];

	if (!current_wp.valid) {
		return false;
	}

	// 计算到航点的距离
	float local_x, local_y;
	proj_ref.project(current_wp.lat, current_wp.lon, local_x, local_y);

	float dx = current_pos(0) - local_x;
	float dy = current_pos(1) - local_y;
	float distance_xy = sqrtf(dx * dx + dy * dy);

	// 检查是否到达航点
	float acceptance = fmaxf(current_wp.acceptance_radius, 5.0f);  // 最小5米
	if (distance_xy < acceptance) {
		// 前进到下一个航点
		_current_seq++;

		if (_current_seq >= _total_count) {
			_mission_complete = true;
			PX4_INFO("[MissionSync] 从机%d: 独立任务完成!", _vehicle_id);
		} else {
			PX4_INFO("[MissionSync] 从机%d: 到达航点%d, 前进到航点%d",
				 _vehicle_id, _current_seq - 1, _current_seq);
		}
		return true;
	}

	return false;
}


void MissionSync::force_advance_waypoint()
{
	if (_mission_complete || _current_seq >= _total_count) {
		_mission_complete = true;
		return;
	}

	_current_seq++;

	if (_current_seq >= _total_count) {
		_mission_complete = true;
		PX4_INFO("[MissionSync] 从机%d: 独立任务完成!", _vehicle_id);
	}
}
