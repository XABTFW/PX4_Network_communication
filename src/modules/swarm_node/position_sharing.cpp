#include "position_sharing.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

PositionSharing::PositionSharing()
{
	// 初始化其他飞机位置数组（增强版初始化）
	for (int i = 0; i < MAX_SWARM_SIZE; i++) {
		_other_vehicles[i].mavid = 0;
		_other_vehicles[i].x = 0.0f;
		_other_vehicles[i].y = 0.0f;
		_other_vehicles[i].z = 0.0f;
		_other_vehicles[i].vx = 0.0f;
		_other_vehicles[i].vy = 0.0f;
		_other_vehicles[i].vz = 0.0f;
		_other_vehicles[i].yaw = 0.0f;
		_other_vehicles[i].timestamp = 0;
		_other_vehicles[i].valid = false;

		//  初始化新增字段
		_other_vehicles[i].last_update_time = 0;
		_other_vehicles[i].update_count = 0;
		_other_vehicles[i].distance = 0.0f;
		_other_vehicles[i].is_leader = false;
		_other_vehicles[i].signal_quality = 0;
	}

	PX4_INFO("位置共享管理器初始化完成");

    // 初始化主机缓存（mavid会在运行时动态更新）
    _last_host.mavid = 0;
    _last_host.valid = false;
}

/**
 * @brief 发布本机位置给其他飞机（通过LEADER_INFO）
 */
void PositionSharing::publish_position(uint8_t mavid,
                                       const vehicle_local_position_s &vehicle_local_position,
                                       const sensor_gps_s &sensor_gps)
{
	leader_info_s pos{};
	pos.timestamp = hrt_absolute_time();
	pos.mavid = mavid;

	// 设置GPS位置（LEADER_INFO消息格式要求）
	pos.lat = sensor_gps.latitude_deg;
	pos.lon = sensor_gps.longitude_deg;
	pos.alt = sensor_gps.altitude_msl_m;  // 海拔高度

	// 设置速度
	pos.vx = vehicle_local_position.vx;
	pos.vy = vehicle_local_position.vy;
	pos.vz = vehicle_local_position.vz;

	// 设置航向信息
	if (vehicle_local_position.heading_good_for_control) {
		pos.yaw = vehicle_local_position.heading;  // NED坐标系：0=北，π/2=东
	} else {
		pos.yaw = 0.0f;  // 航向无效时默认朝北
	}

	pos.yawspeed = vehicle_local_position.delta_heading;  // 航向速度
	pos.land = 0;  // 着陆状态（从机间通信时设为0）

	_leader_info_pub.publish(pos);

	// 调试信息：位置发布 - 增强版
	static uint64_t last_publish_time = 0;
	uint64_t now = hrt_absolute_time();
	if (now - last_publish_time > 2000000) {  // 每2秒打印一次
		PX4_INFO("[位置发布] 从机%d: GPS(%.6f, %.6f), MAV_ID:%d 已通过LEADER_INFO发布",
		         mavid, (double)sensor_gps.latitude_deg, (double)sensor_gps.longitude_deg, mavid);

		// 检查uORB发布状态
		bool advertised = _leader_info_pub.advertised();
		PX4_INFO("[发布状态] 从机%d: LEADER_INFO uORB发布状态: %s", mavid, advertised ? "已广播" : "未广播");
		last_publish_time = now;
	}
}

/**
 * @brief 更新其他飞机位置信息（通过LEADER_INFO和FOLLOWER_INFO）
 */
void PositionSharing::update_other_positions(uint8_t current_vehicle_id,
                                             MapProjection &global_local_proj,
                                             int32_t leader_id)
{
	// 读取所有可用的LEADER_INFO和FOLLOWER_INFO消息
	leader_info_s leader_pos{};
	follower_info_s follower_pos{};
	int message_count = 0;


	// 读取所有LEADER_INFO消息（主机数据和其他从机数据）
	while (_leader_info_sub.update(&leader_pos)) {
		message_count++;

		//  判断是否是主机：如果leader_id>0，检查mavid是否匹配；如果leader_id=0，使用第一个（向后兼容）
		bool is_leader = false;
		if (leader_id > 0) {
			// 指定模式：检查mavid是否匹配选定的主机ID
			//  修复类型不匹配：将int32_t转换为uint32_t进行比较
			is_leader = (leader_pos.mavid == (uint32_t)leader_id);
		} else {
			// 自动模式：使用第一个收到的LEADER_INFO作为主机（向后兼容，默认ID=2）
			// 为了向后兼容，如果mavid=2，也认为是主机
			is_leader = (leader_pos.mavid == 2);
		}

		if (is_leader) {
			PX4_INFO("[主从跟随] 从机%d: 接收到主机%d GPS(%.6f, %.6f), 用于跟随控制",
			         current_vehicle_id, leader_pos.mavid,
			         (double)leader_pos.lat, (double)leader_pos.lon);

			// 新增：将主机数据也保存到other_vehicles中用于APF避撞
			if (global_local_proj.isInitialized() &&
			    PX4_ISFINITE(leader_pos.lat) && PX4_ISFINITE(leader_pos.lon)) {
				float local_x, local_y, local_z;
				global_local_proj.project(leader_pos.lat, leader_pos.lon, local_x, local_y);
				local_z = leader_pos.alt;  // 使用海拔高度

				// 将主机位置保存到other_vehicles中，用于APF避撞
				update_vehicle_position(leader_pos.mavid, local_x, local_y, local_z,
				                       leader_pos.vx, leader_pos.vy, leader_pos.vz,
				                       leader_pos.yaw, leader_pos.timestamp, true);  // is_leader = true

				PX4_INFO("[APF避撞] 从机%d: 主机%d位置已保存到other_vehicles用于避撞 位置=(%.2f,%.2f,%.2f)",
				         current_vehicle_id, leader_pos.mavid, (double)local_x, (double)local_y, (double)local_z);
			}
		} else {
			// 其他从机的LEADER_INFO消息也保存用于避撞
			PX4_INFO("[LEADER_INFO避撞] 从机%d: 收到从机%d的LEADER_INFO消息，将保存用于避撞",
			         current_vehicle_id, leader_pos.mavid);

			// 将其他从机的LEADER_INFO数据也保存到other_vehicles中
			if (global_local_proj.isInitialized() &&
			    PX4_ISFINITE(leader_pos.lat) && PX4_ISFINITE(leader_pos.lon)) {
				float local_x, local_y, local_z;
				global_local_proj.project(leader_pos.lat, leader_pos.lon, local_x, local_y);
				local_z = leader_pos.alt;

				update_vehicle_position(leader_pos.mavid, local_x, local_y, local_z,
				                       leader_pos.vx, leader_pos.vy, leader_pos.vz,
				                       leader_pos.yaw, leader_pos.timestamp, false);  // is_leader = false
			}
		}
	}

	// 读取所有FOLLOWER_INFO消息（仅从机数据，用于避撞）
	while (_follower_info_sub.update(&follower_pos)) {
		message_count++;


		//  关键检查：验证mavid是否正确
		if (follower_pos.mavid == current_vehicle_id) {
			PX4_WARN(" 警告：从机%d收到了自己的FOLLOWER_INFO消息，这可能导致数据混淆", current_vehicle_id);
		} else {
			PX4_INFO(" 确认：从机%d收到了其他从机%d的消息", current_vehicle_id, follower_pos.mavid);
		}

		// 不保存自己的位置（正常过滤）
		if (follower_pos.mavid == current_vehicle_id || follower_pos.mavid == 0) {
			// 正常过滤，不输出调试信息避免刷屏
			continue;
		}

		// 处理所有从机数据（包括主机数据的重复接收）
		// 主机数据可能出现在FOLLOWER_INFO中，用于数据一致性检查
		//  使用leader_id参数来判断是否是主机（不再硬编码ID=2）
		bool is_leader_in_follower_info = false;
		if (leader_id > 0) {
			//  修复类型不匹配：将int32_t转换为uint32_t进行比较
			is_leader_in_follower_info = (follower_pos.mavid == (uint32_t)leader_id);
		} else {
			// 向后兼容：默认ID=2为主机
			is_leader_in_follower_info = (follower_pos.mavid == 2);
		}
		
		if (is_leader_in_follower_info) {
			PX4_INFO("[数据同步] 从机%d: 收到主机%d数据在FOLLOWER_INFO中，用于备份验证",
			         current_vehicle_id, follower_pos.mavid);
			// 注意：这里存储主机数据用于备份，但不影响主从跟随逻辑
		}

		// 检查mavid是否在有效范围内
		if (follower_pos.mavid >= MAX_SWARM_SIZE) {
			continue;
		}

		// 将GPS坐标转换为局部坐标
		float local_x, local_y, local_z;
		if (global_local_proj.isInitialized() &&
		    PX4_ISFINITE(follower_pos.lat) && PX4_ISFINITE(follower_pos.lon)) {
			global_local_proj.project(follower_pos.lat, follower_pos.lon, local_x, local_y);
		} else {
			// GPS坐标转换失败，跳过
			continue;
		}

		// 转换绝对高度到局部高度（直接使用GPS高度差值）
		local_z = follower_pos.alt;

		//  接收端数据有效性检查：过滤无效数据
		bool received_data_valid = PX4_ISFINITE(local_x) &&
		                           PX4_ISFINITE(local_y) &&
		                           PX4_ISFINITE(local_z) &&
		                           PX4_ISFINITE(follower_pos.vx) &&
		                           PX4_ISFINITE(follower_pos.vy) &&
		                           PX4_ISFINITE(follower_pos.vz) &&
		                           PX4_ISFINITE(follower_pos.yaw);

		// 如果接收到的数据无效，跳过更新（保持上一次的有效值）
		if (!received_data_valid) {
			continue;
		}

		// 使用新的智能更新方法存储数据
		//  使用leader_id参数来判断是否是主机（不再硬编码ID=2）
		bool is_leader = false;
		if (leader_id > 0) {
			//  修复类型不匹配：将int32_t转换为uint32_t进行比较
			is_leader = (follower_pos.mavid == (uint32_t)leader_id);
		} else {
			// 向后兼容：默认ID=2为主机
			is_leader = (follower_pos.mavid == 2);
		}
		update_vehicle_position(follower_pos.mavid, local_x, local_y, local_z,
		                       follower_pos.vx, follower_pos.vy, follower_pos.vz,
		                       follower_pos.yaw, follower_pos.timestamp, is_leader);

		//  详细调试：立即显示存储结果
		const OtherVehiclePosition& stored_pos = _other_vehicles[follower_pos.mavid];
		PX4_INFO("[数据存储] 从机%d: 成功存储%s%d 位置(%.2f,%.2f,%.2f) valid=%s 更新次数=%u",
		         current_vehicle_id,
		         is_leader ? "主机" : "从机", follower_pos.mavid,
		         (double)stored_pos.x, (double)stored_pos.y, (double)stored_pos.z,
		         stored_pos.valid ? "是" : "否", stored_pos.update_count);
	}

// 调试：每2秒输出所有存储的飞机位置信息
	static uint64_t last_summary_time = 0;
	uint64_t now = hrt_absolute_time();
	if (now - last_summary_time > 2000000) {  // 每2秒打印一次
		int valid_count = 0;
		PX4_INFO("=== 从机%d: 存储的飞机数据汇总 ===", current_vehicle_id);

		for (int i = 0; i < MAX_SWARM_SIZE; i++) {
			if (_other_vehicles[i].valid) {
				uint64_t age = now - _other_vehicles[i].timestamp;

				//  只显示2秒内的有效数据
				if (age < 2000000) { // 2秒内
					const char* type = (_other_vehicles[i].is_leader) ? "主机" : "从机";
					const char* status = (age < 1000000) ? "实时" : "较旧";

					PX4_INFO("[有效数据] %s%d: 位置(%.2f,%.2f,%.2f) 时延:%lums 更新:%u次 %s [2秒内]",
					         type, i,
					         (double)_other_vehicles[i].x, (double)_other_vehicles[i].y, (double)_other_vehicles[i].z,
					         age / 1000, _other_vehicles[i].update_count, status);

					if (i != current_vehicle_id) {
						valid_count++;
					}
				}
			}
		}

		PX4_INFO("=== 总计: %d架其他飞机 (可用于避撞) ===", valid_count);

		// 显示通信统计
		PX4_INFO("[通信总结] 从机%d: 本轮处理%d条消息 [LEADER_INFO+FOLLOWER_INFO]",
		         current_vehicle_id, message_count);
		last_summary_time = now;
	}

	//  自动清理超过2秒的过期数据，节省内存
	cleanup_expired_data(2000); // 2秒超时
}

/**
 * @brief 获取有效飞机数量
 */
int PositionSharing::get_valid_vehicle_count() const
{
	int count = 0;
	uint64_t current_time = hrt_absolute_time();

	for (int i = 0; i < MAX_SWARM_SIZE; i++) {
		if (_other_vehicles[i].valid) {
			//  只统计2秒内的有效数据
			uint64_t age = current_time - _other_vehicles[i].timestamp;
			if (age < 2000000) { // 2秒内
				count++;
			}
		}
	}
	return count;
}

/**
 * @brief 获取主机位置（方便从机使用）
 */
const OtherVehiclePosition& PositionSharing::get_host_position() const
{
	// 优先使用实时主机位置
	if (_other_vehicles[2].valid) {
		uint64_t time_diff = hrt_absolute_time() - _other_vehicles[2].timestamp;

		// 如果主机数据较新（1秒内），返回实时数据
		if (time_diff < 1000000) {
			return _other_vehicles[2];
		}
	}

	// 否则返回缓存的主机位置
	return _last_host;
}

// 🆕 数据管理方法实现

/**
 * @brief 更新单个飞机位置数据（智能更新机制）
 */
void PositionSharing::update_vehicle_position(uint8_t mavid, float x, float y, float z,
                                            float vx, float vy, float vz, float yaw,
                                            uint64_t timestamp, bool is_leader)
{
	// 检查mavid是否在有效范围内
	if (mavid >= MAX_SWARM_SIZE) {
		return;
	}

	// 记录当前时间
	uint64_t current_time = hrt_absolute_time();
	bool first_time = !_other_vehicles[mavid].valid;

	// 更新基本数据
	_other_vehicles[mavid].mavid = mavid;
	_other_vehicles[mavid].x = x;
	_other_vehicles[mavid].y = y;
	_other_vehicles[mavid].z = z;
	_other_vehicles[mavid].vx = vx;
	_other_vehicles[mavid].vy = vy;
	_other_vehicles[mavid].vz = vz;
	_other_vehicles[mavid].yaw = yaw;
	_other_vehicles[mavid].timestamp = timestamp;
	_other_vehicles[mavid].valid = true;

	// 🆕更新新增字段
	_other_vehicles[mavid].last_update_time = current_time;
	_other_vehicles[mavid].is_leader = is_leader;
	if (first_time) {
		_other_vehicles[mavid].update_count = 1;
	} else {
		_other_vehicles[mavid].update_count++;
	}

	// 调试信息：详细的数据更新日志
	if (first_time) {
		PX4_INFO("[新飞机] 首次记录飞机%s%d 位置(%.2f,%.2f,%.2f)",
		         is_leader ? "主机" : "从机", mavid, (double)x, (double)y, (double)z);
	}
}

/**
 * @brief 计算到指定飞机的距离
 */
float PositionSharing::calculate_distance_to(uint8_t mavid, float my_x, float my_y, float my_z) const
{
	if (mavid >= MAX_SWARM_SIZE || !_other_vehicles[mavid].valid) {
		return -1.0f; // 无效距离
	}

	float dx = _other_vehicles[mavid].x - my_x;
	float dy = _other_vehicles[mavid].y - my_y;
	float dz = _other_vehicles[mavid].z - my_z;

	return sqrtf(dx*dx + dy*dy + dz*dz);
}

/**
 * @brief 获取指定半径内的所有飞机（用于避撞）
 */
int PositionSharing::get_nearby_vehicles(float my_x, float my_y, float my_z, float radius,
                                       OtherVehiclePosition* results, int max_results) const
{
	int count = 0;
	uint64_t current_time = hrt_absolute_time();

	for (int i = 0; i < MAX_SWARM_SIZE && count < max_results; i++) {
		if (!_other_vehicles[i].valid) {
			continue;
		}

		//  检查数据是否过期（2秒内有效）
		uint64_t age = current_time - _other_vehicles[i].timestamp;
		if (age > 2000000) { // 2秒
			continue;
		}

		// 计算距离
		float distance = calculate_distance_to(i, my_x, my_y, my_z);
		if (distance > 0 && distance <= radius) {
			// 复制数据并更新距离字段
			results[count] = _other_vehicles[i];
			results[count].distance = distance;
			count++;
		}
	}

	return count;
}

/**
 * @brief 清理过期数据（2秒超时自动清理）
 */
void PositionSharing::cleanup_expired_data(uint64_t timeout_ms)
{
	uint64_t current_time = hrt_absolute_time();
	int cleaned_count = 0;
	static uint64_t last_cleanup_log = 0;

	for (int i = 0; i < MAX_SWARM_SIZE; i++) {
		if (_other_vehicles[i].valid) {
			uint64_t age = current_time - _other_vehicles[i].timestamp;
			if (age > timeout_ms * 1000) { // 毫秒转微秒
				//  清理超过2秒的过期数据
				const char* vehicle_type = (_other_vehicles[i].is_leader) ? "主机" : "从机";
				PX4_INFO("[数据过期] %s%d 数据超时%lums，已清理 [节省内存]",
				         vehicle_type, i, age / 1000);

				// 重置数据结构
				_other_vehicles[i].valid = false;
				_other_vehicles[i].update_count = 0;
				_other_vehicles[i].signal_quality = 0;
				_other_vehicles[i].distance = 0.0f;
				_other_vehicles[i].last_update_time = 0;
				cleaned_count++;
			}
		}
	}

	// 限流日志：只在有清理操作时记录，且每5秒最多一次
	if (cleaned_count > 0) {
		uint64_t now = hrt_absolute_time();
		if (now - last_cleanup_log > 5000000) { // 5秒
			PX4_INFO("[内存优化] 清理了%d条超时数据，只保留2秒内的有效位置", cleaned_count);
			last_cleanup_log = now;
		}
	}
}
