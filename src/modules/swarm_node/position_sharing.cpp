#include "position_sharing.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

PositionSharing::PositionSharing()
{
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
		_other_vehicles[i].is_leader = false;
	}
}

void PositionSharing::publish_position(uint8_t mavid,
                                       const vehicle_local_position_s &vehicle_local_position,
                                       const sensor_gps_s &sensor_gps)
{
	leader_info_s pos{};
	pos.timestamp = hrt_absolute_time();
	pos.mavid = mavid;
	pos.lat = sensor_gps.latitude_deg;
	pos.lon = sensor_gps.longitude_deg;
	pos.alt = sensor_gps.altitude_msl_m;
	pos.vx = vehicle_local_position.vx;
	pos.vy = vehicle_local_position.vy;
	pos.vz = vehicle_local_position.vz;
	pos.yaw = vehicle_local_position.heading_good_for_control ? vehicle_local_position.heading : 0.0f;
	pos.yawspeed = vehicle_local_position.delta_heading;
	pos.land = 0;

	_leader_info_pub.publish(pos);
}

void PositionSharing::update_other_positions(uint8_t current_vehicle_id,
                                             MapProjection &global_local_proj,
                                             int32_t leader_id)
{
	// 读取所有可用的LEADER_INFO和FOLLOWER_INFO消息
	leader_info_s leader_pos{};
	follower_info_s follower_pos{};

	// 读取所有LEADER_INFO消息
	while (_leader_info_sub.update(&leader_pos)) {
		// 判断是否是主机
		bool is_leader = false;
		if (leader_id > 0) {
			is_leader = (leader_pos.mavid == (uint32_t)leader_id);
		} else {
			is_leader = (leader_pos.mavid == 2);
		}

		if (is_leader) {
			// 将主机数据保存到other_vehicles中用于避撞
			if (global_local_proj.isInitialized() &&
			    PX4_ISFINITE(leader_pos.lat) && PX4_ISFINITE(leader_pos.lon)) {
				float local_x, local_y, local_z;
				global_local_proj.project(leader_pos.lat, leader_pos.lon, local_x, local_y);
				local_z = leader_pos.alt;

				update_vehicle_position(leader_pos.mavid, local_x, local_y, local_z,
				                       leader_pos.vx, leader_pos.vy, leader_pos.vz,
				                       leader_pos.yaw, leader_pos.timestamp, true);
			}
		}
	}

	// 读取所有FOLLOWER_INFO消息
	while (_follower_info_sub.update(&follower_pos)) {


		// 不保存自己的位置
		if (follower_pos.mavid == current_vehicle_id || follower_pos.mavid == 0) {
			continue;
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
			continue;
		}

		local_z = follower_pos.alt;

		// 数据有效性检查
		if (!PX4_ISFINITE(local_x) || !PX4_ISFINITE(local_y) || !PX4_ISFINITE(local_z) ||
		    !PX4_ISFINITE(follower_pos.vx) || !PX4_ISFINITE(follower_pos.vy) ||
		    !PX4_ISFINITE(follower_pos.vz) || !PX4_ISFINITE(follower_pos.yaw)) {
			continue;
		}

		// 判断是否是主机
		bool is_leader = false;
		if (leader_id > 0) {
			is_leader = (follower_pos.mavid == (uint32_t)leader_id);
		} else {
			is_leader = (follower_pos.mavid == 2);
		}
		
		update_vehicle_position(follower_pos.mavid, local_x, local_y, local_z,
		                       follower_pos.vx, follower_pos.vy, follower_pos.vz,
		                       follower_pos.yaw, follower_pos.timestamp, is_leader);
	}
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
			uint64_t age = current_time - _other_vehicles[i].timestamp;
			if (age < 2000000) { // 2秒内
				count++;
			}
		}
	}
	return count;
}

/**
 * @brief 更新单个飞机位置数据
 */
void PositionSharing::update_vehicle_position(uint8_t mavid, float x, float y, float z,
                                            float vx, float vy, float vz, float yaw,
                                            uint64_t timestamp, bool is_leader)
{
	// 检查mavid是否在有效范围内
	if (mavid >= MAX_SWARM_SIZE) {
		return;
	}

	// 更新数据
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
	_other_vehicles[mavid].is_leader = is_leader;
}

/**
 * @brief 计算到指定飞机的距离
 */
