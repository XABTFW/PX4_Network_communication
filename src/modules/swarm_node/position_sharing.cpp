#include "position_sharing.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

PositionSharing::PositionSharing()
{
	// OtherVehiclePosition 结构体已有默认值，无需显式初始化
}

void PositionSharing::publish_position(uint8_t mavid,
                                       const vehicle_local_position_s &vehicle_local_position,
                                       const sensor_gps_s &sensor_gps,
                                       uint8_t group_id,
                                       bool is_leader,
                                       bool at_target)
{
	uav_info_s pos{};
	pos.timestamp = hrt_absolute_time();
	pos.mavid = mavid;
	pos.group_id = group_id;
	pos.is_leader = is_leader;
	pos.lat = sensor_gps.latitude_deg;
	pos.lon = sensor_gps.longitude_deg;
	pos.alt = sensor_gps.altitude_msl_m;
	pos.vx = vehicle_local_position.vx;
	pos.vy = vehicle_local_position.vy;
	pos.vz = vehicle_local_position.vz;
	pos.yaw = vehicle_local_position.heading_good_for_control ? vehicle_local_position.heading : 0.0f;
	pos.yawspeed = vehicle_local_position.delta_heading;
	pos.land = 0;
	pos.at_target = at_target ? 1 : 0;  // 传递是否已到达目标位置

	_uav_info_pub.publish(pos);
}

void PositionSharing::update_other_positions(uint8_t current_vehicle_id,
                                             MapProjection &global_local_proj,
                                             int32_t leader_id)
{
	uav_info_s leader_pos{};
	follower_info_s follower_pos{};

	// 读取 UAV_INFO 消息（主机位置）
	while (_uav_info_sub.update(&leader_pos)) {
		if (leader_pos.mavid == current_vehicle_id || leader_pos.mavid == 0 ||
		    leader_pos.mavid >= MAX_SWARM_SIZE) {
			continue;
		}

		if (!global_local_proj.isInitialized() ||
		    !PX4_ISFINITE(leader_pos.lat) || !PX4_ISFINITE(leader_pos.lon)) {
			continue;
		}

		float local_x, local_y;
		global_local_proj.project(leader_pos.lat, leader_pos.lon, local_x, local_y);

		// 使用消息中的 is_leader 字段，而不是硬编码
		update_vehicle_position(leader_pos.mavid, local_x, local_y, leader_pos.alt,
		                       leader_pos.vx, leader_pos.vy, leader_pos.vz,
		                       leader_pos.yaw, leader_pos.timestamp,
		                       leader_pos.group_id, leader_pos.is_leader,
		                       leader_pos.at_target == 1);
	}

	// 读取 FOLLOWER_INFO 消息（从机位置）
	while (_follower_info_sub.update(&follower_pos)) {
		if (follower_pos.mavid == current_vehicle_id || follower_pos.mavid == 0 ||
		    follower_pos.mavid >= MAX_SWARM_SIZE) {
			continue;
		}

		if (!global_local_proj.isInitialized() ||
		    !PX4_ISFINITE(follower_pos.lat) || !PX4_ISFINITE(follower_pos.lon)) {
			continue;
		}

		float local_x, local_y;
		global_local_proj.project(follower_pos.lat, follower_pos.lon, local_x, local_y);

		if (!PX4_ISFINITE(local_x) || !PX4_ISFINITE(local_y) ||
		    !PX4_ISFINITE(follower_pos.vx) || !PX4_ISFINITE(follower_pos.vy)) {
			continue;
		}

		// follower_info 消息暂时没有 group_id 和 is_leader，使用默认值
		// 从机的 is_leader 为 false
		bool is_leader = (leader_id > 0) ? (follower_pos.mavid == (uint32_t)leader_id)
		                                 : (follower_pos.mavid == 2);

		update_vehicle_position(follower_pos.mavid, local_x, local_y, follower_pos.alt,
		                       follower_pos.vx, follower_pos.vy, follower_pos.vz,
		                       follower_pos.yaw, follower_pos.timestamp,
		                       0, is_leader,  // group_id 暂时为0
		                       follower_pos.at_target == 1);
	}
}

int PositionSharing::get_valid_vehicle_count() const
{
	int count = 0;
	uint64_t current_time = hrt_absolute_time();

	for (int i = 0; i < MAX_SWARM_SIZE; i++) {
		if (_other_vehicles[i].valid &&
		    (current_time - _other_vehicles[i].timestamp) < 2000000) {
			count++;
		}
	}
	return count;
}

void PositionSharing::update_vehicle_position(uint8_t mavid, float x, float y, float z,
                                              float vx, float vy, float vz, float yaw,
                                              uint64_t timestamp, uint8_t group_id,
                                              bool is_leader, bool at_target)
{
	if (mavid >= MAX_SWARM_SIZE) {
		return;
	}

	OtherVehiclePosition& v = _other_vehicles[mavid];
	v.mavid = mavid;
	v.group_id = group_id;
	v.x = x;
	v.y = y;
	v.z = z;
	v.vx = vx;
	v.vy = vy;
	v.vz = vz;
	v.yaw = yaw;
	v.timestamp = timestamp;
	v.valid = true;
	v.is_leader = is_leader;
	v.at_target = at_target;
}


