#pragma once

#include <px4_platform_common/defines.h>
#include <stdint.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/leader_info.h>
#include <uORB/topics/follower_info.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gps.h>
#include <lib/geo/geo.h>

// 定义其他飞机位置结构体
struct OtherVehiclePosition {
    uint8_t mavid;
    float x, y, z;
    float vx, vy, vz;
    float yaw;
    uint64_t timestamp;
    bool valid = false;
    bool is_leader = false;            // 是否为主机
    bool at_target = false;            // 是否已到达目标位置（用于避撞优先级判断）
};

#define MAX_SWARM_SIZE 10

/**
 * @brief 位置共享管理器类
 *
 * 负责飞机之间的位置信息共享和同步
 */
class PositionSharing
{
public:
	PositionSharing();
	~PositionSharing() = default;

	/**
	 * @brief 发布本机位置给其他飞机
	 * @param mavid 飞机ID
	 * @param vehicle_local_position 本地位置信息
	 * @param sensor_gps GPS信息
	 * @param at_target 是否已到达目标位置（用于避撞优先级）
	 */
    void publish_position(uint8_t mavid,
                          const vehicle_local_position_s &vehicle_local_position,
                          const sensor_gps_s &sensor_gps,
                          bool at_target = false);

	/**
	 * @brief 更新其他飞机位置信息（使用GPS坐标转换）
	 * @param current_vehicle_id 当前飞机ID
	 * @param global_local_proj 本地坐标投影引用（用于GPS转本地坐标）
	 * @param leader_id 选定的主机ID（0=自动检测/向后兼容ID=2，>0=指定ID）
	 */
	void update_other_positions(uint8_t current_vehicle_id,
	                            MapProjection &global_local_proj,
	                            int32_t leader_id = 0);

	/**
	 * @brief 获取其他飞机位置数组
	 */
	const OtherVehiclePosition* get_other_vehicles() const { return _other_vehicles; }

	/**
	 * @brief 获取有效飞机数量
	 */
	int get_valid_vehicle_count() const;

	/**
	 * @brief 更新单个飞机位置数据（智能更新机制）
	 * @param mavid 飞机ID
	 * @param x, y, z 局部坐标
	 * @param vx, vy, vz 速度
	 * @param yaw 航向
	 * @param timestamp 时间戳
	 * @param is_leader 是否为主机
	 * @param at_target 是否已到达目标位置
	 */
	void update_vehicle_position(uint8_t mavid, float x, float y, float z,
	                             float vx, float vy, float vz, float yaw,
	                             uint64_t timestamp, bool is_leader = false, bool at_target = false);


private:
	// uORB发布和订阅
	uORB::Publication<leader_info_s> _leader_info_pub{ORB_ID(leader_info)};
	uORB::Subscription _leader_info_sub{ORB_ID(leader_info)};  // 订阅主机LEADER_INFO消息
	uORB::Subscription _follower_info_sub{ORB_ID(follower_info)};  // 订阅从机FOLLOWER_INFO消息

	// 其他飞机位置数组（索引对应vehicle_id）
    OtherVehiclePosition _other_vehicles[MAX_SWARM_SIZE];

};
