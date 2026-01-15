#pragma once

/**
 * @file mission_sync.hpp
 * @brief 集群任务同步模块 - 主机向从机广播任务航点，从机存储并在主机丢失时独立执行
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/swarm_mission_item.h>
#include <uORB/topics/mission.h>
#include <dataman_client/DatamanClient.hpp>
#include <navigator/navigation.h>
#include <lib/geo/geo.h>

// 最大支持的航点数量
static constexpr int MAX_MISSION_ITEMS = 50;

// 主机信号丢失超时时间 (微秒)
static constexpr uint64_t LEADER_LOST_TIMEOUT_US = 3000000;  // 3秒

// 任务同步间隔 (微秒) - 加快广播频率
static constexpr uint64_t MISSION_SYNC_INTERVAL_US = 100000;  // 0.1秒（原来0.5秒太慢）

/**
 * @brief 存储的航点信息
 */
struct StoredWaypoint {
	uint16_t seq{0};              // 航点序号
	uint16_t nav_cmd{0};          // 导航命令
	double lat{0.0};              // 纬度
	double lon{0.0};              // 经度
	float alt{0.0f};              // 高度 (AMSL)
	float yaw{NAN};               // 航向
	float acceptance_radius{10.0f}; // 接受半径
	float loiter_radius{0.0f};    // 盘旋半径
	float time_inside{0.0f};      // 停留时间
	bool autocontinue{true};      // 自动继续
	bool valid{false};            // 是否有效
};

/**
 * @brief 任务同步模块
 */
class MissionSync {
public:
	MissionSync() = default;
	~MissionSync() = default;

	/**
	 * @brief 初始化模块
	 * @param vehicle_id 本机ID
	 * @param group_id 组号
	 * @param is_leader 是否为主机
	 */
	void init(uint8_t vehicle_id, uint8_t group_id, bool is_leader);

	/**
	 * @brief 更新角色信息
	 */
	void update_role(uint8_t group_id, bool is_leader);

	/**
	 * @brief 主机：检测并广播任务航点
	 * @return true 如果有任务更新被广播
	 */
	bool leader_broadcast_mission();

	/**
	 * @brief 从机：在主机丢失后转发已存储的航点给同组其他从机
	 * @return true 如果有航点被转发
	 */
	bool follower_relay_mission();

	/**
	 * @brief 从机：接收并存储主机发来的航点
	 * @return true 如果接收到新航点
	 */
	bool follower_receive_mission();

	/**
	 * @brief 从机：独立模式下只接收航点数据，不更新_current_seq
	 * @return true 如果接收到新航点
	 */
	bool follower_receive_waypoints_only();

	/**
	 * @brief 检查主机信号是否丢失
	 * @param last_leader_timestamp 最后收到主机位置的时间戳
	 * @return true 如果主机信号丢失
	 */
	bool is_leader_lost(uint64_t last_leader_timestamp) const;

	/**
	 * @brief 获取当前应该飞往的航点 (从机独立模式)
	 * @param current_pos 当前位置 (本地坐标)
	 * @param proj_ref 坐标投影参考
	 * @param target_pos [out] 目标位置 (本地坐标)
	 * @param target_yaw [out] 目标航向
	 * @return true 如果有有效航点
	 */
	bool get_current_waypoint(const matrix::Vector3f &current_pos,
				  MapProjection &proj_ref,
				  matrix::Vector3f &target_pos,
				  float &target_yaw);

	/**
	 * @brief 检查是否到达当前航点，如果是则前进到下一个
	 * @param current_pos 当前位置 (本地坐标)
	 * @param proj_ref 坐标投影参考
	 * @return true 如果到达航点并前进
	 */
	bool advance_waypoint_if_reached(const matrix::Vector3f &current_pos,
					 MapProjection &proj_ref);

	/**
	 * @brief 检查是否有有效的存储任务
	 */
	bool has_valid_mission() const { return _mission_valid && _total_count > 0; }

	/**
	 * @brief 获取任务完成状态
	 */
	bool is_mission_complete() const { return _mission_complete; }

	/**
	 * @brief 重置任务状态 (用于重新开始跟随)
	 */
	void reset_mission_state();

	/**
	 * @brief 获取当前航点序号
	 */
	uint16_t get_current_seq() const { return _current_seq; }

	/**
	 * @brief 获取总航点数
	 */
	uint16_t get_total_count() const { return _total_count; }

	/**
	 * @brief 强制前进到下一个航点（用于独立模式下的到达判定）
	 */
	void force_advance_waypoint();

private:
	/**
	 * @brief 从 dataman 读取任务航点
	 */
	bool load_mission_from_dataman();

	/**
	 * @brief 发送单个航点
	 */
	void send_waypoint(uint16_t seq, uint8_t sync_type);

	/**
	 * @brief 存储接收到的航点
	 */
	void store_waypoint(const swarm_mission_item_s &item);

	// 基本信息
	uint8_t _vehicle_id{1};
	uint8_t _group_id{1};
	bool _is_leader{false};
	bool _initialized{false};

	// 任务状态
	bool _mission_valid{false};
	bool _mission_complete{false};
	uint32_t _mission_id{0};
	uint16_t _total_count{0};
	uint16_t _current_seq{0};

	// 存储的航点
	StoredWaypoint _waypoints[MAX_MISSION_ITEMS];

	// 主机：任务广播状态
	uint32_t _last_broadcast_mission_id{0};
	uint16_t _broadcast_seq{0};
	uint64_t _last_broadcast_time{0};
	bool _broadcast_in_progress{false};
	uint16_t _round_robin_idx{0};  // 轮流广播索引

	// 从机：接收状态
	uint32_t _receiving_mission_id{0};
	uint16_t _received_count{0};
	uint64_t _last_receive_time{0};

	// 从机：转发状态（用于主机丢失后转发航点给同组其他从机）
	uint64_t _last_relay_time{0};
	uint16_t _relay_round_robin_idx{0};

	// Dataman 客户端 (主机读取任务)
	DatamanClient _dataman_client{};

	// uORB 发布/订阅
	uORB::Publication<swarm_mission_item_s> _mission_item_pub{ORB_ID(swarm_mission_item)};
	uORB::Subscription _mission_item_sub{ORB_ID(swarm_mission_item)};
	uORB::Subscription _mission_sub{ORB_ID(mission)};
};
