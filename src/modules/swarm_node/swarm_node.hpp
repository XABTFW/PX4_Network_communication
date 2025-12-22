#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <geo/geo.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include "control_node/control_instance.h"
#include <uORB/topics/leader_info.h>
#include <uORB/topics/follower_info.h>
#include <uORB/topics/swarm_start_flag.h>
#include <uORB/topics/leader_id.h>
#include "position_sharing.hpp"
#include "velocity_obstacle_controller.hpp"
#include "formation_planner.hpp"
#include <lib/geo/geo.h>

#define M_PI_PRECISE	3.141592653589793238462643383279502884

#define MAX_UAV_COUNT 20  // 根据实际需要调整
#define INVALID_MAVID 0

// ============== 编译时常量（不需要运行时调整） ==============
static constexpr float PATH_PLANNING_DISTANCE = 5.0f;       // 启用路径规划的最小距离
static constexpr float WAYPOINT_REACH_DISTANCE = 1.0f;      // 航点到达判定距离
static constexpr int PREDICTION_STEPS = 10;                 // 预测步数
static constexpr float PREDICTION_HORIZON = 1.0f;           // 预测时间范围 (秒)
static constexpr uint64_t DATA_VALID_TIMEOUT = 2000000;     // 数据有效超时 (微秒)
static constexpr uint64_t LOG_INTERVAL_US = 2000000;        // 日志输出间隔 (微秒)

class Swarm_Node : public ModuleBase<Swarm_Node>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Swarm_Node();
	~Swarm_Node() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;
	void set_swarm_offset(const matrix::Vector3f &sp_offset);
	void set_swarm_info(const int &set_as_leader,const int &group_id);

private:
	bool _filter_initialized{false};
	bool _init_done = false;

	matrix::Vector3f _sp_position_filtered{0.f, 0.f, 0.f};
	matrix::Vector3f _sp_vel_filtered{0.f, 0.f, 0.f};

	float _last_sp_yaw{NAN};

	bool _offset_initialized{false};
	bool _map_ref_initialized{false};
	void Run() override;
	bool swarm_node_init();
	void start_swarm_node();
	bool update_leader_info(int32_t selected_leader_id);  // 更新主机信息
	bool calculate_target_position(bool &formation_switching);  // 计算目标位置并检测编队切换
	matrix::Vector3f handle_path_planning(const matrix::Vector3f &current_position, bool formation_switching,
	                                      float distance_to_target, const OtherVehiclePosition *other_aircraft);  // 路径规划处理

	// 避撞检测结果结构体
	struct CollisionAvoidanceResult {
		bool collision_risk{false};           // 是否存在碰撞风险
		bool critical_collision_risk{false};  // 是否存在紧急碰撞风险
		matrix::Vector3f position_correction{0.f, 0.f, 0.f};  // 位置修正量
	};

	CollisionAvoidanceResult perform_collision_avoidance(const matrix::Vector3f &current_position,
	                                                      float current_speed_xy, bool need_avoidance,
	                                                      bool in_takeoff_phase, const OtherVehiclePosition *other_aircraft);  // 避撞检测

	void configure_vo_controller(float current_speed_xy);  // 配置速度障碍参数

	void predict_trajectory_collision(const matrix::Vector3f &current_position, float current_speed_xy,
	                                  const OtherVehiclePosition *other_aircraft,
	                                  CollisionAvoidanceResult &result);  // 预测性轨迹碰撞检测

	void apply_avoidance_correction(const matrix::Vector3f &current_position, float current_speed_xy,
	                                const matrix::Vector3f &position_correction,
	                                const OtherVehiclePosition *other_aircraft);  // 应用避撞修正

	matrix::Vector3f calculate_final_velocity(const matrix::Vector3f &current_position, bool formation_switching,
	                                          bool collision_risk, bool enable_avoidance_check);  // 计算最终速度

	void update_setpoint_filter(float current_speed_xy, bool need_avoidance, bool formation_switching,
	                            bool collision_risk, bool critical_collision_risk, bool enable_avoidance_check);  // 更新位置速度滤波

	void unwrap_yaw();  // YAW角度展开处理

	bool uav_takeoff_altitude(matrix::Vector3f vehicle_own_position, matrix::Vector3f target_position, matrix::Vector3f target_velocity, float takeoff_altitude, float target_yaw, float target_yawspeed);

	void handle_parameter_update();
	void handle_init_state();
	void handle_arm_auto_state();
	void handle_arm_leader_state();
	void handle_arm_offboard_state();
	void handle_control_state(vehicle_status_s _state,leader_info_s _leader_info);
	void handle_land_state();
	void handle_disarm_state(vehicle_status_s _state);
	void handle_idle_state(swarm_start_flag_s _start_flag);
	void handle_manual_takeover(vehicle_status_s _state);

	enum state {
		INIT = 0,
		ARM_OFFBOARD,
		ARM_LEADER,
		ARM_AUTO,
		TAKEOFF,
		CONTROL,
		LAND,
		DISARM,
		IDLE
	};
	state STATE = INIT;

	float target_x;
	float target_y;
	int vehicle_id = 1;  // 将在init()中从MAV_SYS_ID参数获取
	vehicle_local_position_s _vehicle_local_position;
	sensor_gps_s _sensor_gps;

	leader_info_s _leader_sp_glo_pos{};
	swarm_start_flag_s _swarm_start_flag{};
	leader_id_s _leader_group_id{};

	MapProjection _global_local_proj_ref{};

	matrix::Vector3f _sp_position;
	matrix::Vector3f _sp_offset;
	matrix::Vector3f _sp_vel;
	float _sp_yaw;
	float _sp_yaw_speed;

	//  用于检测编队切换时的目标位置突变
	matrix::Vector3f _last_target_position{0.f, 0.f, 0.f};
	bool _last_target_valid{false};

	//  路径规划相关
	matrix::Vector3f _current_waypoint{0.f, 0.f, 0.f};  // 当前航点（绕行点）
	bool _using_detour_waypoint{false};  // 是否正在使用绕行航点
	matrix::Vector3f _final_target{0.f, 0.f, 0.f};  // 最终目标（编队位置）

	bool _set_as_leader;
	int  _group_id;
	bool first_loop = 1;

	// 从机间位置共享管理器
	PositionSharing _position_sharing;

	//  基于速度障碍的避撞控制器（主要使用）
	VelocityObstacleController _vo_controller;

	//  编队路径规划器
	FormationPlanner _formation_planner;
	// Subscriptions


	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules Swarm when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _leader_info_sub{ORB_ID(leader_info)};
	uORB::Subscription _swarm_start_flag_sub{ORB_ID(swarm_start_flag)};

	uORB::Publication<leader_id_s>   			_group_id_pub{ORB_ID(leader_id)};
	uORB::Publication<swarm_start_flag_s>			_start_flag_pub{ORB_ID(swarm_start_flag)};

	uORB::Publication<leader_info_s>			_leader_info_pub{ORB_ID(leader_info)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		// Swarm parameters
		(ParamInt<px4::params::SWARM_SET_LEADER>)    _param_swarm_set_leader,
		(ParamInt<px4::params::SWARM_GROUP_ID>)      _param_swarm_group_id,
		(ParamInt<px4::params::SWARM_LEADER_ID>)      _param_swarm_leader_id,  //  选定的主机ID
		(ParamFloat<px4::params::SWARM_TOFF_ALT>)    _param_take_altitude,
		(ParamFloat<px4::params::SWARM_X_OFFSET>)    _param_swarm_X_offset,
		(ParamFloat<px4::params::SWARM_Y_OFFSET>)    _param_swarm_Y_offset,
		(ParamFloat<px4::params::SWARM_Z_OFFSET>)    _param_swarm_Z_offset,

		// APF配置参数
		(ParamInt<px4::params::SWARM_APF_ENABLE>) _param_apf_enable,
		(ParamFloat<px4::params::SWARM_APF_SAFE_R>) _param_apf_safety_radius,
		(ParamFloat<px4::params::SWARM_APF_DANGER>) _param_apf_danger_radius,
		(ParamFloat<px4::params::SWARM_APF_MAXD>) _param_apf_max_distance,
		(ParamFloat<px4::params::SWARM_APF_REP_G>) _param_apf_repulsive_gain,
		(ParamFloat<px4::params::SWARM_APF_TAN_G>) _param_apf_tangential_gain,
		(ParamFloat<px4::params::SWARM_APF_MAX_F>) _param_apf_max_force,
		(ParamInt<px4::params::SWARM_APF_LEADER>) _param_apf_enable_leader_avoidance,

		// 编队控制参数
		(ParamFloat<px4::params::SWARM_FRM_SWTHR>) _param_formation_switch_threshold,  // 编队切换检测阈值
		(ParamFloat<px4::params::SWARM_AVOID_DST>) _param_need_avoidance_distance,     // 需要避撞的距离阈值
		(ParamFloat<px4::params::SWARM_AT_TGT_D>) _param_at_target_distance,           // 判定到达目标的距离
		(ParamFloat<px4::params::SWARM_AT_TGT_V>) _param_at_target_speed,              // 判定到达目标的速度
		(ParamFloat<px4::params::SWARM_MAX_SPD>) _param_max_formation_speed,           // 编队切换最大速度
		(ParamFloat<px4::params::SWARM_HI_SPD>) _param_high_speed_threshold,           // 高速模式阈值
		(ParamFloat<px4::params::SWARM_LO_SPD>) _param_low_speed_threshold,            // 低速模式阈值
		(ParamFloat<px4::params::SWARM_FLT_FRM>) _param_filter_alpha_formation,        // 编队跟随滤波系数
		(ParamFloat<px4::params::SWARM_FLT_SWT>) _param_filter_alpha_switching,        // 编队切换滤波系数

		// 从机碰撞检测阈值
		(ParamFloat<px4::params::SWARM_F_COL_D>) _param_collision_dist_follower,       // 从机碰撞检测距离
		(ParamFloat<px4::params::SWARM_F_CLS_D>) _param_close_dist_follower,           // 从机近距离阈值
		(ParamFloat<px4::params::SWARM_F_CRT_D>) _param_critical_dist_follower,        // 从机紧急距离阈值
		(ParamFloat<px4::params::SWARM_F_WRN_D>) _param_warning_dist_follower,         // 从机警告距离阈值

		// 主机碰撞检测阈值
		(ParamFloat<px4::params::SWARM_L_COL_D>) _param_collision_dist_leader,         // 主机碰撞检测距离
		(ParamFloat<px4::params::SWARM_L_CLS_D>) _param_close_dist_leader,             // 主机近距离阈值
		(ParamFloat<px4::params::SWARM_L_CRT_D>) _param_critical_dist_leader,          // 主机紧急距离阈值
		(ParamFloat<px4::params::SWARM_L_WRN_D>) _param_warning_dist_leader            // 主机警告距离阈值
	)
};
