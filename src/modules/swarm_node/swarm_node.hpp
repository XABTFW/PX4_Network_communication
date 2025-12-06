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
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include "position_sharing.hpp"
#include "apf_controller.hpp"
#include "velocity_obstacle_controller.hpp"
#include "formation_planner.hpp"
#include <lib/geo/geo.h>
//#include <map>

//mc_control_instance* mc_control_instance::instance = nullptr;
#define M_PI_PRECISE	3.141592653589793238462643383279502884

#define MAX_UAV_COUNT 20  // 根据实际需要调整
#define INVALID_MAVID 0

struct uav_cache_entry_s {
	    uint32_t mavid;
	    leader_info_s info;
	    bool valid;
	    uint64_t last_update_time;
	};

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
	// void update_uav_info();
    // void clear_cache();

private:
	bool _filter_initialized{false};
	bool _init_done = false;

	matrix::Vector3f _sp_position_filtered{0.f, 0.f, 0.f};
	matrix::Vector3f _sp_vel_filtered{0.f, 0.f, 0.f};

	float _last_sp_yaw{NAN};

	bool _offset_initialized{false};
	bool _map_ref_initialized{false};
	void Run() override;
	//mc_control_instance* instance = mc_control_instance::getInstance();
//mc_control_instance mc_ctl_instance;
	bool takeoff();
	bool swarm_node_init();
	bool arm_offboard();
	void start_swarm_node();
	double calculate_distance(double lat1, double lon1, double lat2, double lon2);
	leader_info_s try_get_leader_info(uint32_t mav_sys_id);

	void update_uav_info();
	bool uav_takeoff_altitude(matrix::Vector3f veicle_own_position, matrix::Vector3f target_position, matrix::Vector3f target_velocity,float takeoff_altitude,float target_yaw, float target_yawspeed);

	void handle_parameter_update();
	void handle_data_subscription();
	void handle_init_state();
	void handle_arm_auto_state();
	void handle_arm_leader_state();
	void handle_arm_offboard_state();
	void handle_control_state(vehicle_status_s _state,leader_info_s _leader_info);
	void handle_land_state();
	void handle_disarm_state(vehicle_status_s _state);
	void handle_idle_state(swarm_start_flag_s _start_flag);
	void handle_manual_takeover(vehicle_status_s _state);


	vtol_vehicle_status_s _vtol_vehicle_status;
	uint64_t init_gps_time;
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

	enum vtol_ctl_state {
		mc_takeoff = 0,
		mc_to_fw,
		fw_mission,
		fw_to_mc,
		mc_land,
		emerg
	};
	vtol_ctl_state VTOL_STATE = mc_takeoff;

	// float _last_leader_altitude; 

	float begin_x;
	float begin_y;
	float begin_z;
	float target_x;
	float target_y;
	int vehicle_id = 1;  // 将在init()中从MAV_SYS_ID参数获取
	vehicle_local_position_s _vehicle_local_position;
	sensor_gps_s _sensor_gps;
	int recived_point;
	trajectory_setpoint_s _trajectory_setpoint;

	leader_info_s _leader_sp_glo_pos{};
	leader_info_s _thisuav_leader_info{};
	swarm_start_flag_s _swarm_start_flag{};
	leader_id_s _leader_group_id{};
	vehicle_local_position_setpoint_s vehicle_local_position_setpoint{};

	MapProjection _global_local_proj_ref{};

	matrix::Vector3f leader_position;
	matrix::Vector3f _sp_position;
	matrix::Vector3f _sp_offset;
	matrix::Vector3f _sp_vel;
	float _sp_yaw;
	float _sp_yaw_speed;

	//  用于检测编队切换时的目标位置突变
	matrix::Vector3f _last_target_position{0.f, 0.f, 0.f};
	bool _last_target_valid{false};
	float _formation_switch_threshold = 2.0f;  // 目标位置突变阈值（米）

	//  路径规划相关
	matrix::Vector3f _current_waypoint{0.f, 0.f, 0.f};  // 当前航点（绕行点）
	bool _using_detour_waypoint{false};  // 是否正在使用绕行航点
	matrix::Vector3f _final_target{0.f, 0.f, 0.f};  // 最终目标（编队位置）

	bool _set_as_leader;
	int  _group_id;
	bool first_loop = 1;

	// 从机间位置共享管理器
	PositionSharing _position_sharing;

	//  APF避撞控制器（保留用于兼容）
	APFController _apf_controller;

	//  基于速度障碍的避撞控制器（主要使用）
	VelocityObstacleController _vo_controller;

	//  编队路径规划器
	FormationPlanner _formation_planner;

	
	//std::map<uint32_t, leader_info_s> uav_info_cache;
	//uav_cache_entry_s uav_info_cache[MAX_UAV_COUNT];

	int find_uav_index(uint32_t mavid);
    int find_free_slot();

	// 位置共享管理函数
	void update_position_sharing();
	// Subscriptions


	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules Swarm when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _leader_info_sub{ORB_ID(leader_info)};
	uORB::Subscription _follower_info_sub{ORB_ID(follower_info)};
	uORB::Subscription _swarm_start_flag_sub{ORB_ID(swarm_start_flag)};
	uORB::Subscription _leader_id_sub{ORB_ID(leader_id)};
	uORB::Subscription _leader_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};


	uORB::Publication<swarm_start_flag_s>			_swarm_start_pub{ORB_ID(swarm_start_flag)};
	uORB::Publication<position_setpoint_triplet_s>		_position_setpoint_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s>		_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<leader_id_s>   			_group_id_pub{ORB_ID(leader_id)};
	uORB::Publication<swarm_start_flag_s>			_start_flag_pub{ORB_ID(swarm_start_flag)};

	uORB::Publication<leader_info_s>			_leader_info_pub{ORB_ID(leader_info)};


	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		// System parameters
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,  /**< another parameter */

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
		(ParamInt<px4::params::SWARM_APF_LEADER>) _param_apf_enable_leader_avoidance
	)


	bool _armed{false};
};