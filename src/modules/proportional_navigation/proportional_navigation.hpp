/**
 * @file proportional_navigation.hpp
 * 基于PPN的碰撞角约束制导策略（PPNIACG）
 * Pure Proportional Navigation with Impact Angle Constraint Guidance
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/radar_target.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

class ProportionalNavigation : public ModuleBase<ProportionalNavigation>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	ProportionalNavigation();
	~ProportionalNavigation() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// 任务状态
	enum class MissionState {
		IDLE,
		TAKEOFF,       // 起飞阶段
		ENGAGING,      // 接敌/俯冲阶段
		GROUND_STRIKE, // 地面俯冲打击（新增）
		PULLUP,        // 拉升阶段
		COMPLETE
	};

	// 核心功能
	void updateState();
	void updateMissionState();
	void executeEngagement();
	void executeGroundStrike();  // 地面俯冲打击（新增）
	void executePullup();      // 拉升机动
	void executeTakeoff();     // 起飞机动
	void initializeEngagement();  // 初始化任务状态（计算r0, q0, theta0, q_dot0）

	// PPNIACG 核心算法
	matrix::Vector3f computePPNIACG();

	// 铅垂面内落角约束（垂直平面）
	float computeVerticalIACG(float r, float q, float theta, float q_f_desired);

	// 水平面内碰撞角约束
	float computeHorizontalIACG(float r, float q, float theta, float q_f_desired);

	// 辅助计算（二维极坐标系 - 使用解析解）
	float computeRange();                    // 距离 r
	float computeLOSAngle();                 // 视线角 q（在指定平面内）
	float computeLOSRate();                  // 视线角速率 q_dot（支持运动目标）
	float computeLOSRateMovingTarget();      // 运动目标的视线角速率（新增）
	float computeLeadAngle();                // 前置角 theta（公式18解析解）
	float computeRadialVelocity();           // 径向速度 v_r = dot_r
	float computeNormalVelocity();           // 法向速度 v_theta = r*q_dot
	float computeFinalLOSAngle(float q, float theta, float N);  // 终端视线角 q_f（公式28）
	float computeClosingVelocity();          // 接近速度（用于TPN）

	// 实时比例导引系数计算
	float computeAdaptiveN(float q, float theta, float q_f_desired);

	// 转入PPN条件判断
	bool shouldSwitchToPPN(float q, float q_f_desired, float N);

	// 速度和加速度生成
	void generateTrajectorySetpoint();
	void publishSetpoint();

	// 安全检查
	bool safetyCheck();
	void emergencyAbort();
	bool isAirborne();         // 检查是否在空中
	void sendTakeoffCommand(); // 发送起飞指令

	// 雷达数据处理（新增）
	void updateRadarTarget();           // 更新雷达目标
	void filterRadarData();             // 雷达数据滤波（简单α-β滤波）
	void handleRadarLoss();             // 雷达丢失处理（预测）
	bool isRadarDataValid();            // 检查雷达数据有效性

	// uORB订阅
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _radar_target_sub{ORB_ID(radar_target)};  // 订阅雷达数据
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};  // GPS位置
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};  // Home位置（用于坐标转换）

	// uORB发布
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

	// 状态变量
	MissionState _mission_state{MissionState::IDLE};
	hrt_abstime _mission_start_time{0};

	bool _auto_track_radar{false};  // 自动跟踪雷达目标
	uint8_t _radar_target_id{1};    // 跟踪的目标ID

	// 雷达数据滤波（新增）
	matrix::Vector3f _radar_filtered_position;   // 滤波后的位置
	matrix::Vector3f _radar_filtered_velocity;   // 滤波后的速度
	matrix::Vector3f _radar_predicted_position;  // 预测位置（雷达丢失时使用）
	matrix::Vector3f _radar_predicted_velocity;  // 预测速度
	hrt_abstime _last_radar_update_time{0};      // 上次雷达更新时间
	uint32_t _radar_loss_count{0};               // 雷达丢失计数
	float _radar_alpha{0.3f};                    // α-β滤波器参数α（位置）
	float _radar_beta{0.1f};                     // α-β滤波器参数β（速度）
	static constexpr uint32_t RADAR_LOSS_THRESHOLD = 10;  // 雷达丢失阈值（10帧）

	// 起飞相关
	bool _auto_takeoff{false};      // 是否自动起飞
	float _takeoff_altitude{20.0f}; // 起飞高度（米）
	hrt_abstime _takeoff_start_time{0};

	// 当前状态（NED坐标系）
	matrix::Vector3f _current_position;
	matrix::Vector3f _current_velocity;
	matrix::Quatf _current_attitude;

	// 目标状态（支持运动目标）
	matrix::Vector3f _target_position;
	matrix::Vector3f _target_velocity;  // 目标速度（新增）
	matrix::Vector3f _target_acceleration;  // 目标加速度（可选，用于APN）

	// 期望碰撞角（弧度）
	float _desired_impact_angle_vertical{0.0f};    // 铅垂面内落角（-90° = 垂直打击）
	float _desired_impact_angle_horizontal{0.0f};  // 水平面内碰撞角

	// 极坐标系状态变量
	float _r{0.0f};              // 距离
	float _q{0.0f};              // 视线角
	float _q_dot{0.0f};          // 视线角速率
	float _theta{0.0f};          // 前置角
	float _v_r{0.0f};            // 径向速度
	float _v_theta{0.0f};        // 法向速度
	float _q_f{0.0f};            // 终端视线角

	// 初始状态（用于解析解计算）
	float _r0{0.0f};
	float _q0{0.0f};
	float _q_dot0{0.0f};
	float _theta0{0.0f};

	// 控制输出
	matrix::Vector3f _commanded_acceleration;
	matrix::Vector3f _commanded_velocity;

	// 当前导引系数
	float _current_N{3.0f};
	bool _switched_to_ppn{false};

	// 拉升相关状态
	bool _enable_pullup{false};           // 是否启用拉升
	float _pullup_trigger_distance{5.0f}; // 拉升触发距离
	float _pullup_target_altitude{0.0f};  // 拉升目标高度
	float _pullup_acceleration{10.0f};    // 拉升加速度
	hrt_abstime _pullup_start_time{0};    // 拉升开始时间

	// 时间步长
	float _dt{0.01f};
	hrt_abstime _last_run_time{0};

	// GPS坐标转换
	MapProjection _map_projection{};  // 地图投影（GPS <-> NED）
	bool _map_projection_initialized{false};

	// GPS目标位置（度）
	double _target_lat{0.0};
	double _target_lon{0.0};
	float _target_alt{0.0f};  // AMSL高度（米）
	bool _use_gps_target{false};  // 是否使用GPS目标

	// 可调参数
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PNAV_GAIN>) _param_pn_nav_gain,           // 初始导引比 N
		(ParamFloat<px4::params::PN_MAX_ACCEL>) _param_pn_max_accel,         // 最大加速度
		(ParamFloat<px4::params::PN_MAX_VEL>) _param_pn_max_vel,             // 最大速度
		(ParamFloat<px4::params::PN_PULLUP_DIST>) _param_pn_pullup_dist,     // 拉升触发距离
		(ParamFloat<px4::params::PN_PULLUP_ALT>) _param_pn_pullup_alt,       // 拉升目标高度
		(ParamFloat<px4::params::PN_PULLUP_ACCEL>) _param_pn_pullup_accel,   // 拉升加速度
		(ParamInt<px4::params::PN_ENABLE_PULLUP>) _param_pn_enable_pullup,   // 启用拉升
		(ParamFloat<px4::params::PN_IMPACT_ANG_V>) _param_pn_impact_angle_v,  // 铅垂面落角（度）
		(ParamFloat<px4::params::PN_IMPACT_ANG_H>) _param_pn_impact_angle_h,  // 水平面碰撞角（度）
		(ParamFloat<px4::params::PN_MISS_DIST>) _param_pn_miss_dist,         // 命中距离
		(ParamInt<px4::params::PN_CONSTR_MODE>) _param_pn_constraint_mode,   // 约束模式
		(ParamInt<px4::params::PN_NO_VEL_LIMIT>) _param_pn_no_vel_limit      // 禁用速度限制
	);
};
