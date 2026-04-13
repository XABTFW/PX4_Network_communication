/**
 * @file dive_bombing_control.hpp
 * 四旋翼俯冲轰炸控制模块头文件
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
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

class DiveBombingControl : public ModuleBase<DiveBombingControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	DiveBombingControl();
	~DiveBombingControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// 飞行阶段枚举
	enum class FlightPhase {
		IDLE,
		DIVE,      // 直接俯冲到目标
		COMPLETE
	};

	// 轨迹点结构
	struct TrajectoryPoint {
		matrix::Vector3f position;
		matrix::Vector3f velocity;
		matrix::Vector3f acceleration;
		float yaw;
		float yaw_rate;
	};

	// 核心功能函数
	void updateState();
	void generateSimpleSetpoint();
	void publishSetpoint();
	
	// Offboard模式控制
	void publishOffboardControlMode();
	void switchToOffboardMode();
	void checkOffboardStatus();
	
	// 轨迹生成
	TrajectoryPoint computeQuinticPolynomial(float t, float T,
		const matrix::Vector3f &p0, const matrix::Vector3f &v0, const matrix::Vector3f &a0,
		const matrix::Vector3f &pf, const matrix::Vector3f &vf, const matrix::Vector3f &af);
	
	// 空气阻力计算
	float computeDragForce(float velocity);
	float computeTerminalVelocity();
	
	// 状态机
	void updateFlightPhase();
	void executeDivePhase();
	
	// 安全检查
	bool safetyCheck();
	void emergencyAbort();

	// uORB订阅
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// uORB发布
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

	// 状态变量
	FlightPhase _flight_phase{FlightPhase::IDLE};
	hrt_abstime _phase_start_time{0};
	
	// Offboard模式控制
	bool _offboard_mode_requested{false};
	bool _offboard_mode_active{false};
	hrt_abstime _offboard_start_time{0};
	uint32_t _offboard_heartbeat_counter{0};
	
	// 位置和姿态
	matrix::Vector3f _current_position;
	matrix::Vector3f _current_velocity;
	matrix::Quatf _current_attitude;
	
	// 目标参数
	matrix::Vector3f _target_position;      // 目标绝对位置
	matrix::Vector3f _home_position;        // 起始位置
	matrix::Vector3f _relative_target;      // 相对移动距离 (dx, dy, dz)
	
	// 轨迹参数
	TrajectoryPoint _current_setpoint;
	float _trajectory_time{0.0f};
	float _trajectory_duration{0.0f};
	
	// 可调参数
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DBC_INIT_ALT>) _param_dbc_init_alt,
		(ParamFloat<px4::params::DBC_REL_ALT>) _param_dbc_rel_alt,
		(ParamFloat<px4::params::DBC_DIVE_ANG>) _param_dbc_dive_ang,
		(ParamFloat<px4::params::DBC_MAX_VEL>) _param_dbc_max_vel,
		(ParamFloat<px4::params::DBC_PULLUP_ACC>) _param_dbc_pullup_acc,
		(ParamFloat<px4::params::DBC_DRAG_COEF>) _param_dbc_drag_coef,
		(ParamFloat<px4::params::DBC_MASS>) _param_dbc_mass
	);
	
	// 物理参数
	float _vehicle_mass{1.5f};        // kg
	float _drag_coefficient{0.6f};    // 无量纲
	float _frontal_area{0.1f};        // m²
	float _air_density{1.225f};       // kg/m³
};
