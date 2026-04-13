/**
 * @file dive_bombing_control.cpp
 * 四旋翼俯冲轰炸控制模块实现
 */

#include "dive_bombing_control.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

DiveBombingControl::DiveBombingControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化目标位置为零
	_target_position.zero();
	_home_position.zero();
	_current_position.zero();
	_current_velocity.zero();
}

DiveBombingControl::~DiveBombingControl()
{
}

bool DiveBombingControl::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void DiveBombingControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// 更新状态
	updateState();
	
	// 如果任务激活，执行任务（在Position模式下工作）
	if (_flight_phase != FlightPhase::IDLE) {
		// 安全检查
		if (!safetyCheck()) {
			emergencyAbort();
			return;
		}
		
		// 状态机更新
		updateFlightPhase();
		
		// 生成简单的位置设定点
		generateSimpleSetpoint();
		
		// 发布设定点
		publishSetpoint();
	}
}

void DiveBombingControl::updateState()
{
	// 更新本地位置
	vehicle_local_position_s local_pos;
	if (_vehicle_local_position_sub.update(&local_pos)) {
		_current_position(0) = local_pos.x;
		_current_position(1) = local_pos.y;
		_current_position(2) = local_pos.z;
		
		_current_velocity(0) = local_pos.vx;
		_current_velocity(1) = local_pos.vy;
		_current_velocity(2) = local_pos.vz;
	}
	
	// 更新姿态
	vehicle_attitude_s attitude;
	if (_vehicle_attitude_sub.update(&attitude)) {
		_current_attitude = matrix::Quatf(attitude.q);
	}
}

void DiveBombingControl::publishOffboardControlMode()
{
	offboard_control_mode_s offboard_mode{};
	offboard_mode.timestamp = hrt_absolute_time();
	
	// 使用速度控制模式（关键改变！）
	offboard_mode.position = false;
	offboard_mode.velocity = true;  // 改为速度控制
	offboard_mode.acceleration = false;
	offboard_mode.attitude = false;
	offboard_mode.body_rate = false;
	offboard_mode.thrust_and_torque = false;
	offboard_mode.direct_actuator = false;
	
	_offboard_control_mode_pub.publish(offboard_mode);
	_offboard_heartbeat_counter++;
}

void DiveBombingControl::switchToOffboardMode()
{
	// 发送切换到Offboard模式的命令
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.0f; // 自定义模式
	cmd.param2 = 6.0f; // Offboard模式
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.from_external = true;
	
	_vehicle_command_pub.publish(cmd);
	
}

void DiveBombingControl::checkOffboardStatus()
{
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status)) {
		// 检查是否已经进入Offboard模式
		// nav_state == 14 表示 NAVIGATION_STATE_OFFBOARD
		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			if (!_offboard_mode_active) {
				_offboard_mode_active = true;
			}
		} else {
			if (_offboard_mode_active) {
				_offboard_mode_active = false;
			}
		}
	}
}

void DiveBombingControl::updateFlightPhase()
{
	switch (_flight_phase) {
	case FlightPhase::IDLE:
		// 等待启动命令
		break;
		
	case FlightPhase::DIVE:
		executeDivePhase();
		break;
		
	case FlightPhase::COMPLETE:
		_flight_phase = FlightPhase::IDLE;
		break;
		
	default:
		break;
	}
}

void DiveBombingControl::executeDivePhase()
{
	// 计算到目标的距离
	float distance_to_target = (_current_position - _target_position).norm();
	
	// 到达目标点（2米容差）
	if (distance_to_target < 2.0f) {
		_flight_phase = FlightPhase::COMPLETE;
		_phase_start_time = hrt_absolute_time();
		_trajectory_time = 0.0f;
	}
}

void DiveBombingControl::generateSimpleSetpoint()
{
	// 只在俯冲阶段生成设定点
	if (_flight_phase != FlightPhase::DIVE) {
		return;
	}
	
	// 计算到目标的方向向量
	matrix::Vector3f direction = _target_position - _current_position;
	float distance = direction.norm();
	
	if (distance < 2.0f) {
		// 接近目标，停止
		_current_setpoint.velocity.zero();
		_current_setpoint.yaw = atan2f(direction(1), direction(0));
		_current_setpoint.yaw_rate = 0.0f;
		_flight_phase = FlightPhase::COMPLETE;
		return;
	}
	
	// 归一化方向向量
	direction = direction.normalized();
	
	// 俯冲速度：恒定速度，像导弹一样直冲
	float dive_speed = 8.0f;  // 8 m/s 恒定速度
	
	// 速度向量：沿着目标方向
	matrix::Vector3f desired_velocity = direction * dive_speed;
	
	// 只设置速度，不设置位置（这是关键！）
	_current_setpoint.velocity = desired_velocity;
	
	// 位置设为NaN（速度控制模式下不需要位置）
	_current_setpoint.position(0) = NAN;
	_current_setpoint.position(1) = NAN;
	_current_setpoint.position(2) = NAN;
	
	// 加速度设为NaN
	_current_setpoint.acceleration(0) = NAN;
	_current_setpoint.acceleration(1) = NAN;
	_current_setpoint.acceleration(2) = NAN;
	
	// 偏航角指向目标（水平方向）
	_current_setpoint.yaw = atan2f(direction(1), direction(0));
	_current_setpoint.yaw_rate = 0.0f;
}

DiveBombingControl::TrajectoryPoint DiveBombingControl::computeQuinticPolynomial(
	float t, float T,
	const matrix::Vector3f &p0, const matrix::Vector3f &v0, const matrix::Vector3f &a0,
	const matrix::Vector3f &pf, const matrix::Vector3f &vf, const matrix::Vector3f &af)
{
	TrajectoryPoint point;
	
	if (T <= 0.0f) {
		point.position = pf;
		point.velocity.zero();
		point.acceleration.zero();
		return point;
	}
	
	// 五次多项式系数计算
	float t2 = t * t;
	float t3 = t2 * t;
	float t4 = t3 * t;
	float t5 = t4 * t;
	
	float T2 = T * T;
	float T3 = T2 * T;
	float T4 = T3 * T;
	float T5 = T4 * T;
	
	// 系数矩阵求解
	matrix::Vector3f a0_coef = p0;
	matrix::Vector3f a1_coef = v0;
	matrix::Vector3f a2_coef = a0 * 0.5f;
	matrix::Vector3f a3_coef = (20.0f * pf - 20.0f * p0 - (8.0f * vf + 12.0f * v0) * T - 
	                            (3.0f * af - a0) * T2) / (2.0f * T3);
	matrix::Vector3f a4_coef = (30.0f * p0 - 30.0f * pf + (14.0f * vf + 16.0f * v0) * T + 
	                            (3.0f * af - 2.0f * a0) * T2) / (2.0f * T4);
	matrix::Vector3f a5_coef = (12.0f * pf - 12.0f * p0 - (6.0f * vf + 6.0f * v0) * T - 
	                            (af - a0) * T2) / (2.0f * T5);
	
	// 计算位置
	point.position = a0_coef + a1_coef * t + a2_coef * t2 + a3_coef * t3 + a4_coef * t4 + a5_coef * t5;
	
	// 计算速度
	point.velocity = a1_coef + 2.0f * a2_coef * t + 3.0f * a3_coef * t2 + 4.0f * a4_coef * t3 + 5.0f * a5_coef * t4;
	
	// 计算加速度
	point.acceleration = 2.0f * a2_coef + 6.0f * a3_coef * t + 12.0f * a4_coef * t2 + 20.0f * a5_coef * t3;
	
	// 计算偏航角（指向目标）
	point.yaw = atan2f(_target_position(1) - point.position(1), 
	                   _target_position(0) - point.position(0));
	point.yaw_rate = 0.0f;
	
	return point;
}

float DiveBombingControl::computeDragForce(float velocity)
{
	// F_drag = 0.5 * ρ * v² * C_d * A
	float drag_coef = 0.6f;
	return 0.5f * _air_density * velocity * velocity * drag_coef * _frontal_area;
}

float DiveBombingControl::computeTerminalVelocity()
{
	// v_terminal = sqrt(2 * m * g / (ρ * C_d * A))
	float mass = 1.5f;
	float drag_coef = 0.6f;
	float g = 9.81f;
	
	return sqrtf(2.0f * mass * g / (_air_density * drag_coef * _frontal_area));
}

void DiveBombingControl::publishSetpoint()
{
	// 只在任务激活时发布设定点
	if (_flight_phase == FlightPhase::IDLE) {
		return;
	}
	
	trajectory_setpoint_s setpoint{};
	
	setpoint.timestamp = hrt_absolute_time();
	
	// 位置设定点（速度控制模式下设为NaN）
	setpoint.position[0] = NAN;
	setpoint.position[1] = NAN;
	setpoint.position[2] = NAN;
	
	// 速度设定点（关键：这是唯一的控制输入）
	_current_setpoint.velocity.copyTo(setpoint.velocity);
	
	// 加速度设定点（设为NaN）
	setpoint.acceleration[0] = NAN;
	setpoint.acceleration[1] = NAN;
	setpoint.acceleration[2] = NAN;
	
	// 偏航设定点
	setpoint.yaw = _current_setpoint.yaw;
	setpoint.yawspeed = NAN;
	
	_trajectory_setpoint_pub.publish(setpoint);
}

bool DiveBombingControl::safetyCheck()
{
	// 移除最小高度限制，允许俯冲到地面
	// 始终返回true，允许继续执行
	return true;
}

void DiveBombingControl::emergencyAbort()
{
	
	// 切换到悬停模式
	_flight_phase = FlightPhase::IDLE;
	
	// 发送紧急停止命令
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE;
	cmd.param1 = 0.0f; // 暂停
	cmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(cmd);
}

int DiveBombingControl::task_spawn(int argc, char *argv[])
{
	DiveBombingControl *instance = new DiveBombingControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int DiveBombingControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return 1;
	}

	if (!strcmp(argv[0], "execute")) {
		if (argc >= 4) {
			// 检查是否有任务正在执行
			if (get_instance()->_flight_phase != FlightPhase::IDLE) {
				return 1;
			}
			
			// 参数：dx dy dz（相对移动距离）
			float dx = atof(argv[1]);  // X方向移动距离（米）
			float dy = atof(argv[2]);  // Y方向移动距离（米）
			float dz = atof(argv[3]);  // Z方向下降距离（米，正值表示下降）
			
			// 保存当前位置作为起始点
			get_instance()->_home_position = get_instance()->_current_position;
			
			// 计算目标绝对位置（从当前位置开始）
			get_instance()->_target_position(0) = get_instance()->_current_position(0) + dx;
			get_instance()->_target_position(1) = get_instance()->_current_position(1) + dy;
			get_instance()->_target_position(2) = get_instance()->_current_position(2) + dz;
			
			
			// 直接开始俯冲，像炮弹一样！
			get_instance()->_flight_phase = FlightPhase::DIVE;
			get_instance()->_phase_start_time = hrt_absolute_time();
			get_instance()->_trajectory_time = 0.0f;
			
			PX4_INFO("Diving like a cannonball to target!");
			
			return 0;
		} else {
			return 1;
		}
	}

	if (!strcmp(argv[0], "strike")) {
		if (argc >= 4) {
			// 检查是否有任务正在执行
			if (get_instance()->_flight_phase != FlightPhase::IDLE) {
				return 1;
			}
			
			// 参数：x y z（绝对坐标，NED坐标系）
			float target_x = atof(argv[1]);  // 北向坐标（米）
			float target_y = atof(argv[2]);  // 东向坐标（米）
			float target_z = atof(argv[3]);  // 下向坐标（米，负值表示高度）
			
			// 保存当前位置作为起始点
			get_instance()->_home_position = get_instance()->_current_position;
			
			// 设置目标绝对位置
			get_instance()->_target_position(0) = target_x;
			get_instance()->_target_position(1) = target_y;
			get_instance()->_target_position(2) = target_z;
			
			// 直接开始俯冲
			get_instance()->_flight_phase = FlightPhase::DIVE;
			get_instance()->_phase_start_time = hrt_absolute_time();
			get_instance()->_trajectory_time = 0.0f;
			
			return 0;
		} else {
			return 1;
		}
	}

	return print_usage("unknown command");
}

int DiveBombingControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dive bombing control module for quadcopter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dive_bombing_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("execute", "Execute dive bombing with relative movement");
	PRINT_MODULE_USAGE_ARG("<dx> <dy> <dz>", "Relative movement (meters)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("strike", "Missile strike to absolute position");
	PRINT_MODULE_USAGE_ARG("<x> <y> <z>", "Absolute position in NED frame (meters)", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dive_bombing_control_main(int argc, char *argv[])
{
	return DiveBombingControl::main(argc, argv);
}
