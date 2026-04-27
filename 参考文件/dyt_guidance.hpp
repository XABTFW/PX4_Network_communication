/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <math.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/dyt_command.h>
#include <uORB/topics/dyt_guidance_status.h>
#include <uORB/topics/dyt_target.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <matrix/matrix/math.hpp>

using namespace time_literals;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class DytGuidance : public ModuleBase<DytGuidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief 构造制导模块实例。
	 *
	 * 输入：无。
	 * 输出：返回一个内部状态已初始化的模块对象。
	 * 功能：将模块绑定到 PX4 工作队列，并清空缓存的目标数据。
	 */
	DytGuidance();

	/**
	 * @brief 析构制导模块实例。
	 *
	 * 输入：无。
	 * 输出：释放模块对象。
	 * 功能：使用默认析构函数，因为模块没有需要特殊清理的资源。
	 */
	~DytGuidance() override = default;

	/**
	 * @brief 在配置好的 PX4 工作队列上启动模块任务。
	 *
	 * @param argc 传给模块入口的命令行参数个数。
	 * @param argv 模块框架使用的命令行参数数组。
	 * @return 启动成功返回 PX4_OK，否则返回 PX4_ERROR。
	 * 功能：创建模块单例实例并启动周期调度执行。
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief 处理模块自定义命令行命令。
	 *
	 * @param argc 命令参数个数。
	 * @param argv 命令参数数组。
	 * @return 命令处理成功时返回 PX4_OK，否则返回 usage 的结果。
	 * 功能：分发运行期命令，例如状态打印和重新触发请求。
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @brief 打印模块用法说明。
	 *
	 * @param reason 可选的失败原因，会先于用法说明打印；传入 nullptr 表示不打印。
	 * @return 打印完用法说明后固定返回 0。
	 * 功能：输出命令行帮助和模块描述。
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief 初始化模块的运行时调度。
	 *
	 * 输入：无。
	 * @return 周期任务调度成功时返回 true。
	 * 功能：以固定周期启动控制循环。
	 */
	bool init();

	/**
	 * @brief 在 PX4 控制台打印当前模块状态。
	 *
	 * 输入：无。
	 * 输出：在 PX4 日志中输出可读的诊断信息。
	 * 功能：打印关键状态、目标有效性和 AUX 配置，便于调试。
	 */
	void show_status();

private:
	static constexpr int OBS_BUFFER_LEN{6};

	struct LosObservation {
		hrt_abstime sample_time{0};
		Vector3f los_body{};
		float frame_dt_s{0.f};
	};

	struct TrackProfile {
		float nav_gain{0.f};
		float v_cmd{0.f};
		float k_a{0.f};
		float k_v{0.f};
		uint8_t submode{dyt_guidance_status_s::SUBMODE_FOLLOW};
	};

	enum class TaskState : uint8_t {
		Idle = dyt_guidance_status_s::STATE_IDLE,
		SearchWaitLock = dyt_guidance_status_s::STATE_SEARCH_WAIT_LOCK,
		TrackFollow = dyt_guidance_status_s::STATE_TRACK_FOLLOW,
		TrackIntercept = dyt_guidance_status_s::STATE_TRACK_INTERCEPT,
		LostHold = dyt_guidance_status_s::STATE_LOST_HOLD,
		Abort = dyt_guidance_status_s::STATE_ABORT
	};

	/**
	 * @brief 执行一次周期调度的控制循环。
	 *
	 * 输入：最新订阅数据、参数值和内部状态。
	 * 输出：更新状态机并发布 offboard/status 话题。
	 * 功能：运行完整的制导状态机并输出控制设定值。
	 */
	void Run() override;

	/**
	 * @brief 刷新所有订阅的 uORB 话题缓存。
	 *
	 * 输入：订阅中待处理的新数据。
	 * 输出：更新缓存的飞行器、遥控和目标数据成员。
	 * 功能：在状态更新前将最新测量值拉入控制器。
	 */
	void update_subscriptions();

	/**
	 * @brief 在收到参数更新时重新加载模块参数。
	 *
	 * 输入：来自 uORB 的参数更新通知。
	 * 输出：刷新缓存的参数包装器数值。
	 * 功能：保证运行时行为与最新参数配置保持一致。
	 */
	void update_params_if_needed();

	/**
	 * @brief 读取某个遥控 AUX 通道的归一化数值。
	 *
	 * @param index AUX 通道编号，范围为 1..6。
	 * @return 索引有效时返回通道值，否则返回 NAN。
	 * 功能：将 PX4 的 AUX 编号映射到对应的遥控输入字段。
	 */
	float aux_value(int index) const;

	/**
	 * @brief 判断某个 AUX 开关当前是否处于激活状态。
	 *
	 * @param index AUX 通道编号，范围为 1..6。
	 * @return 当通道存在且其值高于激活阈值时返回 true。
	 * 功能：把原始 AUX 数值转换成布尔开关状态。
	 */
	bool aux_switch_active(int index) const;

	/**
	 * @brief 检查当前飞行器状态是否允许自主制导。
	 *
	 * 输入：缓存的解锁状态、失效保护状态和本地位置有效性状态。
	 * @return 当飞行器已解锁、未进入 failsafe 且本地位置有效时返回 true。
	 * 功能：用最基本的安全条件约束制导激活和持续运行。
	 */
	bool preconditions_ok() const;

	/**
	 * @brief 检测飞手杆量是否触发人工接管。
	 *
	 * 输入：缓存的遥控设定值和接管阈值参数。
	 * @return 任一被监控杆量超过配置阈值时返回 true。
	 * 功能：在飞手主动干预时终止制导。
	 */
	bool manual_takeover_detected() const;

	/**
	 * @brief 处理一帧新收到的目标样本。
	 *
	 * @param target 来自感知链路的最新目标消息。
	 * 输出：当目标样本有效时向 LOS 观测缓冲区追加一条记录。
	 * 功能：校验目标数据，并将其转换为观测缓冲区使用的表示形式。
	 */
	void handle_new_target(const dyt_target_s &target);

	/**
	 * @brief 将目标 LOS 角从云台坐标系转换到机体坐标系。
	 *
	 * @param target 包含 LOS 和云台姿态数据的目标消息。
	 * @param[out] los_body 转换成功时输出归一化后的机体系 LOS 向量。
	 * @return 坐标变换后 LOS 向量有限且有效时返回 true，否则返回 false。
	 * 功能：应用符号约定和角度零偏，生成机体系单位 LOS 向量。
	 */
	bool build_los_body(const dyt_target_s &target, Vector3f &los_body) const;

	/**
	 * @brief 向定长历史缓冲区追加一条 LOS 观测。
	 *
	 * @param los_body 本次样本对应的机体系 LOS 单位向量。
	 * @param sample_time 该样本对应的时间戳。
	 * @param frame_dt_s 感知源上报的帧间隔，单位为秒。
	 * 输出：更新观测缓冲区和最近一次 LOS 缓存。
	 * 功能：保留一小段历史数据，用于 LOS 预测和 LOS 角速度估计。
	 */
	void push_observation(const Vector3f &los_body, hrt_abstime sample_time, float frame_dt_s);

	/**
	 * @brief 重置 LOS 观测历史和滤波后的制导状态。
	 *
	 * 输入：无。
	 * 输出：清空缓冲样本、滤波 LOS 向量和派生角速度。
	 * 功能：将 LOS 估计器恢复到干净的初始状态。
	 */
	void clear_observations();

	/**
	 * @brief 更新滤波后的 NED 系 LOS 估计和 LOS 角速度。
	 *
	 * @param now 当前高精度时间戳，用于预测和求导。
	 * @return 成功生成有效 LOS 估计时返回 true，否则返回 false。
	 * 功能：外推最新机体系 LOS，将其旋转到 NED 系，并对结果进行低通滤波。
	 */
	bool update_los_estimate(hrt_abstime now);

	/**
	 * @brief 将当前飞行器位姿保存为悬停设定值。
	 *
	 * 输入：缓存的本地位置和姿态。
	 * 输出：把悬停位置和偏航参考保存到模块状态中。
	 * 功能：冻结当前位姿，供悬停和丢目标模式使用。
	 */
	void capture_hold_setpoint();

	/**
	 * @brief 发布位置悬停轨迹设定值。
	 *
	 * 输入：缓存的悬停位置和悬停偏航角。
	 * 输出：发布 `trajectory_setpoint` 消息，并更新设定值缓存。
	 * 功能：指令飞行器保持在捕获到的悬停位姿。
	 */
	void publish_hold_setpoint();

	/**
	 * @brief 发布由当前 LOS 解算结果得到的跟踪设定值。
	 *
	 * @param profile 当前跟踪模式使用的制导增益和指令速度。
	 * 输出：发布速度、加速度和偏航设定值。
	 * 功能：根据 LOS 几何关系和飞行器运动状态计算跟随或拦截指令。
	 */
	void publish_track_setpoint(const TrackProfile &profile);

	/**
	 * @brief 发布期望的 offboard 控制模式选择。
	 *
	 * @param hold_mode 为 true 时启用位置悬停控制，为 false 时启用速度跟踪控制。
	 * 输出：发布 `offboard_control_mode` 消息。
	 * 功能：通知 PX4 下一条轨迹设定值中哪些控制字段有效。
	 */
	void publish_offboard_mode(bool hold_mode);

	/**
	 * @brief 周期性请求 PX4 切换到 offboard 模式。
	 *
	 * 输入：缓存的飞行器模式、系统/组件 ID，以及距离上次请求的时间。
	 * 输出：未进入 offboard 前最多以 5 Hz 发布请求，进入后降为 1 Hz 保活。
	 * 功能：在制导控制器工作期间尽快切入并持续维持 offboard 模式。
	 */
	void request_offboard_mode();

	/**
	 * @brief 发布内部制导状态话题。
	 *
	 * 输入：当前状态机状态、LOS 估计和控制设定值。
	 * 输出：发布 `dyt_guidance_status` 消息。
	 * 功能：对外暴露控制器状态和输出，便于监控和调试。
	 */
	void publish_status();

	/**
	 * @brief 将模块状态机切换到新状态。
	 *
	 * @param new_state 目标任务状态。
	 * @param lost_reason 与本次状态切换相关的丢失原因码。
	 * 输出：更新状态跟踪成员，并执行状态进入时的清理逻辑。
	 * 功能：集中处理状态切换及其相关复位逻辑。
	 */
	void enter_state(TaskState new_state, uint8_t lost_reason = dyt_guidance_status_s::LOST_REASON_NONE);

	/**
	 * @brief 从空闲状态启动一次新的制导流程。
	 *
	 * 输入：当前前置条件状态和当前飞行器位姿。
	 * 输出：初始化控制器状态、记录悬停点并下发自动锁定命令。
	 * 功能：在确认激活条件满足后启动跟踪流程。
	 */
	void activate_guidance();

	/**
	 * @brief 中止当前激活的制导流程。
	 *
	 * @param lost_reason 本次中止记录的原因码。
	 * 输出：发送停止跟踪命令，并将状态机切换到中止处理状态。
	 * 功能：在操作员或系统条件不满足时终止自主制导。
	 */
		void abort_guidance(uint8_t lost_reason);

		/**
		 * @brief 进入临时丢目标保持状态。
	 *
	 * @param lost_reason 描述跟踪丢失原因的原因码。
	 * 输出：更新状态机，使飞行器在等待重新锁定时保持位置。
	 * 功能：在目标短暂丢失期间保持飞行器稳定。
		 */
		void enter_lost_hold(uint8_t lost_reason);

		/**
		 * @brief 在丢目标保持期间主动让吊舱回中并重触发锁定。
		 *
		 * @param now 当前高精度时间戳。
		 * 输出：按配置的回中等待和重试周期发送吊舱控制命令。
		 * 功能：适配丢锁后退出自动跟踪且停在边界角的吊舱行为。
		 */
		void update_lost_reacquire(hrt_abstime now);

	/**
	 * @brief 检查最新目标消息是否报告了有效锁定目标。
	 *
	 * 输入：缓存的最新目标样本。
	 * @return 当目标存在、有效且被跟踪器标记为 locked 时返回 true。
	 * 功能：区分可靠的锁定状态和普通的目标可见状态。
	 */
	bool target_locked() const;

	/**
	 * @brief 检查最新目标数据是否足够新鲜，可用于控制。
	 *
	 * 输入：缓存的目标时间戳、帧间隔和新鲜度参数。
	 * @return 当目标数据年龄和帧间隔都在配置限制内时返回 true。
	 * 功能：在制导使用前剔除陈旧、超时的跟踪数据。
	 */
	bool target_fresh() const;

	/**
	 * @brief 检查目标数据是否可用于主动跟踪。
	 *
	 * 输入：目标锁定状态、数据新鲜度状态和 LOS 观测可用性。
	 * @return 当目标已锁定、数据新鲜且至少存在一条 LOS 观测时返回 true。
	 * 功能：为跟随和拦截模式提供统一的可用性判断。
	 */
	bool target_usable() const;

	/**
	 * @brief 检查当前是否允许进入拦截模式。
	 *
	 * 输入：目标可用性、配置的时延限制、帧间隔和 LOS 几何关系。
	 * @return 当目标可用且几何/时延条件允许时返回 true。
	 * 功能：为更激进的拦截模式设置更严格的进入条件。
	 */
	bool intercept_allowed() const;

	/**
	 * @brief 构造跟随模式的制导参数组合。
	 *
	 * 输入：跟随模式的增益和速度参数。
	 * @return 返回填充完成的跟踪参数结构体，用于常规跟随行为。
	 * 功能：将跟随模式调参打包成一个统一结构，供设定值生成使用。
	 */
	TrackProfile follow_profile() const;

	/**
	 * @brief 构造拦截模式的制导参数组合。
	 *
	 * 输入：拦截模式的增益和速度参数。
	 * @return 返回填充完成的跟踪参数结构体，用于激进拦截行为。
	 * 功能：将拦截模式调参打包成一个统一结构，供设定值生成使用。
	 */
	TrackProfile intercept_profile() const;

	/**
	 * @brief 向 DYT 载荷/控制接口发布命令。
	 *
		 * @param command 要发送的命令码。
		 * @param param_x 可选的整型命令参数，默认值为 0。
		 * @param param_y 可选的第二整型命令参数，默认值为 0。
		 * 输出：发布 `dyt_command` 消息。
		 * 功能：发送自动锁定、停止跟踪、重新触发等跟踪器控制命令。
		 */
		void send_dyt_command(uint8_t command, int16_t param_x = 0, int16_t param_y = 0);

	TaskState _state{TaskState::Idle};
	uint8_t _lost_reason{dyt_guidance_status_s::LOST_REASON_NONE};
	uint8_t _requested_submode{dyt_guidance_status_s::SUBMODE_FOLLOW};

	bool _prev_activation_request{false};
	int _lock_streak{0};
	int _relock_streak{0};

		hrt_abstime _state_enter_time{0};
		hrt_abstime _last_offboard_request{0};
		hrt_abstime _last_retrigger_time{0};

	LosObservation _observations[OBS_BUFFER_LEN]{};
	int _observation_count{0};

	dyt_target_s _last_target{};
	bool _have_target{false};

	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_status_s _vehicle_status{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	manual_control_setpoint_s _manual_control{};

	Vector3f _hold_position{};
	float _hold_yaw{0.f};

	Vector3f _los_ned{};
	Vector3f _los_body_latest{};
	Vector3f _los_filtered{};
	Vector3f _prev_los_filtered{};
	Vector3f _omega_los{};
	Vector3f _velocity_sp{};
	Vector3f _acceleration_sp{};
	float _yaw_sp{NAN};
	float _yaw_rate_sp{NAN};
	hrt_abstime _prev_los_update{0};
	bool _los_filter_initialized{false};

	uORB::Subscription _dyt_target_sub{ORB_ID(dyt_target)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<dyt_command_s> _dyt_command_pub{ORB_ID(dyt_command)};
	uORB::Publication<dyt_guidance_status_s> _dyt_guidance_status_pub{ORB_ID(dyt_guidance_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DYTG_ACT_AUX>) _param_act_aux,
		(ParamInt<px4::params::DYTG_INT_AUX>) _param_int_aux,
		(ParamFloat<px4::params::DYTG_STK_TK>) _param_stick_takeover,
		(ParamInt<px4::params::DYTG_LOCK_N>) _param_lock_frames,
		(ParamInt<px4::params::DYTG_RELOCKN>) _param_relock_frames,
			(ParamInt<px4::params::DYTG_WAITMS>) _param_wait_ms,
			(ParamInt<px4::params::DYTG_LOSTMS>) _param_lost_ms,
			(ParamInt<px4::params::DYTG_CTRMS>) _param_center_ms,
			(ParamInt<px4::params::DYTG_RTRYMS>) _param_retry_ms,
			(ParamFloat<px4::params::DYTG_DLY_MS>) _param_delay_ms,
		(ParamFloat<px4::params::DYTG_MAXAGE>) _param_max_age,
		(ParamFloat<px4::params::DYTG_MAXJIT>) _param_max_gap,
		(ParamFloat<px4::params::DYTG_INTDLY>) _param_intercept_delay,
		(ParamFloat<px4::params::DYTG_N_FOL>) _param_n_follow,
		(ParamFloat<px4::params::DYTG_V_FOL>) _param_v_follow,
		(ParamFloat<px4::params::DYTG_KA_FOL>) _param_ka_follow,
		(ParamFloat<px4::params::DYTG_KV_FOL>) _param_kv_follow,
		(ParamFloat<px4::params::DYTG_N_INT>) _param_n_intercept,
		(ParamFloat<px4::params::DYTG_V_INT>) _param_v_intercept,
		(ParamFloat<px4::params::DYTG_KA_INT>) _param_ka_intercept,
		(ParamFloat<px4::params::DYTG_KV_INT>) _param_kv_intercept,
		(ParamFloat<px4::params::DYTG_VMIN>) _param_vmin,
		(ParamFloat<px4::params::DYTG_MAXV>) _param_max_vel,
		(ParamFloat<px4::params::DYTG_MAXACC>) _param_max_acc,
		(ParamFloat<px4::params::DYTG_MAXYAWR>) _param_max_yaw_rate_deg,
		(ParamFloat<px4::params::DYTG_YAWLIM>) _param_yaw_limit_deg,
		(ParamFloat<px4::params::DYTG_MAXDZ>) _param_max_dz,
		(ParamFloat<px4::params::DYTG_ZSCALE>) _param_z_scale,
		(ParamFloat<px4::params::DYTG_FCONE>) _param_front_cone_deg,
		(ParamFloat<px4::params::DYTG_LPF_A>) _param_lpf_alpha,
		(ParamFloat<px4::params::DYTG_PREDMAX>) _param_pred_max,
		(ParamInt<px4::params::DYTG_LXSIGN>) _param_los_x_sign,
		(ParamInt<px4::params::DYTG_LYSIGN>) _param_los_y_sign,
		(ParamInt<px4::params::DYTG_RSIGN>) _param_roll_sign,
		(ParamInt<px4::params::DYTG_PSIGN>) _param_pitch_sign,
		(ParamInt<px4::params::DYTG_YSIGN>) _param_yaw_sign,
		(ParamFloat<px4::params::DYTG_ROFF>) _param_roll_off_deg,
		(ParamFloat<px4::params::DYTG_POFF>) _param_pitch_off_deg,
		(ParamFloat<px4::params::DYTG_YOFF>) _param_yaw_off_deg
	);
};
