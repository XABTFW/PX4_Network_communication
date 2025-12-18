#include "swarm_node.hpp"

using namespace matrix;

Swarm_Node::Swarm_Node() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_sp_position(0.f, 0.f, 0.f),
	_sp_vel(0.f, 0.f, 0.f),
	_sp_yaw(0.f),
	_sp_yaw_speed(0.f)
{
}

Swarm_Node::~Swarm_Node()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


void
Swarm_Node::set_swarm_offset(const matrix::Vector3f &sp_offset)
{
	_sp_offset = sp_offset;
}

void
Swarm_Node::set_swarm_info(const int &set_as_leader, const int &group_id)
{
	_set_as_leader = set_as_leader;
	_group_id = group_id;


	if (_set_as_leader) {
		_leader_group_id.group_id = _group_id;
		_leader_group_id.leader = 1;

	} else {
		_leader_group_id.group_id = _group_id;
		_leader_group_id.leader = 0;

	}

	_group_id_pub.publish(_leader_group_id);
}


bool
Swarm_Node::init()
{
	ScheduleOnInterval(20000_us); // 50 Hz rate
	return true;
}


bool Swarm_Node::swarm_node_init()
{

    if (_init_done) {
        return true;
    }

    // 获取MAV_SYS_ID参数，这样启动命令-i 2就会让vehicle_id=2
    int32_t mav_sys_id = -1;
    param_t param_id = param_find("MAV_SYS_ID");
    if (param_id != PARAM_INVALID) {
        param_get(param_id, &mav_sys_id);
        vehicle_id = mav_sys_id;
    } else {
        PX4_WARN("无法获取MAV_SYS_ID参数，使用默认vehicle_id=%d", vehicle_id);
    }

    _last_sp_yaw = NAN;

    if (_vehicle_local_position_sub.updated()) {
        _vehicle_local_position_sub.copy(&_vehicle_local_position);
    }

	_swarm_start_flag_sub.copy(&_swarm_start_flag);

    // 设置组号、偏移量等
    set_swarm_info(_param_swarm_set_leader.get(), _param_swarm_group_id.get());
    set_swarm_offset(Vector3f(_param_swarm_X_offset.get(), _param_swarm_Y_offset.get(), _param_swarm_Z_offset.get()));

    bool swarm_active = (_swarm_start_flag.start_swarm || _swarm_start_flag.start_swarm_auto);


    if ((_vehicle_local_position.xy_valid) && swarm_active) {
        // 如果是 leader 或 follower
        begin_x = _vehicle_local_position.x;
        begin_y = _vehicle_local_position.y;
        begin_z = _vehicle_local_position.z;

        // 如果是 leader 或 follower 都需要初始化参考坐标
        if (!_map_ref_initialized) {
            _global_local_proj_ref.initReference(_vehicle_local_position.ref_lat, _vehicle_local_position.ref_lon, hrt_absolute_time());
            _map_ref_initialized = true;
        }

        // Follower 情况下，投影到目标位置
        if (!_set_as_leader) {
            _global_local_proj_ref.project(_leader_sp_glo_pos.lat, _leader_sp_glo_pos.lon, target_x, target_y);
        }

        _init_done = true;
        return true;
    }
    return false;
}



void Swarm_Node::start_swarm_node()
{
    if (!_set_as_leader) {
        // Update leader information and check for stop command
        update_leader_info();

        if (_swarm_start_flag.stop_swarm) {
            STATE = state::LAND;
            return;
        }

        // Calculate target position from leader GPS coordinates
        calculate_target_position();

        // Check for formation switching
        bool formation_switching = check_formation_switching();

        // Get current position and calculate basic parameters
        matrix::Vector3f current_pos(_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
        matrix::Vector3f to_target = _final_target - current_pos;
        to_target(2) = 0.0f;
        float distance_to_target = to_target.norm();
        float current_speed_xy = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
                                       _vehicle_local_position.vy * _vehicle_local_position.vy);

        // Update position sharing
        bool at_target_position = (distance_to_target < 2.0f) && (current_speed_xy < 0.5f);
        int32_t selected_leader_id = _param_swarm_leader_id.get();
        _position_sharing.update_other_positions(vehicle_id, _global_local_proj_ref, selected_leader_id);
        _position_sharing.publish_position(vehicle_id, _vehicle_local_position, _sensor_gps, at_target_position);
        const OtherVehiclePosition* other_aircraft = _position_sharing.get_other_vehicles();

        // Run path planning if needed
        run_path_planning(current_pos, other_aircraft, formation_switching, distance_to_target);

        // Set basic control parameters
        _sp_vel(0) = _leader_sp_glo_pos.vx;
        _sp_vel(1) = _leader_sp_glo_pos.vy;
        _sp_vel(2) = _leader_sp_glo_pos.vz;
        _sp_yaw = _leader_sp_glo_pos.yaw;
        _sp_yaw_speed = _leader_sp_glo_pos.yawspeed;

        // YAW unwrap
        if (PX4_ISFINITE(_last_sp_yaw)) {
            float yaw_diff = _sp_yaw - _last_sp_yaw;
            if (yaw_diff > M_PI_F) {
                _sp_yaw -= 2.f * M_PI_F;
            } else if (yaw_diff < -M_PI_F) {
                _sp_yaw += 2.f * M_PI_F;
            }
        }
        _last_sp_yaw = _sp_yaw;

        // Check takeoff phase
        float takeoff_altitude = _param_take_altitude.get();
        bool in_takeoff_phase = (current_pos(2) > -takeoff_altitude);

        if (in_takeoff_phase) {
            static uint64_t last_takeoff_log = 0;
            uint64_t now = hrt_absolute_time();
            if (now - last_takeoff_log > 2000000) {
                PX4_INFO("[起飞阶段] 从机%d: 当前高度%.2fm 目标高度%.2fm 避撞已禁用",
                         vehicle_id, (double)(-current_pos(2)), (double)takeoff_altitude);
                last_takeoff_log = now;
            }
        }

        // Run collision avoidance - 只有未到达目标位置的飞机才进行避撞
        // at_target_position 已经在前面计算过了：(distance_to_target < 2.0f) && (current_speed_xy < 0.5f)
        bool need_avoidance = formation_switching || (distance_to_target > 3.0f);
        bool enable_avoidance_check = _param_apf_enable.get() > 0.5f && !in_takeoff_phase && need_avoidance && !at_target_position;
        bool collision_risk = false;
        bool critical_collision_risk = false;
        matrix::Vector3f position_correction(0.0f, 0.0f, 0.0f);

        if (enable_avoidance_check) {
            run_collision_avoidance(current_pos, other_aircraft, current_speed_xy, need_avoidance,
                                   in_takeoff_phase, collision_risk, critical_collision_risk, position_correction);
        }

        // Calculate filter alpha and apply filtering
        float alpha = calculate_filter_alpha(formation_switching, collision_risk, critical_collision_risk,
                                           current_speed_xy, enable_avoidance_check, at_target_position);

        matrix::Vector3f desired_position = _sp_position + _sp_offset;
        if (!_filter_initialized) {
            _sp_position_filtered = desired_position;
            _sp_vel_filtered = _sp_vel;
            _filter_initialized = true;
        } else {
            float pos_alpha = alpha;
            if (formation_switching && (!collision_risk || !enable_avoidance_check)) {
                pos_alpha = 0.15f;
            }
            _sp_position_filtered = pos_alpha * desired_position + (1.f - pos_alpha) * _sp_position_filtered;
            _sp_vel_filtered = alpha * _sp_vel + (1.f - alpha) * _sp_vel_filtered;
        }

        // Handle takeoff altitude control
        if (uav_takeoff_altitude(current_pos, _sp_position_filtered, _sp_vel_filtered,
                                 takeoff_altitude, _sp_yaw, _sp_yaw_speed)) {
            return;
        }

        // Apply avoidance corrections if needed
        if (enable_avoidance_check && collision_risk) {
            bool is_high_speed = current_speed_xy > 2.0f;
            apply_avoidance_correction(current_pos, other_aircraft, current_speed_xy, is_high_speed, position_correction);
        }

        // Calculate final velocity
        matrix::Vector3f final_vel = calculate_final_velocity(formation_switching, collision_risk,
                                                            enable_avoidance_check, current_speed_xy, at_target_position);

        // Send control command
        control_instance::getInstance()->Control_pos_vel_yaw(_sp_position_filtered, final_vel, _sp_yaw, _sp_yaw_speed);
    }
}

	void Swarm_Node::Run()
	{
	    static int iteration_count_Run = 0;
	    iteration_count_Run++;

	    if (should_exit()) {
	        ScheduleClear();
	        exit_and_cleanup();
	        return;
	    }

	    perf_begin(_loop_perf);
	    perf_count(_loop_interval_perf);

	    // 处理参数更新
	    handle_parameter_update();

	    // 处理数据订阅
	    swarm_start_flag_s _start_flag{};
	    vehicle_status_s _state{};
	    leader_info_s _leader_info{};

	    _swarm_start_flag_sub.copy(&_start_flag);
	    _vehicle_status_sub.copy(&_state);

	    // 根据当前状态进行处理
	    switch (STATE) {
	    case state::INIT:
	        handle_init_state();
	        break;

	    case state::ARM_AUTO:
	        handle_arm_auto_state();
	        break;

	    case state::ARM_LEADER:
	        handle_arm_leader_state();
	        break;

	    case state::ARM_OFFBOARD:
	        handle_arm_offboard_state();
	        break;

	    case state::CONTROL:
	        handle_control_state(_state, _leader_info);
	        break;

	    case state::LAND:
	        handle_land_state();
	        break;

	    case state::DISARM:
	        handle_disarm_state(_state);
	        break;

	    case state::IDLE:
	        handle_idle_state(_start_flag);
	        break;

	    default:
	        PX4_WARN("Unknown state: %d", STATE);
	        break;
	    }

	    perf_end(_loop_perf);
	}

	// 处理参数更新
	void Swarm_Node::handle_parameter_update()
	{
	    if (_parameter_update_sub.updated()) {
	        parameter_update_s param_update;
	        _parameter_update_sub.copy(&param_update);

	        //  保存旧的_set_as_leader值，用于检测主机切换
	        bool old_set_as_leader = _set_as_leader;

	        updateParams();

	        //  关键：参数更新后，重新设置swarm信息（包括_set_as_leader和_group_id）
	        // 这样当QGC动态改变主机选择时，follower能够及时响应
	        set_swarm_info(_param_swarm_set_leader.get(), _param_swarm_group_id.get());

	        //  关键修复：如果主机角色发生变化（从机变主机或主机变从机），需要重新初始化状态机
	        if (old_set_as_leader != _set_as_leader) {
	            PX4_WARN("[主机切换] 飞机%d: 角色从%s切换为%s，重新初始化状态机",
	                     vehicle_id, old_set_as_leader ? "主机" : "从机", _set_as_leader ? "主机" : "从机");

	            // 重置初始化标志，强制重新初始化
	            _init_done = false;

	            // 重置状态机到INIT状态，让它重新根据新的角色进入正确的状态
	            STATE = state::INIT;

	            // 重新设置偏移量
	            set_swarm_offset(Vector3f(_param_swarm_X_offset.get(), _param_swarm_Y_offset.get(), _param_swarm_Z_offset.get()));
	            _offset_initialized = false;
	        } else {
	            // 角色未变化，只重新设置偏移量
	            set_swarm_offset(Vector3f(_param_swarm_X_offset.get(), _param_swarm_Y_offset.get(), _param_swarm_Z_offset.get()));
	            _offset_initialized = false;
	        }

	        // 调试输出：显示参数更新
	        static uint64_t last_param_log = 0;
	        uint64_t now = hrt_absolute_time();
	        if (now - last_param_log > 2000000) {  // 每2秒最多输出一次
	            PX4_INFO("[参数更新] 飞机%d: SWARM_SET_LEADER=%d, SWARM_GROUP_ID=%d, SWARM_LEADER_ID=%d",
	                     vehicle_id, _param_swarm_set_leader.get(), _param_swarm_group_id.get(), _param_swarm_leader_id.get());
	            last_param_log = now;
	        }
	    }
	}


	// 处理 INIT 状态
	void Swarm_Node::handle_init_state()
	{
	    if (swarm_node_init()) {
	        if (!_set_as_leader) {
	            STATE = state::ARM_OFFBOARD;
	            PX4_INFO("STATE = state::ARM_OFFBOARD;");
	        } else if (_set_as_leader) {
	            if (_swarm_start_flag.start_swarm_auto) {
	                STATE = state::ARM_AUTO;
	                PX4_INFO("STATE = state::ARM_AUTO;");
	            } else if (_swarm_start_flag.start_swarm) {
	                STATE = state::ARM_LEADER;
	                PX4_INFO("STATE = state::ARM_LEADER;");
	            }
	        }
	    }
	}

	// 处理 ARM_AUTO 状态
	void Swarm_Node::handle_arm_auto_state()
	{
	    if (control_instance::getInstance()->Change_Auto() && control_instance::getInstance()->Arm_vehicle()) {
	        STATE = state::CONTROL;
	    } else {
	        STATE = state::IDLE;
	    }
	}

	// 处理 ARM_LEADER 状态
	void Swarm_Node::handle_arm_leader_state()
	{
	    if (control_instance::getInstance()->Change_position_mode() && control_instance::getInstance()->Arm_vehicle()) {
	        STATE = state::CONTROL;
	    } else {
	        STATE = state::IDLE;
	    }
	}

	// 处理 ARM_OFFBOARD 状态
	void Swarm_Node::handle_arm_offboard_state()
	{
	    if (control_instance::getInstance()->Change_offborad() && control_instance::getInstance()->Arm_vehicle()) {
	        STATE = state::CONTROL;
	        PX4_INFO("STATE = state::ARM_OFFBOARD;->state::CONTROL;");
	    } else {
	        STATE = state::IDLE;
	        PX4_INFO("STATE = state::ARM_OFFBOARD;->state::IDLE;");
	    }
	}

	// 处理 CONTROL 状态
	void Swarm_Node::handle_control_state(vehicle_status_s _state, leader_info_s _leader_info)
	{
		//  主机：只发布位置信息，不执行跟随逻辑
		if (_set_as_leader) {
			// 主机：发布自己的位置信息，让从机能够接收到
			vehicle_local_position_s vehicle_local_pos{};
			sensor_gps_s sensor_gps{};

			if (_vehicle_local_position_sub.copy(&vehicle_local_pos) && _sensor_gps_sub.copy(&sensor_gps)) {
				// 主机始终设置at_target=false，因为主机不需要避让其他飞机
				_position_sharing.publish_position(vehicle_id, vehicle_local_pos, sensor_gps, false);

				// 调试输出（每2秒一次）
				static uint64_t last_leader_publish_log = 0;
				uint64_t now = hrt_absolute_time();
				if (now - last_leader_publish_log > 2000000) {
					PX4_INFO("[主机发布] 主机%d: 发布位置信息 GPS(%.6f, %.6f) 给从机",
					         vehicle_id, (double)sensor_gps.latitude_deg, (double)sensor_gps.longitude_deg);
					last_leader_publish_log = now;
				}
			}

			//  关键：主机只需要发布位置信息，不需要执行跟随逻辑
			// 主机的控制完全由PX4的任务系统（AUTO模式）或用户手动控制
			// 不调用start_swarm_node()，避免干扰主机控制
			return;  // 主机提前返回，不执行后续的跟随控制逻辑
		}

		 if (_leader_info.land == 1 ) {
        // 主机进入 MISSION 模式且准备降落，从机也应该进入降落

        // 同步从机目标高度为主机当前高度
        // 高度处理：主机发送的是 MSL 高度，需要转换为本地 NED z 坐标
        float leader_land_z = _vehicle_local_position.ref_alt - static_cast<float>(_leader_sp_glo_pos.alt);
        _sp_position(2) = leader_land_z;


        // 设置垂直速度为负值，确保从机开始降落
        _sp_vel(2) = 0.2f;  // 设置合适的下降速度（可以根据需要调整）

        // 确保水平速度为 0，避免水平运动
        _sp_vel(0) = 0.0f;
        _sp_vel(1) = 0.0f;

        // 发送控制命令，确保从机按目标位置、速度和航向进行降落
        control_instance::getInstance()->Control_pos_vel_yaw(
            _sp_position_filtered,
            _sp_vel_filtered,
            _sp_yaw,
            _sp_yaw_speed
        );

        // 如果从机已经接近降落目标，可以切换状态为 DISARM
        STATE = state::DISARM;  // 降落完成后停止控制
        return;
    }


	// 从机：检查是否处于 OFFBOARD 模式（主机已在上面 return）
	if (_state.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
	    // 判断是否人为接管
	    if (_state.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL ||
	        _state.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL ||
	        _state.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
	        _state.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB ||
	        _state.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) {
	        PX4_WARN("Manual takeover detected, not switching back to OFFBOARD");
	        return;
	    } else {
	        PX4_WARN("Follower accidentally left OFFBOARD, attempting to re-enter...");
	        control_instance::getInstance()->Change_offborad();
	        return;
	    }
	}


	    //  2. 初始化偏移
	    if (!_offset_initialized) {
	        set_swarm_offset(Vector3f(
	            _param_swarm_X_offset.get(),
	            _param_swarm_Y_offset.get(),
	            _param_swarm_Z_offset.get()));
	        _offset_initialized = true;
	    }

	    //  3. 控制起始延迟
	    if (first_loop == 1) {
	        px4_usleep(1_s);
	        first_loop = 0;
	    }

	    //  6. 暂停集群控制
	    if (_swarm_start_flag.pause_swarm == 1) {
	        STATE = _set_as_leader ? state::ARM_AUTO : state::ARM_OFFBOARD;
	        return;
	    }

	    //  7. 从机执行跟随控制逻辑
	    //  注意：主机已经在上面提前return了，这里只会执行从机的跟随逻辑
	    start_swarm_node();

	    //  8. 自动降落
	    if (_state.arming_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
	        STATE = state::IDLE;
	        return;
	    }

	    //  9. 主动触发 LAND
	    if (_swarm_start_flag.stop_swarm == 1 || _leader_info.land == 1) {
	        PX4_WARN("Swarm termination triggered, landing...");
	        STATE = state::LAND;
	        return;
	    }
	}

	// 处理 LAND 状态
	void Swarm_Node::handle_land_state()
	{
	    if (_swarm_start_flag.start_swarm || _swarm_start_flag.start_swarm_auto) {
	        PX4_INFO("[Swarm] Received auto command in LAND state, switching to ARM state");

	        //  关键修复：根据start_swarm_auto或start_swarm选择正确的状态
	        if (_set_as_leader) {
	            if (_swarm_start_flag.start_swarm_auto) {
	                STATE = state::ARM_AUTO;  // 自动任务：进入ARM_AUTO
	                PX4_INFO("[主机] 切换到ARM_AUTO状态，准备执行自动任务");
	            } else if (_swarm_start_flag.start_swarm) {
	                STATE = state::ARM_LEADER;  // 手动任务：进入ARM_LEADER
	                PX4_INFO("[主机] 切换到ARM_LEADER状态");
	            }
	        } else {
	            STATE = state::ARM_OFFBOARD;  // 从机：进入ARM_OFFBOARD
	            PX4_INFO("[从机] 切换到ARM_OFFBOARD状态");
	        }

	        // 清除 stop_swarm 标志，确保后续状态判断正确
	        swarm_start_flag_s clear_flag{};
	        clear_flag.start_swarm = _swarm_start_flag.start_swarm;
	        clear_flag.start_swarm_auto = _swarm_start_flag.start_swarm_auto;
	        clear_flag.stop_swarm = 0;
	        _start_flag_pub.publish(clear_flag);
	        return;
	    }

	    if (control_instance::getInstance()->Change_land()) {
	        STATE = state::DISARM;
	    }
	}

	// 处理 DISARM 状态
	void Swarm_Node::handle_disarm_state(vehicle_status_s _state)
	{
	    if (_state.arming_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
	        STATE = state::IDLE;
	    }
	}

	// 处理 IDLE 状态
	void Swarm_Node::handle_idle_state(swarm_start_flag_s _start_flag)
	{
	    _start_flag = {};
	    _start_flag_pub.publish(_start_flag);
	    STATE = state::INIT;
	}

	// 处理人工接管
	void Swarm_Node::handle_manual_takeover(vehicle_status_s _state)
	{
	    const char *nav_mode_name = "UNKNOWN";

	    switch (_state.nav_state) {
	    case vehicle_status_s::NAVIGATION_STATE_POSCTL:
	        nav_mode_name = "POSCTL";
	        break;
	    case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
	        nav_mode_name = "ALTCTL";
	        break;
	    case vehicle_status_s::NAVIGATION_STATE_MANUAL:
	        nav_mode_name = "STABILIZED";
	        break;
	    default:
	        break;
	    }

	    PX4_WARN("Remote takeover: OFFBOARD -> %s, exiting swarm control", nav_mode_name);
	    STATE = state::IDLE;
	}


int Swarm_Node::task_spawn(int argc, char *argv[])
{
	Swarm_Node *instance = new Swarm_Node();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Swarm_Node::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int Swarm_Node::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Swarm_Node::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Swarm node module for multi-UAV formation control.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("swarm_node", "template");
	PRINT_MODULE_USAGE_COMMAND("start");

	return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
	return Swarm_Node::main(argc, argv);
}




bool Swarm_Node::uav_takeoff_altitude(matrix::Vector3f veicle_own_position, matrix::Vector3f target_position, matrix::Vector3f target_velocity,float takeoff_altitude,float target_yaw, float target_yawspeed) {
    // 如果从机没有达到目标高度，则控制其爬升
    if ( veicle_own_position(2) > -takeoff_altitude) {
        // 控制从机逐步爬升至目标高度
        _sp_position(0) = veicle_own_position(0);
        _sp_position(1) = veicle_own_position(1);
        _sp_position(2) = target_position(2); // 目标高度设为目标高度

        // 设置目标速度，控制从机逐渐爬升
        _sp_vel(0) = 0.0f;
        _sp_vel(1) = 0.0f;
        _sp_vel(2) = target_velocity(2); // 设置爬升速度适中，避免突变


        // 调用控制器进行位置、速度和航向控制
        control_instance::getInstance()->Control_pos_vel_yaw(
            _sp_position,
            _sp_vel,
            target_yaw,
            target_yawspeed
        );

        PX4_INFO("Follower is climbing to %.2f meters... Current altitude: %.2f", (double)takeoff_altitude, (double)_vehicle_local_position.z);
        return true; // 如果还没到达目标高度，直接退出此函数，保持爬升状态
    }
    // 如果达到了目标高度，停止爬升，返回 false
    return false;
}

void Swarm_Node::run_collision_avoidance(const matrix::Vector3f& current_pos, const OtherVehiclePosition* other_aircraft,
                                         float current_speed, bool need_avoidance, bool in_takeoff,
                                         bool& collision_risk, bool& critical_risk, matrix::Vector3f& correction)
{
    collision_risk = false;
    critical_risk = false;
    correction = matrix::Vector3f(0.0f, 0.0f, 0.0f);

    bool is_high_speed = current_speed > 2.0f;
    bool is_low_speed = current_speed < 1.0f;

    // 配置速度障碍参数
    VelocityObstacleController::VOConfig vo_config;
    vo_config.enable_avoidance = true;
    float speed_scale = is_high_speed ? 1.5f : (is_low_speed ? 1.0f : 1.2f);
    vo_config.safety_radius = _param_apf_safety_radius.get() * speed_scale;
    vo_config.danger_radius = _param_apf_danger_radius.get() * speed_scale;
    vo_config.max_avoidance_distance = _param_apf_max_distance.get() * 1.5f;
    vo_config.repulsive_gain = _param_apf_repulsive_gain.get() * speed_scale;
    vo_config.tangential_gain = _param_apf_tangential_gain.get() * speed_scale;
    vo_config.max_avoidance_force = _param_apf_max_force.get() * (is_high_speed ? 2.0f : 1.5f);
    vo_config.max_safe_velocity = is_high_speed ? (8.0f + current_speed) : 5.0f;
    vo_config.velocity_blend_factor = is_high_speed ? 0.9f : 0.7f;
    vo_config.enable_leader_avoidance = true;
    vo_config.clockwise_tangential = true;

    _vo_controller.set_config(vo_config);

    // 计算时间步长
    static hrt_abstime last_time = 0;
    hrt_abstime current_time = hrt_absolute_time();
    float dt = (last_time > 0) ? (current_time - last_time) * 1e-6f : 0.1f;
    dt = math::constrain(dt, 0.01f, 0.2f);
    last_time = current_time;

    // 使用速度障碍方法计算避撞
    matrix::Vector3f current_vel(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
    VelocityObstacleController::AvoidanceResult vo_result = _vo_controller.calculate_safe_velocity(
        current_pos, current_vel, _sp_vel_filtered, other_aircraft, MAX_SWARM_SIZE, vehicle_id, dt);

    correction = vo_result.position_correction;
    collision_risk = (vo_result.avoided_aircraft_count > 0);
    critical_risk = vo_result.emergency_avoidance;

    // 预测性轨迹碰撞检测
    uint64_t current_time_check = hrt_absolute_time();
    for (int i = 0; i < MAX_SWARM_SIZE; i++) {
        if (!other_aircraft[i].valid || other_aircraft[i].mavid == vehicle_id ||
            (current_time_check - other_aircraft[i].timestamp) >= 2000000) {
            continue;
        }

        matrix::Vector3f my_pos(current_pos(0), current_pos(1), 0.0f);
        matrix::Vector3f other_pos(other_aircraft[i].x, other_aircraft[i].y, 0.0f);
        matrix::Vector3f relative_pos = other_pos - my_pos;
        float distance = relative_pos.norm();

        bool is_leader = other_aircraft[i].is_leader;
        float critical_threshold = is_leader ? 4.5f : 2.5f;
        float warning_threshold = is_leader ? 6.0f : 4.0f;

        // 简化的轨迹预测
        matrix::Vector3f my_vel(_vehicle_local_position.vx, _vehicle_local_position.vy, 0.0f);
        matrix::Vector3f other_vel(other_aircraft[i].vx, other_aircraft[i].vy, 0.0f);
        matrix::Vector3f relative_vel = other_vel - my_vel;

        // 预测1秒后的最小距离
        float min_distance = distance;
        for (float t = 0.1f; t <= 1.0f; t += 0.2f) {
            matrix::Vector3f future_relative = relative_pos + relative_vel * t;
            min_distance = fminf(min_distance, future_relative.norm());
        }

        if (min_distance < critical_threshold) {
            critical_risk = true;
        } else if (min_distance < warning_threshold) {
            collision_risk = true;
        }
    }
}

void Swarm_Node::apply_avoidance_correction(const matrix::Vector3f& current_pos, const OtherVehiclePosition* other_aircraft,
                                            float current_speed, bool is_high_speed, const matrix::Vector3f& correction)
{
    const VelocityObstacleController::AvoidanceResult& vo_result = _vo_controller.get_last_result();

    // 速度替换逻辑
    float ttc_threshold = is_high_speed ? 4.0f : 3.0f;
    float min_avoidance_speed = is_high_speed ? (6.0f + current_speed * 0.5f) : 3.0f;

    if (vo_result.time_to_collision < ttc_threshold) {
        matrix::Vector3f avoidance_vel = vo_result.safe_target_velocity;
        float avoidance_speed = avoidance_vel.norm();

        if (avoidance_speed < min_avoidance_speed && avoidance_speed > 0.1f) {
            avoidance_vel = avoidance_vel.normalized() * min_avoidance_speed;
        }
        _sp_vel_filtered(0) = avoidance_vel(0);
        _sp_vel_filtered(1) = avoidance_vel(1);
    }

    // 位置修正
    float ttc_threshold_pos = (current_speed > 5.0f) ? 3.0f : (current_speed > 1.0f) ? 3.5f : 4.0f;

    // 检查主机避撞
    bool has_leader_collision = false;
    uint64_t current_time = hrt_absolute_time();
    for (int i = 0; i < MAX_SWARM_SIZE; i++) {
        if (other_aircraft[i].valid && other_aircraft[i].is_leader &&
            (current_time - other_aircraft[i].timestamp) < 2000000) {
            matrix::Vector3f leader_pos(other_aircraft[i].x, other_aircraft[i].y, 0.0f);
            matrix::Vector3f my_pos(current_pos(0), current_pos(1), 0.0f);
            if ((leader_pos - my_pos).norm() < _param_apf_max_distance.get() * 2.0f) {
                has_leader_collision = true;
                break;
            }
        }
    }

    // 应用位置修正
    if (vo_result.time_to_collision < ttc_threshold_pos) {
        float ttc = vo_result.time_to_collision;
        float base_scale = is_high_speed ? 3.0f : 1.5f;
        float ttc_scale = is_high_speed ? 3.0f : 1.5f;
        float correction_scale = has_leader_collision ?
            (base_scale + 1.0f) + (ttc_scale + 1.0f) * (ttc_threshold_pos - ttc) / ttc_threshold_pos :
            base_scale + ttc_scale * (ttc_threshold_pos - ttc) / ttc_threshold_pos;

        _sp_position_filtered(0) += correction(0) * correction_scale;
        _sp_position_filtered(1) += correction(1) * correction_scale;
    }
}

// Helper function implementations
void Swarm_Node::update_leader_info()
{
    int32_t selected_leader_id = _param_swarm_leader_id.get();

    _swarm_start_flag_sub.copy(&_swarm_start_flag);
    _vehicle_local_position_sub.copy(&_vehicle_local_position);
    _sensor_gps_sub.copy(&_sensor_gps);

    if (_leader_info_sub.updated()) {
        leader_info_s temp_info{};
        _leader_info_sub.copy(&temp_info);

        if (selected_leader_id == 0) {
            if (!PX4_ISFINITE(_leader_sp_glo_pos.lat) || _leader_sp_glo_pos.mavid == 0) {
                _leader_sp_glo_pos = temp_info;
            }
        } else {
            if (temp_info.mavid == (uint32_t)selected_leader_id) {
                if (_leader_sp_glo_pos.mavid != 0 && _leader_sp_glo_pos.mavid != (uint32_t)selected_leader_id) {
                    PX4_WARN("[主机切换] 从机%d: 从主机ID=%d切换到主机ID=%d",
                             vehicle_id, _leader_sp_glo_pos.mavid, selected_leader_id);
                }
                _leader_sp_glo_pos = temp_info;
            } else {
                if (_leader_sp_glo_pos.mavid != 0 && _leader_sp_glo_pos.mavid != (uint32_t)selected_leader_id) {
                    _leader_sp_glo_pos = leader_info_s{};
                }
            }
        }
    }
}

void Swarm_Node::calculate_target_position()
{
    if (!PX4_ISFINITE(_leader_sp_glo_pos.lat) || !PX4_ISFINITE(_leader_sp_glo_pos.lon)) {
        PX4_WARN("主从跟随问题: _leader_sp_glo_pos无效 lat=%.6f lon=%.6f mav_id=%u",
                 (double)_leader_sp_glo_pos.lat, (double)_leader_sp_glo_pos.lon, _leader_sp_glo_pos.mavid);
        return;
    }

    _global_local_proj_ref.project(_leader_sp_glo_pos.lat, _leader_sp_glo_pos.lon, target_x, target_y);

    _final_target(0) = target_x;
    _final_target(1) = target_y;

    // 高度处理：主机发送的是 MSL 高度，需要转换为本地 NED z 坐标
    float leader_ned_z = _vehicle_local_position.ref_alt - static_cast<float>(_leader_sp_glo_pos.alt);
    _final_target(2) = leader_ned_z;

    // 记录当前目标位置
    _last_target_position = _final_target;
    _last_target_valid = true;
}

bool Swarm_Node::check_formation_switching()
{
    bool formation_switching = false;
    if (_last_target_valid) {
        matrix::Vector3f target_delta = _final_target - _last_target_position;
        float target_distance = target_delta.norm();

        if (target_distance > 3.0f) {
            formation_switching = true;
            PX4_WARN("[编队切换检测] 从机%d: 目标位置突变%.2fm", vehicle_id, (double)target_distance);
        }
    }
    return formation_switching;
}

void Swarm_Node::run_path_planning(const matrix::Vector3f& current_pos, const OtherVehiclePosition* other_aircraft,
                                   bool formation_switching, float distance_to_target)
{
    matrix::Vector3f actual_target = _final_target;
    bool enable_path_planning = formation_switching && (distance_to_target > 5.0f);

    if (enable_path_planning) {
        FormationPlanner::PlannerConfig planner_config;
        planner_config.enable_planning = true;
        planner_config.prediction_time = 2.0f;
        planner_config.collision_threshold = 2.0f;
        planner_config.detour_distance = 3.0f;
        planner_config.max_detection_distance = 7.0f;
        _formation_planner.set_config(planner_config);

        matrix::Vector3f current_velocity(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

        FormationPlanner::PlanResult plan_result = _formation_planner.plan_formation_path(
            current_pos, _final_target, current_velocity, other_aircraft, MAX_SWARM_SIZE, vehicle_id);

        if (plan_result.needs_detour && !plan_result.direct_path_safe) {
            actual_target = plan_result.waypoint;
            _using_detour_waypoint = true;
            _current_waypoint = plan_result.waypoint;
            PX4_WARN("[路径规划] 从机%d: 启用绕行路径", vehicle_id);
        }
    } else {
        // 正常编队跟随：不使用路径规划，直接飞向目标
        if (_using_detour_waypoint) {
            if (_current_waypoint.norm() < 0.1f) {
                _using_detour_waypoint = false;
            } else {
                matrix::Vector3f to_waypoint = _current_waypoint - current_pos;
                if (to_waypoint.norm() < 1.0f) {
                    _using_detour_waypoint = false;
                    actual_target = _final_target;
                } else {
                    actual_target = _current_waypoint;
                }
            }
        }
    }

    _sp_position = actual_target;
}

float Swarm_Node::calculate_filter_alpha(bool formation_switching, bool collision_risk, bool critical_risk,
                                         float current_speed, bool enable_avoidance, bool at_target_position)
{
    bool is_high_speed = current_speed > 2.0f;
    bool is_low_speed = current_speed < 1.0f;
    bool is_formation_following = !formation_switching && !collision_risk;

    // 如果已经到达目标位置，使用非常平滑的滤波，保持稳定
    if (at_target_position) {
        return 0.05f;  // 已到达目标：超平滑滤波，保持稳定
    }

    if (is_formation_following) {
        return 0.1f;  // 编队跟随：平滑滤波
    } else if (is_high_speed) {
        return critical_risk ? 0.9f : (collision_risk ? 0.7f : 0.5f);
    } else if (is_low_speed) {
        return critical_risk ? 0.5f : (collision_risk ? 0.3f : 0.2f);
    } else {
        return critical_risk ? 0.7f : (collision_risk ? 0.5f : 0.3f);
    }
}

matrix::Vector3f Swarm_Node::calculate_final_velocity(bool formation_switching, bool collision_risk,
                                                     bool enable_avoidance, float current_speed, bool at_target_position)
{
    matrix::Vector3f final_vel = _sp_vel_filtered;

    // 如果已经到达目标位置，使用更小的速度指令保持稳定
    if (at_target_position) {
        // 已到达目标位置：使用很小的速度指令，主要依靠位置控制器
        float vel_magnitude = sqrtf(final_vel(0) * final_vel(0) + final_vel(1) * final_vel(1));
        const float max_stable_speed = 1.0f;  // 限制在1m/s以内保持稳定

        if (vel_magnitude > max_stable_speed) {
            final_vel(0) = final_vel(0) / vel_magnitude * max_stable_speed;
            final_vel(1) = final_vel(1) / vel_magnitude * max_stable_speed;
        }
        return final_vel;
    }

    // 编队切换时限制速度并计算指向目标的速度指令
    if (formation_switching && (!collision_risk || !enable_avoidance)) {
        const float max_allowed_speed = 6.0f;
        matrix::Vector3f current_pos(_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
        matrix::Vector3f target_position = _sp_position + _sp_offset;
        matrix::Vector3f to_target_vel = target_position - current_pos;
        float distance_to_target_vel = sqrtf(to_target_vel(0) * to_target_vel(0) + to_target_vel(1) * to_target_vel(1));

        if (distance_to_target_vel > 0.1f) {
            // 计算指向目标位置的方向
            matrix::Vector3f direction_to_target = to_target_vel.normalized();
            final_vel(0) = direction_to_target(0) * max_allowed_speed;
            final_vel(1) = direction_to_target(1) * max_allowed_speed;
        } else {
            // 已经接近目标位置，使用原速度指令但限制大小
            float vel_magnitude = sqrtf(final_vel(0) * final_vel(0) + final_vel(1) * final_vel(1));
            if (vel_magnitude > max_allowed_speed) {
                final_vel(0) = final_vel(0) / vel_magnitude * max_allowed_speed;
                final_vel(1) = final_vel(1) / vel_magnitude * max_allowed_speed;
            }
        }
    }

    return final_vel;
}
