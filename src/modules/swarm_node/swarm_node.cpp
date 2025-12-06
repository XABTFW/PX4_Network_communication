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
	//  关键修复：不要重置滤波器状态，避免切换队形时从机乱动
	// 只在初始化时重置滤波器，切换队形时保持滤波器状态，确保平滑过渡
	_sp_offset = sp_offset;
	// _filter_initialized = false;  // 注释掉，避免切换队形时重置滤波器
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

//上面这样的写法，实际上主机从机都会发布自己的_leader_group_id，也就意味着所有的主机、从机都在往leader_id这个mavlink消息包里面发消息，
//这也就是为什么在地面站接这个包的时候，bool leader这个变量会出现跳变的情况
}


bool
Swarm_Node::init()
{

	ScheduleOnInterval(20000_us); // 2000 us interval, 50 Hz rate

	return true;
}


bool
Swarm_Node::takeoff()
{
	control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 1);
	return false;
}


bool
Swarm_Node::arm_offboard()
{
	return false;
}

bool Swarm_Node::swarm_node_init()
{

    if (_init_done) {
        // 如果已经初始化过了，直接返回
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

    // 初始化逻辑
    _last_sp_yaw = NAN;

    // 订阅数据更新
    if (_vehicle_local_position_sub.updated()) {
        _vehicle_local_position_sub.copy(&_vehicle_local_position);
    }

	// 删除重复的leader_info读取，避免数据竞争
	// 主从跟随的数据读取在后面的Task中进行
	//那现在的逻辑就是反正来了包先copy，然后去做判断，必须是本机的才进行之后的操作

	_swarm_start_flag_sub.copy(&_swarm_start_flag);

	//_last_leader_altitude = _leader_sp_glo_pos.alt;


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

        // 设置 _init_done 为 true，表示初始化完成
        _init_done = true;

		PX4_INFO("swarm_node init return true");
        return true;
    }
    //PX4_INFO("swarm_node init return false");
    return false;
}



void Swarm_Node::start_swarm_node()
{
    //  在函数开始处统一获取选定的主机ID，避免重复声明
    int32_t selected_leader_id = _param_swarm_leader_id.get();
    
    if (!_set_as_leader) {
        _swarm_start_flag_sub.copy(&_swarm_start_flag);

        if (_swarm_start_flag.stop_swarm) {
            STATE = state::LAND;
            return;
        }



        _vehicle_local_position_sub.copy(&_vehicle_local_position);
        
        // 主从跟随：使用选定的主机ID（支持动态选择主机）
        //  关键改进：从机可以跟随任意选定的主机，不再硬编码ID=2
        if (_leader_info_sub.updated()) {
            leader_info_s temp_info{};
            _leader_info_sub.copy(&temp_info);
            
            // 使用函数开始处获取的selected_leader_id
            
            //  如果参数为0（未设置），使用第一个收到的leader_info（向后兼容）
            // 如果参数>0，只使用指定ID的主机数据
            if (selected_leader_id == 0) {
                // 自动模式：使用第一个收到的leader_info（向后兼容）
                if (!PX4_ISFINITE(_leader_sp_glo_pos.lat) || _leader_sp_glo_pos.mavid == 0) {
                    _leader_sp_glo_pos = temp_info;
                    PX4_INFO("[自动选择主机] 从机%d: 自动选择主机ID=%d", vehicle_id, temp_info.mavid);
                }
            } else {
                // 指定模式：只使用选定ID的主机数据
                //  修复类型不匹配：将int32_t转换为uint32_t进行比较
                if (temp_info.mavid == (uint32_t)selected_leader_id) {
                    //  检查是否切换了主机（从旧主机切换到新主机）
                    if (_leader_sp_glo_pos.mavid != 0 && _leader_sp_glo_pos.mavid != (uint32_t)selected_leader_id) {
                        PX4_WARN("[主机切换] 从机%d: 从主机ID=%d切换到主机ID=%d", 
                                 vehicle_id, _leader_sp_glo_pos.mavid, selected_leader_id);
                    }
                    _leader_sp_glo_pos = temp_info;  // 更新选定主机的位置
                    PX4_INFO("[跟随选定主机] 从机%d: 跟随主机ID=%d", vehicle_id, selected_leader_id);
                } else {
                    // 收到其他飞机的数据，忽略（用于避撞，但不用于主从跟随）
                    //  如果之前的主机ID不匹配，清除旧的主机位置信息
                    if (_leader_sp_glo_pos.mavid != 0 && _leader_sp_glo_pos.mavid != (uint32_t)selected_leader_id) {
                        PX4_WARN("[清除旧主机] 从机%d: 清除旧主机ID=%d的位置信息，等待新主机ID=%d", 
                                 vehicle_id, _leader_sp_glo_pos.mavid, selected_leader_id);
                        _leader_sp_glo_pos = leader_info_s{};  // 清除旧主机位置
                    }
                    PX4_DEBUG("[忽略其他主机] 从机%d: 收到ID=%d的数据，但选定主机是ID=%d", 
                              vehicle_id, temp_info.mavid, selected_leader_id);
                }
            }
        }

        // 位置共享：主机和从机都发布位置信息
        //  关键：主机也需要发布位置信息，让从机能够接收到
        _sensor_gps_sub.copy(&_sensor_gps);
        
        // 1. 发布本机位置（主机和从机都发布）
        _position_sharing.publish_position(vehicle_id, _vehicle_local_position, _sensor_gps);
        
        if (_set_as_leader) {
            // 主机：发布位置信息给从机
            PX4_DEBUG("[主机发布] 主机%d: 发布位置信息给从机", vehicle_id);
        }

        // 2. 接收其他飞机位置信息（来自 MAVLink 广播）
        // 传递选定的主机ID，用于识别主机位置（用于避撞和跟随）
        // 使用函数开始处获取的selected_leader_id
        if (!_set_as_leader) {
            // 从机：接收位置信息
            _position_sharing.update_other_positions(vehicle_id, _global_local_proj_ref, selected_leader_id);
        }

        // 3. 获取其他从机位置（用于避障和协调）
        const OtherVehiclePosition* other_vehicles = _position_sharing.get_other_vehicles();

        // 4. 可选：打印调试信息（只关注从机间通信）
        static uint64_t last_debug_time = 0;
        uint64_t now = hrt_absolute_time();
        if (now - last_debug_time > 10000000) {  // 每10秒打印一次，减少日志
            // 只显示其他从机，排除主机和本机
            int follower_count = 0;
            // 使用函数开始处获取的selected_leader_id
            int32_t leader_id_to_exclude = (selected_leader_id > 0) ? selected_leader_id : 2;  // 如果未设置，默认排除ID=2
            for (int i = 0; i < MAX_SWARM_SIZE; i++) {
                if (other_vehicles[i].valid && i != vehicle_id && i != leader_id_to_exclude) {  // 排除主机和自己
                    follower_count++;
                }
            }
            PX4_INFO("从机%d: 检测到%d个其他从机位置信息（排除主机ID=%d）", vehicle_id, follower_count, leader_id_to_exclude);
            last_debug_time = now;
        }

        matrix::Vector3f veicle_own_position;
        veicle_own_position(0) = _vehicle_local_position.x;
        veicle_own_position(1) = _vehicle_local_position.y;
        veicle_own_position(2) = _vehicle_local_position.z;
        //可地面站设置的起逻辑飞高度
        float takeoff_altitude = _param_take_altitude.get();
         
        if (!PX4_ISFINITE(_leader_sp_glo_pos.lat) || !PX4_ISFINITE(_leader_sp_glo_pos.lon)) {
            PX4_WARN("主从跟随问题: _leader_sp_glo_pos无效 lat=%.6f lon=%.6f mav_id=%u",
                     (double)_leader_sp_glo_pos.lat, (double)_leader_sp_glo_pos.lon, _leader_sp_glo_pos.mavid);

            // 调试：检查是否有任何leader_info消息
            leader_info_s debug_info{};
            bool has_leader_info = _leader_info_sub.copy(&debug_info);
            PX4_INFO("调试: leader_info_sub状态: %s, mav_id=%u",
                     has_leader_info ? "有数据" : "无数据", debug_info.mavid);

            return;
        }


        _global_local_proj_ref.project(_leader_sp_glo_pos.lat, _leader_sp_glo_pos.lon, target_x, target_y);

        // 更新目标状态和速度
        matrix::Vector3f new_target_position(target_x, target_y, begin_z + static_cast<float>(_leader_sp_glo_pos.alt));
        
        //  改进的编队切换检测：只检测目标位置的突变，不依赖速度
        // 速度检测会导致误判，因为正常跟随时也可能高速移动
        bool formation_switching = false;
        if (_last_target_valid) {
            matrix::Vector3f target_delta = new_target_position - _last_target_position;
            float target_distance = target_delta.norm();
            
            //  关键改进：提高阈值到3.0米，避免小幅位置调整被误判为编队切换
            // 只有真正的编队切换（偏移量大幅变化）才会触发
            if (target_distance > 3.0f) {  // 从2.0f提高到3.0f
                formation_switching = true;
                PX4_WARN("[编队切换检测] 从机%d: 目标位置突变%.2fm，检测到编队切换!", 
                         vehicle_id, (double)target_distance);
            }
        }
        
        //  移除速度检测：速度检测会导致误判，正常跟随时也可能高速移动
        // 只依赖目标位置的突变来检测编队切换，更准确
        
        // 更新最终目标位置（编队位置）
        _final_target(0) = target_x;
        _final_target(1) = target_y;
        _final_target(2) = begin_z + static_cast<float>(_leader_sp_glo_pos.alt);
        
        //  更新位置数据（用于路径规划和避撞检查）
        // 传递选定的主机ID，用于识别主机位置（用于避撞）
        // 使用函数开始处获取的selected_leader_id
        _position_sharing.update_other_positions(vehicle_id, _global_local_proj_ref, selected_leader_id);
        const OtherVehiclePosition* other_aircraft = _position_sharing.get_other_vehicles();
        
        // 路径规划：在编队切换或距离目标较远时启用
        matrix::Vector3f actual_target = _final_target;  // 默认直达编队位置
        
        // 计算到目标的距离
        matrix::Vector3f to_target = _final_target - veicle_own_position;
        to_target(2) = 0.0f;  // 只考虑XY平面
        float distance_to_target = to_target.norm();
        
        //  启用路径规划的条件：
        // 1. 编队切换时
        // 2. 或距离目标超过3米（起飞后找初始队形）
        bool enable_path_planning = formation_switching || (distance_to_target > 3.0f);
        
        if (enable_path_planning) {
            FormationPlanner::PlannerConfig planner_config;
            planner_config.enable_planning = true;
            planner_config.prediction_time = 3.0f;
            planner_config.collision_threshold = 2.5f;  // 2.5米碰撞阈值
            planner_config.detour_distance = 3.5f;      // 3.5米绕行距离
            _formation_planner.set_config(planner_config);
            
            matrix::Vector3f current_velocity(
                _vehicle_local_position.vx,
                _vehicle_local_position.vy,
                _vehicle_local_position.vz
            );
            
            FormationPlanner::PlanResult plan_result = _formation_planner.plan_formation_path(
                veicle_own_position,
                _final_target,
                current_velocity,
                other_aircraft,
                MAX_SWARM_SIZE,
                vehicle_id
            );
            
            // 根据规划结果选择目标
            if (plan_result.needs_detour && !plan_result.direct_path_safe) {
                // 需要绕行：先飞到绕行航点
                actual_target = plan_result.waypoint;
                _using_detour_waypoint = true;
                _current_waypoint = plan_result.waypoint;
                
                const char* reason = formation_switching ? "编队切换" : "初始队形";
                PX4_WARN("[路径规划] 从机%d: %s检测到碰撞风险，使用绕行航点 航点=(%.2f,%.2f) 绕开飞机%d",
                         vehicle_id, reason, (double)actual_target(0), (double)actual_target(1),
                         plan_result.conflict_aircraft_id);
            } else {
                const char* reason = formation_switching ? "编队切换" : "初始队形";
                PX4_INFO("[路径规划] 从机%d: %s路径安全，直达编队位置", vehicle_id, reason);
            }
        } else {
            // 正常跟随模式：直接使用编队目标，不启用路径规划
            // 如果之前在绕行，检查是否已经接近航点，可以切回直达
            if (_using_detour_waypoint) {
                matrix::Vector3f to_waypoint = _current_waypoint - veicle_own_position;
                float distance_to_waypoint = to_waypoint.norm();
                
                if (distance_to_waypoint < 1.0f) {
                    // 已经接近绕行航点，切回直达编队位置
                    PX4_INFO("[路径规划] 从机%d: 已到达绕行航点，切回直达编队位置", vehicle_id);
                    _using_detour_waypoint = false;
                    actual_target = _final_target;
                } else {
                    // 继续飞向绕行航点
                    actual_target = _current_waypoint;
                }
            }
            // else: 正常直达编队位置，actual_target已经设置为_final_target
        }
        
        // 使用规划后的目标位置
        _sp_position = actual_target;
        _sp_vel(0) = _leader_sp_glo_pos.vx;
        _sp_vel(1) = _leader_sp_glo_pos.vy;
        _sp_vel(2) = _leader_sp_glo_pos.vz;
        
        // 不再限制主机速度，位置控制器会根据位置偏差自动加速
        _sp_yaw = _leader_sp_glo_pos.yaw;
        _sp_yaw_speed = _leader_sp_glo_pos.yawspeed;
        
        // 记录当前目标位置
        _last_target_position = new_target_position;
        _last_target_valid = true;

        // --- YAW unwrap 保留 ---
        if (PX4_ISFINITE(_last_sp_yaw)) {
            float yaw_diff = _sp_yaw - _last_sp_yaw;
            if (yaw_diff > M_PI_F) {
                _sp_yaw -= 2.f * M_PI_F;
            } else if (yaw_diff < -M_PI_F) {
                _sp_yaw += 2.f * M_PI_F;
            }
        }
        _last_sp_yaw = _sp_yaw;

        // 检查是否处于起飞阶段（高度未达标）
        bool in_takeoff_phase = (veicle_own_position(2) > -takeoff_altitude);
        
        // 起飞阶段日志（每2秒输出一次）
        static uint64_t last_takeoff_log = 0;
        uint64_t now_takeoff = hrt_absolute_time();
        if (in_takeoff_phase && (now_takeoff - last_takeoff_log > 2000000)) {
            PX4_INFO("[起飞阶段] 从机%d: 当前高度%.2fm 目标高度%.2fm 避撞已禁用",
                     vehicle_id, (double)(-veicle_own_position(2)), (double)takeoff_altitude);
            last_takeoff_log = now_takeoff;
        }
        
        // 【优先】基于速度障碍的避撞检查：始终启用（不只是编队切换时），提高响应速度
        bool collision_risk = false;
        bool critical_collision_risk = false;
        matrix::Vector3f position_correction(0.0f, 0.0f, 0.0f);  // 位置修正量
        
        //  关键改进：避撞检查始终启用（不只是编队切换时），确保正常跟随时也能快速响应避撞
        // 这样可以在任何情况下都检测碰撞风险，提高避撞响应速度
        bool enable_avoidance_check = _param_apf_enable.get() > 0.5f && !in_takeoff_phase;
        if (enable_avoidance_check) {
            // 始终启用避撞检查，提高响应速度
            if (formation_switching) {
                PX4_INFO("[避撞检查] 从机%d: 编队切换模式，启用速度障碍避撞检查", vehicle_id);
            } else if (enable_path_planning) {
                PX4_INFO("[避撞检查] 从机%d: 找目标位置模式，启用速度障碍避撞检查", vehicle_id);
            } else {
                PX4_DEBUG("[避撞检查] 从机%d: 正常跟随模式，启用速度障碍避撞检查", vehicle_id);
            }
            
            // 配置速度障碍参数
            VelocityObstacleController::VOConfig vo_config;
            vo_config.enable_avoidance = true;
            
            // 关键改进：在保持编队时，大幅减弱避撞反应，避免过度避让
            // 保持编队时，飞机之间保持固定偏移量，这是正常的，不需要强烈避撞
            // 修复：从机在找目标位置时（enable_path_planning=true）也应该使用正常避撞配置
            bool is_moving_to_target = formation_switching || enable_path_planning;
            if (is_moving_to_target) {
                // 编队切换或找目标位置时：使用正常避撞配置，确保安全
                vo_config.safety_radius = _param_apf_safety_radius.get();
                vo_config.danger_radius = _param_apf_danger_radius.get();
                vo_config.max_avoidance_distance = _param_apf_max_distance.get();
                vo_config.repulsive_gain = _param_apf_repulsive_gain.get();
                vo_config.tangential_gain = _param_apf_tangential_gain.get();
                vo_config.max_avoidance_force = _param_apf_max_force.get();
            } else {
                // 保持编队时：大幅减弱避撞反应，避免过度避让
                // 减小安全半径和危险半径，只在真正危险时才避撞
                vo_config.safety_radius = _param_apf_safety_radius.get() * 0.5f;  // 减半
                vo_config.danger_radius = _param_apf_danger_radius.get() * 0.5f;  // 减半
                vo_config.max_avoidance_distance = _param_apf_max_distance.get() * 0.6f;  // 减小到60%
                // 大幅减弱避撞增益，避免过度避让
                vo_config.repulsive_gain = _param_apf_repulsive_gain.get() * 0.2f;  // 减弱到20%
                vo_config.tangential_gain = _param_apf_tangential_gain.get() * 0.2f;  // 减弱到20%
                vo_config.max_avoidance_force = _param_apf_max_force.get() * 0.3f;  // 减弱到30%
            }
            
            // 根据当前速度动态调整最大安全速度，改善高速状态下的避撞效果
            float current_speed_config = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx + 
                                               _vehicle_local_position.vy * _vehicle_local_position.vy);
            // 高速时（>5m/s）允许更大的避撞速度，确保高速避撞有效
            vo_config.max_safe_velocity = (current_speed_config > 5.0f) ? 5.0f : 3.0f;  // 高速时提高到5m/s
            vo_config.velocity_blend_factor = 0.8f;  // 速度融合因子（提高以更快响应）
            vo_config.enable_leader_avoidance = true;
            vo_config.clockwise_tangential = true;

            _vo_controller.set_config(vo_config);

            //  关键：获取当前速度和期望速度
            matrix::Vector3f current_vel(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
            matrix::Vector3f desired_vel = _sp_vel_filtered;  // 来自位置控制的期望速度
            
            // 计算时间步长（用于位置平滑）
            static hrt_abstime last_time = 0;
            hrt_abstime current_time = hrt_absolute_time();
            float dt = (last_time > 0) ? (current_time - last_time) * 1e-6f : 0.1f;
            dt = math::constrain(dt, 0.01f, 0.2f);  // 限制在合理范围
            last_time = current_time;

            //  使用速度障碍方法计算安全目标速度和位置修正
            VelocityObstacleController::AvoidanceResult vo_result = 
                _vo_controller.calculate_safe_velocity(
                    veicle_own_position,
                    current_vel,
                    desired_vel,
                    other_aircraft,
                    MAX_SWARM_SIZE,
                    vehicle_id,
                    dt
                );

            // 获取位置修正量
            position_correction = vo_result.position_correction;
            collision_risk = (vo_result.avoided_aircraft_count > 0);
            critical_collision_risk = vo_result.emergency_avoidance;
            
            // 检测严重碰撞风险：对冲情况 + 预测性轨迹碰撞检测
            //  预测未来1秒内的运动轨迹，检测路径交叉
            // 先计算有效飞机数量
            int valid_aircraft_count = 0;
            for (int j = 0; j < MAX_SWARM_SIZE; j++) {
                if (other_aircraft[j].valid) valid_aircraft_count++;
            }
            
            for (int i = 0; i < MAX_SWARM_SIZE; i++) {
                if (other_aircraft[i].valid && i != vehicle_id) {
                    // 当前位置和速度
                    matrix::Vector3f my_pos(veicle_own_position(0), veicle_own_position(1), 0.0f);
                    matrix::Vector3f my_vel(_vehicle_local_position.vx, _vehicle_local_position.vy, 0.0f);
                    matrix::Vector3f other_pos(other_aircraft[i].x, other_aircraft[i].y, 0.0f);
                    matrix::Vector3f other_vel(other_aircraft[i].vx, other_aircraft[i].vy, 0.0f);
                    
                    // 修复：检查是否是主机，主机避撞需要更敏感的检测
                    bool is_leader_aircraft = other_aircraft[i].is_leader;
                    
                    // 如果编队切换或找目标位置，使用目标点方向作为预测方向（而不是当前速度）
                    // 修复：从机在找目标位置时也应该使用目标点方向，更准确预测碰撞风险
                    matrix::Vector3f my_target_dir = my_vel;
                    if (formation_switching || enable_path_planning) {
                        matrix::Vector3f target_direction(
                            _sp_position_filtered(0) - veicle_own_position(0),
                            _sp_position_filtered(1) - veicle_own_position(1),
                            0.0f
                        );
                        float target_dist = target_direction.norm();
                        if (target_dist > 0.1f) {
                            // 使用朝向目标点的方向，假设速度为1.5m/s（编队切换或找位置速度）
                            my_target_dir = target_direction.normalized() * 1.5f;
                        }
                    }
                    
                    // 相对位置和速度
                    matrix::Vector3f relative_pos = other_pos - my_pos;
                    matrix::Vector3f relative_vel = other_vel - my_vel;
                    float distance = relative_pos.norm();
                    
                    //  轨迹预测：采样未来1.5秒内的路径，检测最小距离
                    float min_predicted_distance = distance;  // 初始化为当前距离
                    float critical_time = 0.0f;  // 最危险时刻
                    
                    //  小规模编队：预测更长时间（1.5秒），采样更密集（15个点）
                    int prediction_steps = (valid_aircraft_count <= 5) ? 15 : 10;
                    float prediction_horizon = (valid_aircraft_count <= 5) ? 1.5f : 1.0f;
                    
                    // 采样未来轨迹
                    for (int t = 1; t <= prediction_steps; t++) {
                        float dt_pred = t * (prediction_horizon / prediction_steps);
                        
                        // 预测t秒后的位置
                        matrix::Vector3f my_future = my_pos + my_target_dir * dt_pred;
                        matrix::Vector3f other_future = other_pos + other_vel * dt_pred;
                        
                        float predicted_distance = (other_future - my_future).norm();
                        
                        if (predicted_distance < min_predicted_distance) {
                            min_predicted_distance = predicted_distance;
                            critical_time = dt_pred;
                        }
                    }
                    
                    // 计算TTC（Time To Collision）
                    float closing_speed = -relative_pos.normalized().dot(relative_vel);
                    float ttc = 999.0f;  // 默认很大
                    if (closing_speed > 0.1f) {
                        ttc = distance / closing_speed;  // 碰撞时间（秒）
                    }
                    
                    //  关键改进：在保持编队时，提高对冲检测阈值，避免过度避让
                    // 保持编队时，飞机之间保持固定偏移量，这是正常的，不需要强烈避撞
                    // 修复：从机在找目标位置时也应该使用正常避撞阈值
                    // 修复：主机避撞使用更敏感的阈值，确保主从避撞更早触发
                    bool is_moving_to_target_local = formation_switching || enable_path_planning;
                    float collision_distance_threshold;
                    float collision_dot_threshold;
                    if (is_leader_aircraft) {
                        // 主机避撞：使用更敏感的阈值，更早检测
                        collision_distance_threshold = is_moving_to_target_local ? 5.0f : 4.0f;  // 主机使用更大的检测距离
                        collision_dot_threshold = is_moving_to_target_local ? -0.2f : -0.3f;  // 主机使用更宽松的dot阈值
                    } else {
                        // 从机避撞：使用正常阈值
                        collision_distance_threshold = is_moving_to_target_local ? 4.0f : 2.5f;
                        collision_dot_threshold = is_moving_to_target_local ? -0.3f : -0.5f;
                    }
                    
                    //  当前距离对冲检测
                    if (distance > 0.1f && distance < collision_distance_threshold) {
                        float dot_product = relative_pos.dot(relative_vel);
                        
                        //  对冲检测：相向而行
                        if (dot_product < collision_dot_threshold) {
                            critical_collision_risk = true;
                            PX4_WARN("[当前对冲] 从机%d↔飞机%d: 距离=%.2fm TTC=%.2fs 轨迹最近=%.2fm@%.1fs",
                                     vehicle_id, i, (double)distance, (double)ttc, 
                                     (double)min_predicted_distance, (double)critical_time);
                        }
                        // 近距离对冲（保持编队时提高阈值）
                        // 修复：主机避撞使用更敏感的近距离阈值
                        float close_distance_threshold;
                        float close_dot_threshold;
                        if (is_leader_aircraft) {
                            // 主机避撞：使用更敏感的阈值
                            close_distance_threshold = is_moving_to_target_local ? 3.5f : 3.0f;  // 主机使用更大的近距离阈值
                            close_dot_threshold = is_moving_to_target_local ? 0.0f : -0.1f;  // 主机使用更宽松的dot阈值
                        } else {
                            // 从机避撞：使用正常阈值
                            close_distance_threshold = is_moving_to_target_local ? 2.5f : 1.5f;
                            close_dot_threshold = is_moving_to_target_local ? -0.1f : -0.3f;
                        }
                        if (distance < close_distance_threshold && dot_product < close_dot_threshold) {
                            critical_collision_risk = true;
                            PX4_WARN("[近距对冲] 从机%d↔飞机%d: %.2fm TTC=%.2fs",
                                     vehicle_id, i, (double)distance, (double)ttc);
                        }
                    }
                    
                    //  关键改进：在保持编队时，提高轨迹交叉检测阈值，避免过度避让
                    // 保持编队时，飞机之间保持固定偏移量，这是正常的，不需要强烈避撞
                    // 修复：从机在找目标位置时也应该使用正常避撞阈值
                    // 修复：主机避撞使用更敏感的轨迹交叉阈值
                    float critical_distance_threshold;
                    float warning_distance_threshold;
                    if (is_leader_aircraft) {
                        // 主机避撞：使用更敏感的阈值，更早检测
                        critical_distance_threshold = is_moving_to_target_local ? 3.5f : 3.0f;  // 主机使用更大的critical阈值
                        warning_distance_threshold = is_moving_to_target_local ? 5.0f : 4.5f;   // 主机使用更大的warning阈值
                    } else {
                        // 从机避撞：使用正常阈值
                        critical_distance_threshold = is_moving_to_target_local ? 2.5f : 1.5f;
                        warning_distance_threshold = is_moving_to_target_local ? 4.0f : 2.5f;
                    }
                    
                    // 预测距离<2.5米：critical（立即强制避撞）
                    if (min_predicted_distance < critical_distance_threshold && critical_time < prediction_horizon) {
                        critical_collision_risk = true;
                        PX4_WARN("[轨迹交叉] 从机%d↔飞机%d: 当前%.2fm, %.1f秒后交叉至%.2fm 立即避撞!",
                                 vehicle_id, i, (double)distance, (double)critical_time, 
                                 (double)min_predicted_distance);
                    }
                    // 预测距离<4米：warning（提前减速）
                    else if (min_predicted_distance < warning_distance_threshold && critical_time < prediction_horizon) {
                        collision_risk = true;
                        PX4_WARN("[轨迹接近] 从机%d↔飞机%d: 当前%.2fm, %.1f秒后接近至%.2fm  提前减速!",
                                 vehicle_id, i, (double)distance, (double)critical_time, 
                                 (double)min_predicted_distance);
                    }
                }
            }
            
            if (collision_risk) {
                PX4_WARN("[避撞优先] 从机%d: 检测到碰撞风险! 避开%d架飞机，最近距离=%.2fm TTC=%.2fs %s",
                         vehicle_id, vo_result.avoided_aircraft_count, (double)vo_result.min_distance,
                         (double)vo_result.time_to_collision,
                         critical_collision_risk ? "【对冲警告】" : "");
            }
        }

        // --- 一阶滤波处理（根据碰撞风险和编队切换动态调整滤波系数）---
        float alpha = 0.3f; //  默认滤波系数，正常跟随快速响应
        
        //  关键改进：检测到碰撞风险时，增大alpha值提高响应速度
        // 编队切换时的alpha值在滤波器逻辑中单独处理，这里不再设置
        if (critical_collision_risk) {
            // 紧急碰撞风险：极大提高响应速度，让避撞指令立即生效
            alpha = 0.8f;  // 提高响应速度
        } else if (collision_risk) {
            // 一般碰撞风险：提高响应速度，让避撞指令快速生效
            alpha = 0.6f;  // 提高响应速度
        } else {
            // 队形稳定时：快速跟随
            alpha = 0.3f;
        }
        // 注意：编队切换时的alpha值在滤波器逻辑中单独处理（使用0.15f），确保平滑过渡

        //  关键改进：只在编队切换时限制速度，正常跟随时不限制
        // 位置控制器会根据位置偏差自动计算速度，所以需要限制位置setpoint的变化率
        // 但只在编队切换时限制，正常跟随时不限制，避免影响跟随功能
        matrix::Vector3f desired_position = _sp_position + _sp_offset;
        
        //  关键改进：编队切换时，使用更大的滤波系数，让位置setpoint平滑过渡到新目标
        // 而不是突然跳变，这样可以避免从机乱动
        // 保持滤波器状态，让位置平滑过渡
        
        if (!_filter_initialized) {
            // 初始阶段直接赋值，避免滤波器历史值为零或未定义
            _sp_position_filtered = desired_position;
            _sp_vel_filtered = _sp_vel;
            _filter_initialized = true;
        } else {
            //  关键改进：编队切换时，使用更大的alpha值让位置快速但平滑地过渡到新目标
            // 这样既能让从机快速响应新队形，又能避免突然跳变导致的乱动
            if (formation_switching && (!collision_risk || !enable_avoidance_check)) {
                // 编队切换时：使用较大的alpha值（0.15），让位置快速但平滑地过渡
                // 这样既能快速响应新队形，又能避免突然跳变
                float switch_alpha = 0.15f;  // 编队切换时的滤波系数
                _sp_position_filtered = switch_alpha * desired_position + (1.f - switch_alpha) * _sp_position_filtered;
                
                // 速度正常滤波
                _sp_vel_filtered = alpha * _sp_vel + (1.f - alpha) * _sp_vel_filtered;
                
                PX4_INFO("[编队切换平滑过渡] 从机%d: 位置平滑过渡到新目标 (%.2f,%.2f,%.2f)", 
                         vehicle_id, (double)desired_position(0), (double)desired_position(1), (double)desired_position(2));
            } else if (collision_risk && enable_avoidance_check) {
                // 有碰撞风险时：使用高alpha值快速响应
                // 位置：快速更新，位置修正会在后面统一应用
                _sp_position_filtered = alpha * desired_position + (1.f - alpha) * _sp_position_filtered;
                
                // 速度：快速更新，避撞速度会在后面直接替换
                _sp_vel_filtered = alpha * _sp_vel + (1.f - alpha) * _sp_vel_filtered;
            } else {
                // 正常滤波计算（使用限制后的位置）
                _sp_position_filtered = alpha * desired_position + (1.f - alpha) * _sp_position_filtered;
                _sp_vel_filtered = alpha * _sp_vel + (1.f - alpha) * _sp_vel_filtered;
            }
        }

        // 不再限制速度，位置控制器会根据位置偏差自动加速
        // 只保留高速避撞时的速度替换逻辑
        // 不再限制位置误差，位置控制器会根据位置偏差自动加速
        // 高速对冲时，避撞修正会直接修改位置和速度，确保避撞效果

       // 这里是起飞逻辑，飞行器首先爬升到设定的高度，然后才会按照队形进行变换,可在地面站设置起飞高度
      if(uav_takeoff_altitude( veicle_own_position ,_sp_position_filtered, _sp_vel_filtered, takeoff_altitude, _sp_yaw, _sp_yaw_speed)){
      	 return;
      }

      //  关键：应用位置修正到位置setpoint（位置控制+势场位移修正）
      // 始终应用位置修正（不只是编队切换时），提高避撞响应速度
      if (_param_apf_enable.get() > 0.5f && collision_risk) {
          const VelocityObstacleController::AvoidanceResult& vo_result = _vo_controller.get_last_result();
          
          //  关键改进：更早触发速度替换，提高响应速度
          // 只要有碰撞风险就立即替换速度，不等待TTC阈值
          // 这样可以更快响应避撞指令
          float current_speed_check = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx + 
                                            _vehicle_local_position.vy * _vehicle_local_position.vy);
          
          //  修复：根据速度动态调整TTC阈值，低速时延长阈值，确保低速时也能及时触发避撞
          // 低速时需要更早触发，因为速度慢意味着需要更多时间避让
          float ttc_threshold_replace;
          if (current_speed_check > 5.0f) {
              // 高速时：3.0秒
              ttc_threshold_replace = 3.0f;
          } else if (current_speed_check > 1.0f) {
              // 中速时：3.5秒
              ttc_threshold_replace = 3.5f;
          } else {
              // 低速时：4.0秒，给更多时间提前避撞
              ttc_threshold_replace = 4.0f;
          }
          
          //  改进：只要有碰撞风险就替换速度，不等待critical_collision_risk
          // 这样可以更快响应，提高避撞响应速度
          if (collision_risk && vo_result.time_to_collision < ttc_threshold_replace) {
              // 检测到碰撞风险时：立即使用避撞速度，覆盖期望速度
              matrix::Vector3f avoidance_vel = vo_result.safe_target_velocity;
              
              //  修复：根据当前速度动态调整避撞速度最小值，确保低速时也能有效避撞
              // 低速时避撞速度应该至少是当前速度的1.5倍，但不超过合理范围
              float avoidance_speed = avoidance_vel.norm();
              float min_avoidance_speed;
              if (current_speed_check > 5.0f) {
                  // 高速时：要求更大的避撞速度（8m/s）
                  min_avoidance_speed = 8.0f;
              } else if (current_speed_check > 1.0f) {
                  // 中速时：要求适中的避撞速度（4m/s）
                  min_avoidance_speed = 4.0f;
              } else {
                  // 低速时：避撞速度至少是当前速度的1.5倍，但最小1.0m/s
                  min_avoidance_speed = fmaxf(current_speed_check * 1.5f, 1.0f);
              }
              
              if (avoidance_speed < min_avoidance_speed && avoidance_speed > 0.1f) {
                  avoidance_vel = avoidance_vel.normalized() * min_avoidance_speed;
                  PX4_WARN("[避撞速度增强] 从机%d: 避撞速度%.2fm/s太小，增强到%.2fm/s 当前速度=%.2fm/s", 
                           vehicle_id, (double)avoidance_speed, (double)min_avoidance_speed, 
                           (double)current_speed_check);
              }
              
              //  直接替换速度，绕过滤波器延迟，立即生效
              _sp_vel_filtered(0) = avoidance_vel(0);
              _sp_vel_filtered(1) = avoidance_vel(1);
              // Z方向保持原速度
              
              PX4_WARN("[快速避撞速度替换] 从机%d: 立即替换速度指令！避撞速度=%.2fm/s TTC=%.2fs 当前速度=%.2fm/s", 
                       vehicle_id, (double)avoidance_vel.norm(), (double)vo_result.time_to_collision,
                       (double)current_speed_check);
          }
          
          //  将位置修正量应用到位置setpoint
          //  关键改进：立即应用位置修正，不等待TTC阈值，提高响应速度
          // 只要有碰撞风险就立即应用位置修正，让避撞指令快速生效
          float current_speed_pos = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx + 
                                         _vehicle_local_position.vy * _vehicle_local_position.vy);
          
          //  修复：根据速度动态调整位置修正的TTC阈值，低速时延长阈值，确保低速时也能及时触发避撞
          // 低速时需要更早触发，因为速度慢意味着需要更多时间避让
          float ttc_threshold_pos_scale;
          if (current_speed_pos > 5.0f) {
              // 高速时：3.0秒
              ttc_threshold_pos_scale = 3.0f;
          } else if (current_speed_pos > 1.0f) {
              // 中速时：3.5秒
              ttc_threshold_pos_scale = 3.5f;
          } else {
              // 低速时：4.0秒，给更多时间提前避撞
              ttc_threshold_pos_scale = 4.0f;
          }
          
          float correction_scale = 1.0f;
          // 修复：检查是否有主机在避撞范围内，主机避撞需要更强的位置修正
          bool has_leader_collision = false;
          if (collision_risk) {
              for (int i = 0; i < MAX_SWARM_SIZE; i++) {
                  if (other_aircraft[i].valid && other_aircraft[i].is_leader) {
                      matrix::Vector3f leader_pos(other_aircraft[i].x, other_aircraft[i].y, 0.0f);
                      matrix::Vector3f my_pos(veicle_own_position(0), veicle_own_position(1), 0.0f);
                      float leader_distance = (leader_pos - my_pos).norm();
                      // 如果主机在避撞范围内，标记为主机避撞
                      if (leader_distance < _param_apf_max_distance.get() * 2.0f) {
                          has_leader_collision = true;
                          break;
                      }
                  }
              }
          }
          
          // 关键改进：在保持编队时，大幅减弱位置修正，避免过度避让
          // 保持编队时，飞机之间保持固定偏移量，这是正常的，不需要强烈避撞
          // 修复：主机避撞始终使用强位置修正，确保主从避撞有效
          if (collision_risk && vo_result.time_to_collision < ttc_threshold_pos_scale) {
              float ttc = vo_result.time_to_collision;
              if (has_leader_collision) {
                  // 主机避撞：使用强位置修正，确保能及时避让
                  // TTC越小，放大倍数越大 [3, 5]
                  correction_scale = 3.0f + 2.0f * (ttc_threshold_pos_scale - ttc) / ttc_threshold_pos_scale;  // ttc=0时5倍
                  PX4_WARN("[主机位置修正] 主机避撞！放大位置修正量！TTC=%.2fs 倍数=%.1fx 当前速度=%.2fm/s", 
                           (double)ttc, (double)correction_scale, (double)current_speed_pos);
              } else if (formation_switching) {
                  // 编队切换时：适度放大位置修正量，确保避撞效果
                  // TTC越小，放大倍数越大 [2, 4]
                  correction_scale = 2.0f + 2.0f * (ttc_threshold_pos_scale - ttc) / ttc_threshold_pos_scale;  // ttc=0时4倍
                  PX4_WARN("[快速位置修正] 编队切换时放大位置修正量！TTC=%.2fs 倍数=%.1fx 当前速度=%.2fm/s", 
                           (double)ttc, (double)correction_scale, (double)current_speed_pos);
              } else {
                  // 保持编队时：大幅减弱位置修正，避免过度避让
                  // 只在真正危险时才应用修正，且修正量很小
                  correction_scale = 0.3f + 0.2f * (ttc_threshold_pos_scale - ttc) / ttc_threshold_pos_scale;  // [0.3, 0.5]
                  PX4_INFO("[保持编队位置修正] 保持编队时减弱位置修正量！TTC=%.2fs 倍数=%.2fx 当前速度=%.2fm/s", 
                           (double)ttc, (double)correction_scale, (double)current_speed_pos);
              }
          }
          
          //  立即应用位置修正，不等待
          _sp_position_filtered(0) += position_correction(0) * correction_scale;
          _sp_position_filtered(1) += position_correction(1) * correction_scale;
          // Z方向不修正
          
          PX4_WARN("[位置修正] 从机%d: 立即应用位置修正(%.2f,%.2f) 放大%.1fx 最近距离=%.2fm TTC=%.2fs %s",
                   vehicle_id, 
                   (double)(position_correction(0) * correction_scale), 
                   (double)(position_correction(1) * correction_scale),
                   (double)correction_scale,
                   (double)vo_result.min_distance, (double)vo_result.time_to_collision,
                   critical_collision_risk ? "【紧急】" : "");
      }

        //  关键改进：只在编队切换时限制速度，正常跟随时不限制
        // 位置控制器会根据位置偏差自动加速，所以需要限制速度指令
        // 但只在编队切换时限制，正常跟随时不限制，避免影响跟随功能
        matrix::Vector3f final_vel = _sp_vel_filtered;
        
        //  只在编队切换时限制速度，正常跟随时不限制
        // 避撞时不受限速影响，确保避撞效果
        // enable_avoidance_check 在上面已经定义
        if (formation_switching && (!collision_risk || !enable_avoidance_check)) {
            // 速度限制：6m/s（只在编队切换时）
            float max_allowed_speed = 6.0f;
            
            //  关键改进：在编队切换时，计算指向目标位置的速度指令
            // 因为位置setpoint等于当前位置，需要通过速度指令来控制移动方向
            matrix::Vector3f target_position = _sp_position + _sp_offset;
            matrix::Vector3f to_target_vel = target_position - veicle_own_position;
            float distance_to_target_vel = sqrtf(to_target_vel(0) * to_target_vel(0) + to_target_vel(1) * to_target_vel(1));
            
            // 如果距离目标位置较远，计算指向目标位置的速度指令
            if (distance_to_target_vel > 0.1f) {
                // 计算指向目标位置的方向
                matrix::Vector3f direction_to_target = to_target_vel.normalized();
                
                // 速度指令指向目标位置，大小限制在max_allowed_speed
                final_vel(0) = direction_to_target(0) * max_allowed_speed;
                final_vel(1) = direction_to_target(1) * max_allowed_speed;
                // Z方向保持原速度
            } else {
                // 已经接近目标位置，使用原速度指令但限制大小
                float vel_magnitude = sqrtf(final_vel(0) * final_vel(0) + final_vel(1) * final_vel(1));
                if (vel_magnitude > max_allowed_speed) {
                    final_vel(0) = final_vel(0) / vel_magnitude * max_allowed_speed;
                    final_vel(1) = final_vel(1) / vel_magnitude * max_allowed_speed;
                }
            }
        }
        
        //  关键改进：编队切换时，使用平滑过渡的位置setpoint，不再强制设置为当前位置
        // 因为滤波器已经平滑过渡，位置setpoint会逐渐接近新目标，避免突然跳变
        matrix::Vector3f final_position = _sp_position_filtered;
        
        //  始终使用位置控制模式（位置+速度混合控制）
        // position_correction已经应用到_sp_position_filtered，位置控制器会自动处理
        // Control_pos_vel_yaw内部已经设置了ocm.position=true和ocm.velocity=true
        control_instance::getInstance()->Control_pos_vel_yaw(
            final_position,  // 使用平滑过渡的位置setpoint
            final_vel,  // 使用限制后的速度
            _sp_yaw,
            _sp_yaw_speed
        );

 
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

	    //if (_swarm_start_flag_sub.updated()) {
	        _swarm_start_flag_sub.copy(&_start_flag);
	    //}

	    //if (_vehicle_status_sub.updated()) {
	        _vehicle_status_sub.copy(&_state);
	   // }


	    // 移除无条件的leader_info_copy，避免数据竞争
	    // leader_info的读取在主从跟随逻辑中处理

	    follower_info_s _follower_info{};
		_follower_info_sub.copy(&_follower_info);
		//在这里补一个对于其他飞机消息的更新

		update_uav_info();

		//double distance=0;
		//distance = calculate_distance(uav_info_cache[3].lat,uav_info_cache[3].lon,uav_info_cache[2].lat,uav_info_cache[2].lon);
		//PX4_INFO("distance is %f",distance);
		// PX4_INFO("The uav_info_cache[2].lat is %f",uav_info_cache[2].lat);
		// PX4_INFO("The uav_info_cache[2].lon is %f",uav_info_cache[2].lon);
		// PX4_INFO("The uav_info_cache[3].lat is %f",uav_info_cache[3].lat);
		// PX4_INFO("The uav_info_cache[3].lon is %f",uav_info_cache[3].lon);
		// PX4_INFO("The uav_info_cache[4].lat is %f",uav_info_cache[4].lat);
		// PX4_INFO("The uav_info_cache[4].lon is %f",uav_info_cache[4].lon);
		// PX4_INFO("The uav_info_cache[5].lat is %f",uav_info_cache[5].lat);
		// PX4_INFO("The uav_info_cache[5].lon is %f",uav_info_cache[5].lon);


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
				_position_sharing.publish_position(vehicle_id, vehicle_local_pos, sensor_gps);
				
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
        //PX4_WARN("Leader in MISSION state, starting to land!");

        // 同步从机目标高度为主机当前高度
        _sp_position(2) = _leader_sp_glo_pos.alt;  // 从机目标高度同步主机高度
       

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


  	if (!_set_as_leader) {
	    // 检查是否处于 OFFBOARD 模式，如果是并且没有手动切换，则自动切回
	    if (_state.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
	        // 判断是否人为接管（比如进入 POSITION 或 ALTCTL 模式）
	        if (_state.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || 
	            _state.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL ||
	            _state.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
	            _state.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB ||
	            _state.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) {
	            // 人工接管，停止自动切换到 OFFBOARD 模式
	            PX4_WARN("Manual takeover detected, not switching back to OFFBOARD");
	            return;
	        } else {
	            // 如果从机意外退出 OFFBOARD 模式，重新进入 OFFBOARD
	            PX4_WARN("Follower accidentally left OFFBOARD, attempting to re-enter...");
	            control_instance::getInstance()->Change_offborad();
	            return; // 这一轮不执行控制，等下一轮确认模式切换成功
	        }
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
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
	return Swarm_Node::main(argc, argv);
}


// 计算两点间的 Haversine 距离（单位：米）
double Swarm_Node::calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371e3; // 地球半径，单位：米
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double delta_phi = (lat2 - lat1) * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_phi / 2) * sin(delta_phi / 2) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2) * sin(delta_lambda / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // 返回距离，单位：米
}



void Swarm_Node::update_uav_info() {

    
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

