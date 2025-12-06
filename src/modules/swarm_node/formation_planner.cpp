#include "formation_planner.hpp"
#include <px4_platform_common/log.h>
#include <cmath>

FormationPlanner::PlanResult FormationPlanner::plan_formation_path(
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& target_pos,
    const matrix::Vector3f& current_vel,
    const OtherVehiclePosition* other_aircraft,
    int max_aircraft_count,
    uint8_t current_vehicle_id)
{
    _last_result = PlanResult{};
    _last_result.waypoint = target_pos;  // 默认直达目标
    _last_result.direct_path_safe = true;
    _last_result.needs_detour = false;

    if (!_config.enable_planning) {
        return _last_result;
    }

    // 计算到目标的方向和距离
    matrix::Vector3f to_target = target_pos - current_pos;
    float distance_to_target = to_target.norm();

    if (distance_to_target < 0.5f) {
        // 已经到达目标附近，不需要规划
        return _last_result;
    }

    matrix::Vector3f my_velocity = to_target.normalized() * 2.0f;  // 假设2m/s速度前往目标

    // 遍历所有其他飞机，检测碰撞
    float min_time_to_collision = 999.0f;
    int conflict_aircraft = -1;
    matrix::Vector3f conflict_position(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < max_aircraft_count; i++) {
        if (!other_aircraft[i].valid || i == current_vehicle_id) {
            continue;
        }

        // 其他飞机的位置和速度
        matrix::Vector3f other_pos(other_aircraft[i].x, other_aircraft[i].y, current_pos(2));
        matrix::Vector3f other_vel(other_aircraft[i].vx, other_aircraft[i].vy, 0.0f);

        // 预测碰撞
        float time_to_collision = 999.0f;
        float min_distance = 999.0f;
        bool will_collide = predict_collision(
            current_pos, my_velocity,
            other_pos, other_vel,
            time_to_collision, min_distance
        );

        if (will_collide && time_to_collision < min_time_to_collision) {
            min_time_to_collision = time_to_collision;
            conflict_aircraft = i;
            conflict_position = other_pos;
            _last_result.direct_path_safe = false;
        }

        // 调试信息
        if (will_collide) {
            PX4_WARN("[路径规划] 从机%d: 检测到与飞机%d的潜在碰撞 时间=%.1fs 最小距离=%.2fm",
                     current_vehicle_id, i, (double)time_to_collision, (double)min_distance);
        }
    }

    // 如果检测到碰撞，生成绕行航点
    if (!_last_result.direct_path_safe && conflict_aircraft >= 0) {
        _last_result.needs_detour = true;
        _last_result.conflict_aircraft_id = conflict_aircraft;
        _last_result.time_to_collision = min_time_to_collision;

        // 生成顺时针绕行航点
        _last_result.waypoint = generate_detour_waypoint(
            current_pos, target_pos, conflict_position
        );

        PX4_WARN("[路径规划] 从机%d: 规划绕行路径 航点=(%.2f,%.2f) 绕开飞机%d",
                 current_vehicle_id,
                 (double)_last_result.waypoint(0), (double)_last_result.waypoint(1),
                 conflict_aircraft);
    }

    return _last_result;
}

bool FormationPlanner::predict_collision(
    const matrix::Vector3f& my_pos,
    const matrix::Vector3f& my_vel,
    const matrix::Vector3f& other_pos,
    const matrix::Vector3f& other_vel,
    float& time_to_collision,
    float& min_distance)
{
    // 相对位置和速度（只考虑XY平面）
    matrix::Vector3f relative_pos(
        other_pos(0) - my_pos(0),
        other_pos(1) - my_pos(1),
        0.0f
    );
    matrix::Vector3f relative_vel(
        other_vel(0) - my_vel(0),
        other_vel(1) - my_vel(1),
        0.0f
    );

    float current_distance = relative_pos.norm();
    
    // 如果相对速度很小，认为不会碰撞
    float relative_speed = relative_vel.norm();
    if (relative_speed < 0.1f) {
        time_to_collision = 999.0f;
        min_distance = current_distance;
        return false;
    }

    // 计算最近接近时间 (CPA - Closest Point of Approach)
    float t_cpa = -(relative_pos.dot(relative_vel)) / (relative_speed * relative_speed);

    // 只考虑未来的碰撞
    if (t_cpa < 0.0f) {
        t_cpa = 0.0f;
    }

    // 限制预测时间
    if (t_cpa > _config.prediction_time) {
        t_cpa = _config.prediction_time;
    }

    // 计算CPA时的距离
    matrix::Vector3f pos_at_cpa = relative_pos + relative_vel * t_cpa;
    float distance_at_cpa = pos_at_cpa.norm();

    time_to_collision = t_cpa;
    min_distance = distance_at_cpa;

    // 判断是否会碰撞
    bool will_collide = (distance_at_cpa < _config.collision_threshold);

    // 额外检查：如果是相向而行（对冲），降低阈值
    float dot_product = relative_pos.dot(relative_vel);
    // 提高对冲判断阈值：只有真正高速对冲且距离很近时才判断为碰撞
    if (dot_product < -0.3f && current_distance < 5.0f && relative_speed > 1.0f) {
        // 对冲情况，更严格的碰撞判断（提高阈值，避免误判）
        will_collide = (distance_at_cpa < _config.collision_threshold * 1.2f);
    }

    return will_collide;
}

matrix::Vector3f FormationPlanner::generate_detour_waypoint(
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& target_pos,
    const matrix::Vector3f& conflict_pos)
{
    // 计算从当前位置到目标的向量
    matrix::Vector3f to_target = target_pos - current_pos;
    to_target(2) = 0.0f;  // 只考虑XY平面
    
    if (to_target.norm() < 0.5f) {
        return target_pos;  // 已经很接近目标
    }

    // 计算到冲突点的向量
    matrix::Vector3f to_conflict = conflict_pos - current_pos;
    to_conflict(2) = 0.0f;

    // 计算路径中点
    matrix::Vector3f path_midpoint = current_pos + to_target * 0.5f;

    // 计算顺时针垂直方向（右侧绕行）
    matrix::Vector3f perpendicular(to_target(1), -to_target(0), 0.0f);
    
    // 判断冲突点在路径的哪一侧
    float cross_product = to_target(0) * to_conflict(1) - to_target(1) * to_conflict(0);
    
    // 如果冲突点在左侧，向右绕；如果在右侧，向左绕
    if (cross_product > 0.0f) {
        // 冲突点在左侧，向右（顺时针）绕行
        perpendicular = perpendicular.normalized() * _config.detour_distance;
    } else {
        // 冲突点在右侧，向左（逆时针）绕行
        perpendicular = perpendicular.normalized() * (-_config.detour_distance);
    }

    // 生成绕行航点：路径中点 + 垂直偏移
    matrix::Vector3f waypoint = path_midpoint + perpendicular;
    waypoint(2) = current_pos(2);  // 保持相同高度

    // 确保航点不会太靠近冲突飞机
    matrix::Vector3f to_conflict_from_waypoint = conflict_pos - waypoint;
    to_conflict_from_waypoint(2) = 0.0f;
    float distance_to_conflict = to_conflict_from_waypoint.norm();

    if (distance_to_conflict < _config.detour_distance) {
        // 如果航点太靠近冲突点，增加偏移
        waypoint = path_midpoint + perpendicular * 2.0f;
    }

    return waypoint;
}

