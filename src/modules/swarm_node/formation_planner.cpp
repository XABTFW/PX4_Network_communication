#include "formation_planner.hpp"
#include <px4_platform_common/log.h>

FormationPlanner::PlanResult FormationPlanner::plan_formation_path(
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& target_pos,
    const matrix::Vector3f& /* current_vel */,
    const OtherVehiclePosition* other_aircraft,
    int max_aircraft_count,
    uint8_t current_vehicle_id)
{
    _last_result = PlanResult{};
    _last_result.waypoint = target_pos;

    if (!_config.enable_planning) {
        return _last_result;
    }

    matrix::Vector3f to_target = target_pos - current_pos;
    float distance_to_target = to_target.norm();

    if (distance_to_target < 0.5f) {
        return _last_result;
    }

    // 假设以2m/s速度前往目标
    matrix::Vector3f my_velocity = to_target.normalized() * 2.0f;

    float min_time_to_collision = 999.0f;
    int conflict_aircraft = -1;
    matrix::Vector3f conflict_position;

    for (int i = 0; i < max_aircraft_count; i++) {
        // 使用mavid比较，而非数组索引
        if (!other_aircraft[i].valid || other_aircraft[i].mavid == current_vehicle_id) {
            continue;
        }

        matrix::Vector3f other_pos(other_aircraft[i].x, other_aircraft[i].y, current_pos(2));
        matrix::Vector3f other_vel(other_aircraft[i].vx, other_aircraft[i].vy, 0.0f);

        // 距离超过检测阈值则跳过
        matrix::Vector3f relative_pos = other_pos - current_pos;
        relative_pos(2) = 0.0f;
        float current_distance = relative_pos.norm();

        if (current_distance > _config.max_detection_distance) {
            continue;
        }

        float time_to_collision = 999.0f;
        float min_distance = 999.0f;
        bool will_collide = predict_collision(current_pos, my_velocity, other_pos, other_vel,
                                              time_to_collision, min_distance);

        if (will_collide && time_to_collision < min_time_to_collision) {
            min_time_to_collision = time_to_collision;
            conflict_aircraft = other_aircraft[i].mavid;
            conflict_position = other_pos;
            _last_result.direct_path_safe = false;

            PX4_WARN("[路径规划] 从机%d: 检测到与飞机%d碰撞风险 t=%.1fs d=%.2fm",
                     current_vehicle_id, conflict_aircraft, (double)time_to_collision, (double)min_distance);
        }
    }

    // 检测到碰撞则生成绕行航点
    if (!_last_result.direct_path_safe && conflict_aircraft >= 0) {
        _last_result.needs_detour = true;
        _last_result.conflict_aircraft_id = conflict_aircraft;
        _last_result.time_to_collision = min_time_to_collision;
        _last_result.waypoint = generate_detour_waypoint(current_pos, target_pos, conflict_position);

        PX4_WARN("[路径规划] 从机%d: 绕行航点=(%.2f,%.2f) 避开飞机%d",
                 current_vehicle_id, (double)_last_result.waypoint(0),
                 (double)_last_result.waypoint(1), conflict_aircraft);
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
    // 相对位置和速度（XY平面）
    matrix::Vector3f relative_pos(other_pos(0) - my_pos(0), other_pos(1) - my_pos(1), 0.0f);
    matrix::Vector3f relative_vel(other_vel(0) - my_vel(0), other_vel(1) - my_vel(1), 0.0f);

    float current_distance = relative_pos.norm();
    float relative_speed = relative_vel.norm();

    if (relative_speed < 0.1f) {
        time_to_collision = 999.0f;
        min_distance = current_distance;
        return false;
    }

    // 计算最近接近时间 (CPA)
    float t_cpa = -(relative_pos.dot(relative_vel)) / (relative_speed * relative_speed);
    t_cpa = fmaxf(0.0f, fminf(t_cpa, _config.prediction_time));

    // CPA时的距离
    matrix::Vector3f pos_at_cpa = relative_pos + relative_vel * t_cpa;
    float distance_at_cpa = pos_at_cpa.norm();

    time_to_collision = t_cpa;
    min_distance = distance_at_cpa;

    float threshold = _config.collision_threshold;

    // 对冲情况：相向而行时使用更严格的阈值（更小）
    if (current_distance > 0.1f && relative_speed > 1.0f && current_distance < 5.0f) {
        float cos_angle = relative_pos.normalized().dot(relative_vel.normalized());
        if (cos_angle < -0.3f) {
            threshold *= 0.8f;  // 对冲时更严格
        }
    }

    return distance_at_cpa < threshold;
}

matrix::Vector3f FormationPlanner::generate_detour_waypoint(
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& target_pos,
    const matrix::Vector3f& conflict_pos)
{
    matrix::Vector3f to_target = target_pos - current_pos;
    to_target(2) = 0.0f;

    if (to_target.norm() < 0.5f) {
        return target_pos;
    }

    matrix::Vector3f to_conflict = conflict_pos - current_pos;
    to_conflict(2) = 0.0f;

    // 路径中点
    matrix::Vector3f midpoint = current_pos + to_target * 0.5f;

    // 垂直方向（右侧）
    matrix::Vector3f perpendicular(to_target(1), -to_target(0), 0.0f);
    float perp_norm = perpendicular.norm();

    if (perp_norm < 0.1f) {
        perpendicular = matrix::Vector3f(1.0f, 0.0f, 0.0f);
    } else {
        perpendicular = perpendicular.normalized();
    }

    // 根据冲突点位置决定绕行方向
    float cross = to_target(0) * to_conflict(1) - to_target(1) * to_conflict(0);
    float direction = (cross > 0.0f) ? 1.0f : -1.0f;  // 左侧向右绕，右侧向左绕

    matrix::Vector3f waypoint = midpoint + perpendicular * (_config.detour_distance * direction);
    waypoint(2) = current_pos(2);

    // 确保航点与冲突点保持足够距离
    matrix::Vector3f wp_to_conflict = conflict_pos - waypoint;
    wp_to_conflict(2) = 0.0f;

    if (wp_to_conflict.norm() < _config.detour_distance) {
        waypoint = midpoint + perpendicular * (_config.detour_distance * 2.0f * direction);
        waypoint(2) = current_pos(2);
    }

    return waypoint;
}

