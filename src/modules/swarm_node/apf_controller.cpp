/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file apf_controller.cpp
 * @author PX4 Development Team
 * @brief 简化APF避撞控制器实现 - 只产生斥力避免碰撞
 */

#include "apf_controller.hpp"
#include <cmath>
#include <px4_platform_common/log.h>

matrix::Vector3f APFController::calculate_avoidance_vector(
    const matrix::Vector3f& current_pos,
    const OtherVehiclePosition* other_aircraft,
    int max_aircraft_count,
    uint8_t current_vehicle_id)
{
    // 重置避撞结果
    _last_result = AvoidanceResult{};

    if (!_config.enable_avoidance) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    matrix::Vector3f total_avoidance_vector(0.0f, 0.0f, 0.0f);
    _last_result.min_distance = 999.0f;

    // 遍历所有其他飞机
    for (int i = 0; i < max_aircraft_count; i++) {
        const OtherVehiclePosition& aircraft = other_aircraft[i];

        // 跳过无效数据和自己
        if (!aircraft.valid || aircraft.mavid == current_vehicle_id) {
            continue;
        }

  
        // 🚀 只考虑XY平面避撞，忽略Z方向高度差
        matrix::Vector3f obstacle_pos(aircraft.x, aircraft.y, current_pos(2)); // 保持相同高度

        // 计算XY平面距离和方向（忽略高度差）
        matrix::Vector3f relative_pos_xy = current_pos - obstacle_pos;
        // 只使用XY距离，忽略Z分量
        float distance_xy = sqrtf(relative_pos_xy(0)*relative_pos_xy(0) + relative_pos_xy(1)*relative_pos_xy(1));

        // 更新最小XY距离
        if (distance_xy < _last_result.min_distance) {
            _last_result.min_distance = distance_xy;
        }

        // 只在避障范围内处理
        if (distance_xy < _config.max_avoidance_distance && distance_xy > 0.1f) {

            // 1. 计算径向斥力（远离障碍物）- 仅XY方向
            matrix::Vector3f radial_force = calculate_radial_repulsive_force_2d(
                obstacle_pos, current_pos, distance_xy);

            // 2. 构造障碍物速度向量 - 仅XY方向
            matrix::Vector3f obstacle_vel(aircraft.vx, aircraft.vy, 0.0f); // 忽略Z速度

            // 3. 计算顺时针横向切力（侧向避撞）- 仅XY方向
            matrix::Vector3f tangential_force = calculate_clockwise_tangential_force_2d(
                obstacle_pos, current_pos, obstacle_vel, distance_xy);

            // 4. 合成避撞力 - 仅XY方向，Z方向为0
            matrix::Vector3f total_force = radial_force + tangential_force;

            // 5. 添加到总避撞向量
            total_avoidance_vector += total_force;

            // 6. 更新统计信息
            _last_result.repulsive_vector += radial_force;
            _last_result.tangential_vector += tangential_force;
            _last_result.avoided_aircraft_count++;

            // 7. 检查是否需要紧急避撞
            if (distance_xy < _config.danger_radius) {
                _last_result.emergency_avoidance = true;
                PX4_WARN("[APF紧急避撞] 从机%d: 与飞机%d距离%.2f < 危险半径%.2f",
                         current_vehicle_id, aircraft.mavid, (double)distance_xy, (double)_config.danger_radius);
            }

            // 8. 调试输出
            const char* aircraft_type = aircraft.is_leader ? "主机" : "从机";
            PX4_INFO("[APF避撞] 从机%d: 避开%s%d 距离=%.2f 径向力=(%.2f,%.2f) 切力=(%.2f,%.2f) 总力=(%.2f,%.2f)",
                     current_vehicle_id, aircraft_type, aircraft.mavid, (double)distance_xy,
                     (double)radial_force(0), (double)radial_force(1),
                     (double)tangential_force(0), (double)tangential_force(1),
                     (double)total_force(0), (double)total_force(1));
        }
    }

    // 🔥 关键修改：在紧急情况下不限制避撞力，确保能够有效避让
    if (!_last_result.emergency_avoidance) {
        // 正常情况下限制避撞力
        total_avoidance_vector = limit_vector(total_avoidance_vector, _config.max_avoidance_force);
    } else {
        // 紧急情况下允许更大的避撞力（2倍）
        total_avoidance_vector = limit_vector(total_avoidance_vector, _config.max_avoidance_force * 2.0f);
        PX4_WARN("[APF紧急] 允许更大避撞力: %.2f m/s", (double)(_config.max_avoidance_force * 2.0f));
    }
    _last_result.avoidance_vector = total_avoidance_vector;

    // 输出避撞结果总结
    if (_last_result.avoided_aircraft_count > 0) {
        PX4_INFO("[APF结果] 从机%d: 避开%d架飞机 最近距离=%.2f 总避撞向量=(%.2f,%.2f,%.2f) %s",
                 current_vehicle_id, _last_result.avoided_aircraft_count, (double)_last_result.min_distance,
                 (double)total_avoidance_vector(0), (double)total_avoidance_vector(1), (double)total_avoidance_vector(2),
                 _last_result.emergency_avoidance ? "紧急避撞" : "正常避撞");
    }

    return total_avoidance_vector;
}

matrix::Vector3f APFController::calculate_radial_repulsive_force(
    const matrix::Vector3f& obstacle_pos,
    const matrix::Vector3f& current_pos,
    float distance)
{
    matrix::Vector3f direction = current_pos - obstacle_pos;

    if (distance < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    direction = direction.normalized();

    float repulsive_magnitude;

    if (distance < _config.danger_radius) {
        // 危险范围内，强烈斥力
        repulsive_magnitude = _config.repulsive_gain *
                             (1.0f/_config.danger_radius - 1.0f/distance) * 2.0f;
    } else if (distance < _config.safety_radius) {
        // 安全范围内，中等斥力
        repulsive_magnitude = _config.repulsive_gain *
                             (1.0f/distance - 1.0f/_config.safety_radius);
    } else if (distance < _config.max_avoidance_distance) {
        // 避障范围内，渐弱斥力
        float normalized_distance = (distance - _config.safety_radius) /
                                   (_config.max_avoidance_distance - _config.safety_radius);
        repulsive_magnitude = _config.repulsive_gain * 0.1f * (1.0f - normalized_distance);
    } else {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    return direction * repulsive_magnitude;
}

matrix::Vector3f APFController::calculate_clockwise_tangential_force(
    const matrix::Vector3f& obstacle_pos,
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& obstacle_vel,
    float distance)
{
    if (distance < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    // 1. 计算相对位置向量（从障碍物指向自己）
    matrix::Vector3f relative_pos = current_pos - obstacle_pos;
    relative_pos = relative_pos.normalized();

    // 2. 计算相对速度向量
    matrix::Vector3f relative_vel = -obstacle_vel; // 假设本机静止，障碍物运动

    // 3. 计算顺时针切线方向
    matrix::Vector3f tangential_direction = get_clockwise_perpendicular(relative_pos);

    // 4. 计算相对速度在切线方向的投影
    float tangential_component = relative_vel.dot(tangential_direction);

    // 5. 计算切线力大小（距离越近，切线力越大；相对速度越大，切线力越大）
    float distance_factor = 1.0f - (distance / _config.max_avoidance_distance);
    distance_factor = math::max(distance_factor, 0.0f);

    float velocity_factor = math::min(fabs(tangential_component) / 2.0f, 1.0f);

    float tangential_magnitude = _config.tangential_gain * distance_factor * velocity_factor;

    // 6. 顺时针方向为正
    if (tangential_component < 0) {
        tangential_magnitude = -tangential_magnitude;
    }

    return tangential_direction * tangential_magnitude;
}

bool APFController::needs_avoidance(
    const OtherVehiclePosition* other_aircraft,
    int max_aircraft_count,
    const matrix::Vector3f& current_pos,
    uint8_t current_vehicle_id)
{
    if (!_config.enable_avoidance) {
        return false;
    }

    for (int i = 0; i < max_aircraft_count; i++) {
        const OtherVehiclePosition& aircraft = other_aircraft[i];

        if (!aircraft.valid || aircraft.mavid == current_vehicle_id) {
            continue;
        }

        if (!_config.enable_leader_avoidance && aircraft.is_leader) {
            continue;
        }

        matrix::Vector3f obstacle_pos(aircraft.x, aircraft.y, aircraft.z);
        matrix::Vector3f relative_pos = current_pos - obstacle_pos;
        float distance = relative_pos.norm();

        if (distance < _config.safety_radius) {
            return true;
        }
    }

    return false;
}

matrix::Vector3f APFController::calculate_radial_repulsive_force_2d(
    const matrix::Vector3f& obstacle_pos,
    const matrix::Vector3f& current_pos,
    float distance_xy)
{
    if (distance_xy < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    // 超出避障范围，不产生斥力
    if (distance_xy >= _config.max_avoidance_distance) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    // 只计算XY方向的径向斥力
    matrix::Vector3f direction_xy = current_pos - obstacle_pos;
    direction_xy(2) = 0.0f;
    direction_xy = direction_xy.normalized();

    // ⭐ TPY立方衰减公式：远处快速衰减，近处急剧增强
    // s = (max_distance - distance) / max_distance，归一化到[0,1]
    // s^3 = 立方增益，让远处几乎无力，近处极强
    float s = (_config.max_avoidance_distance - distance_xy) / _config.max_avoidance_distance;
    s = fmaxf(0.0f, fminf(1.0f, s));  // 限制在[0,1]
    s = s * s * s;  // 立方增益

    // 基础斥力公式：增益 × 立方因子 × (范围-距离) / 距离²
    float repulsive_magnitude = _config.repulsive_gain * s * 
                                (_config.max_avoidance_distance - distance_xy) / 
                                (distance_xy * distance_xy);

    matrix::Vector3f force_xy = direction_xy * repulsive_magnitude;
    force_xy(2) = 0.0f;

    return force_xy;
}

matrix::Vector3f APFController::calculate_clockwise_tangential_force_2d(
    const matrix::Vector3f& obstacle_pos,
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& obstacle_vel,
    float distance_xy)
{
    if (distance_xy < 0.1f || distance_xy >= _config.max_avoidance_distance) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    // 1. 计算XY平面相对位置向量（从障碍物指向自己）
    matrix::Vector3f relative_pos_xy = current_pos - obstacle_pos;
    relative_pos_xy(2) = 0.0f;

    if (relative_pos_xy.norm() < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    matrix::Vector3f dir_xy = relative_pos_xy.normalized();

    // 2. 计算右侧顺时针方向（右侧通行规则）
    matrix::Vector3f tangential_dir;
    tangential_dir(0) = dir_xy(1);   // 顺时针旋转90度: (x,y) -> (y,-x)
    tangential_dir(1) = -dir_xy(0);
    tangential_dir(2) = 0.0f;

    // 3. ⭐ 计算接近速度（closing speed）
    // closing_speed > 0 表示两机正在接近
    float closing_speed = -(obstacle_vel(0) * dir_xy(0) + obstacle_vel(1) * dir_xy(1));

    // 4. ⭐ 基于距离的切向力基础倍数（TPY方法）
    float base_scale;
    if (distance_xy < 1.5f) {
        base_scale = 6.0f;
    } else if (distance_xy < 2.5f) {
        base_scale = 4.0f;
    } else if (distance_xy < _config.safety_radius) {
        base_scale = 3.0f;
    } else {
        base_scale = 2.0f;
    }

    // 5. ⭐ 速度增益：接近速度越快，切向力越强（TPY方法）
    float speed_boost = 1.0f;
    if (closing_speed > 0.1f) {
        // 接近时的速度增益，限制在 [1.0, 2.5]
        speed_boost = fmaxf(1.0f, fminf(1.0f + 0.3f * closing_speed, 2.5f));
    }

    // 6. 计算切向力大小
    // 使用立方衰减因子保持一致性
    float s = (_config.max_avoidance_distance - distance_xy) / _config.max_avoidance_distance;
    s = fmaxf(0.0f, fminf(1.0f, s));
    s = s * s * s;  // 立方增益

    float tangential_magnitude = _config.tangential_gain * s * base_scale * speed_boost;

    // 7. 返回右侧切向力（所有飞机统一右侧通行）
    return tangential_dir * tangential_magnitude;
}

matrix::Vector3f APFController::limit_vector(const matrix::Vector3f& vec, float max_magnitude)
{
    float magnitude = vec.norm();

    if (magnitude <= max_magnitude) {
        return vec;
    }

    return vec.normalized() * max_magnitude;
}