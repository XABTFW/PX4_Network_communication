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
 * @file velocity_obstacle_controller.cpp
 * @brief 基于速度障碍（Velocity Obstacle）的避撞控制器实现
 */

#include "velocity_obstacle_controller.hpp"
#include <cmath>
#include <px4_platform_common/log.h>

VelocityObstacleController::AvoidanceResult
VelocityObstacleController::calculate_safe_velocity(
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& current_vel,
    const matrix::Vector3f& desired_vel,
    const OtherVehiclePosition* other_aircraft,
    int max_aircraft_count,
    uint8_t current_vehicle_id,
    float dt)
{
    // 重置避撞结果
    _last_result = AvoidanceResult{};
    _last_result.safe_target_velocity = desired_vel;  // 初始化为期望速度
    _last_result.min_distance = 999.0f;
    _last_result.time_to_collision = 999.0f;

    if (!_config.enable_avoidance) {
        // 无避撞：直接返回期望速度
        _last_result.position_correction = desired_vel * dt;
        return _last_result;
    }

    matrix::Vector3f total_avoidance_force(0.0f, 0.0f, 0.0f);
    float min_ttc = 999.0f;

    // 计算当前速度的XY分量（在循环外定义，后续会用到）
    matrix::Vector3f current_vel_xy(current_vel(0), current_vel(1), 0.0f);
    float current_speed = current_vel_xy.norm();

    // 根据速度动态调整避撞策略
    bool is_high_speed = current_speed > 2.0f;
    bool is_low_speed = current_speed < 1.0f;

    // 遍历所有其他飞机
    for (int i = 0; i < max_aircraft_count; i++) {
        const OtherVehiclePosition& aircraft = other_aircraft[i];

        // 跳过无效数据和自己
        if (!aircraft.valid || aircraft.mavid == current_vehicle_id) {
            continue;
        }

        // 检查是否应该避开主机（根据配置）
        if (!_config.enable_leader_avoidance && aircraft.is_leader) {
            continue;
        }

        // 避撞优先级：已到位的飞机优先级更高
        bool other_at_target = aircraft.at_target;
        bool i_am_at_target = (current_speed < 0.5f);

        // 我方已到位，对方未到位 → 跳过，让对方来避让
        if (i_am_at_target && !other_at_target) {
            continue;
        }

        // 只考虑XY平面避撞，忽略Z方向高度差
        matrix::Vector3f obstacle_pos(aircraft.x, aircraft.y, current_pos(2));
        matrix::Vector3f obstacle_vel(aircraft.vx, aircraft.vy, 0.0f);

        // 计算XY平面相对位置和速度
        matrix::Vector3f relative_pos_xy = current_pos - obstacle_pos;
        relative_pos_xy(2) = 0.0f;
        float distance_xy = sqrtf(relative_pos_xy(0)*relative_pos_xy(0) + relative_pos_xy(1)*relative_pos_xy(1));

        // 计算相对速度（仅XY）
        matrix::Vector3f relative_vel_xy = current_vel_xy - obstacle_vel;

        // 更新最小距离
        if (distance_xy < _last_result.min_distance) {
            _last_result.min_distance = distance_xy;
        }

        // 避障范围检测
        float effective_max_distance = _config.max_avoidance_distance;

        if (distance_xy < effective_max_distance && distance_xy > 0.1f) {
            // 计算时间到碰撞（TTC）
            float ttc = calculate_ttc(relative_pos_xy, relative_vel_xy);
            if (ttc < min_ttc) {
                min_ttc = ttc;
                _last_result.time_to_collision = ttc;
            }

            // 判断飞机是否在路径上（会碰撞）
            bool on_path = false;
            float path_collision_risk = 0.0f;

            matrix::Vector3f desired_vel_xy(desired_vel(0), desired_vel(1), 0.0f);
            float effective_speed = fmaxf(desired_vel_xy.norm(), current_vel_xy.norm());
            float ttc_threshold = (effective_speed > 1.0f) ? 2.0f : 3.0f;

            if (ttc > 0.0f && ttc < ttc_threshold && effective_speed > 0.3f && distance_xy > 0.1f) {
                matrix::Vector3f relative_dir = relative_pos_xy.normalized();
                float closing_speed = -relative_dir.dot(relative_vel_xy);
                float closing_speed_threshold = (effective_speed > 1.0f) ? 0.5f : 0.2f;

                if (closing_speed > closing_speed_threshold) {
                    // 预测路径
                    float min_future_distance = distance_xy;
                    float critical_time = 0.0f;

                    for (float t = 0.1f; t <= 1.0f; t += 0.2f) {
                        matrix::Vector3f future_my_pos = current_pos + desired_vel_xy * t;
                        matrix::Vector3f future_obstacle_pos = obstacle_pos + obstacle_vel * t;
                        future_my_pos(2) = 0.0f;
                        future_obstacle_pos(2) = 0.0f;
                        float future_dist = (future_my_pos - future_obstacle_pos).norm();
                        if (future_dist < min_future_distance) {
                            min_future_distance = future_dist;
                            critical_time = t;
                        }
                    }

                    // 主机使用更大的危险半径
                    float effective_danger_radius = aircraft.is_leader ?
                        (_config.danger_radius * 3.5f) : _config.danger_radius;

                    if (min_future_distance < effective_danger_radius && critical_time < 1.0f) {
                        on_path = true;
                        path_collision_risk = math::constrain(
                            1.0f - (min_future_distance / effective_danger_radius), 0.0f, 1.0f);
                    }
                }
            }

            // 使用速度障碍方法计算避撞力
            matrix::Vector3f vo_force = calculate_velocity_obstacle(
                relative_pos_xy, relative_vel_xy, distance_xy);

            // 同时计算传统APF力（作为补充）
            matrix::Vector3f radial_force = calculate_radial_repulsive_force_2d(
                obstacle_pos, current_pos, distance_xy);
            matrix::Vector3f tangential_force = calculate_clockwise_tangential_force_2d(
                obstacle_pos, current_pos, obstacle_vel, current_vel_xy, distance_xy);

            // 根据速度和路径状态动态调整避撞力
            float force_scale = 1.0f;

            // 对方已到位时的避撞力增强因子
            float at_target_boost = 1.0f;
            if (other_at_target && !i_am_at_target) {
                at_target_boost = 2.0f;  // 我方需要主动避让
            } else if (i_am_at_target && other_at_target) {
                at_target_boost = 0.2f;  // 双方都已到位，减弱避撞
            }

            // 基础速度因子
            float base_speed_scale;
            if (is_low_speed) {
                base_speed_scale = 0.1f + 0.2f * (current_speed / 1.0f);
            } else if (is_high_speed) {
                base_speed_scale = 1.0f + math::min(1.0f, (current_speed - 2.0f) / 3.0f);
            } else {
                base_speed_scale = 0.3f + 0.7f * ((current_speed - 1.0f) / 1.0f);
            }
            base_speed_scale *= at_target_boost;

            if (aircraft.is_leader) {
                // 主机避撞：增强避撞力
                const float leader_boost = 4.0f;
                float leader_speed_scale = fmaxf(0.8f, base_speed_scale);

                if (on_path) {
                    force_scale = (5.0f + 3.0f * path_collision_risk) * leader_speed_scale * leader_boost;
                } else {
                    float distance_factor = distance_xy / effective_max_distance;
                    force_scale = (2.0f + 2.0f * (1.0f - distance_factor)) * leader_speed_scale * leader_boost;
                }

                // 主机近距离紧急避撞
                if (distance_xy < 4.0f) {
                    _last_result.emergency_avoidance = true;
                    force_scale *= 2.0f;
                }
            } else {
                // 从机避撞
                if (on_path) {
                    force_scale = (1.0f + 1.0f * path_collision_risk) * base_speed_scale;
                } else {
                    float distance_factor = distance_xy / effective_max_distance;
                    force_scale = is_low_speed ?
                        (0.02f + 0.08f * (1.0f - distance_factor)) :
                        (0.05f + 0.15f * (1.0f - distance_factor)) * base_speed_scale;
                }
            }

            // 融合速度障碍力和APF力
            const float radial_scale = 0.3f;
            const float tangential_scale = 2.5f;
            const float vo_weight = 0.7f;

            matrix::Vector3f combined_force = (vo_force * vo_weight +
                (radial_force * radial_scale + tangential_force * tangential_scale) * (1.0f - vo_weight)) * force_scale;

            total_avoidance_force += combined_force;

            // 更新统计信息
            _last_result.repulsive_vector += radial_force;
            _last_result.tangential_vector += tangential_force;
            _last_result.avoided_aircraft_count++;

            // 紧急避撞触发条件
            float emergency_ttc_threshold = is_high_speed ? 5.0f : (is_low_speed ? 2.0f : 3.5f);
            float emergency_distance_threshold = is_high_speed ? _config.danger_radius * 1.5f :
                                                 (is_low_speed ? _config.danger_radius * 0.7f : _config.danger_radius);

            // 主机使用更宽松的触发条件
            if (aircraft.is_leader) {
                emergency_ttc_threshold *= 2.0f;
                emergency_distance_threshold = 5.0f;
            }

            if (distance_xy < emergency_distance_threshold || (ttc > 0.0f && ttc < emergency_ttc_threshold)) {
                _last_result.emergency_avoidance = true;
            }
        }
    }

    // 根据速度动态调整避撞力限制
    float base_max_force = _config.max_avoidance_force;
    if (is_low_speed) {
        base_max_force *= 0.3f;
    } else if (is_high_speed) {
        base_max_force *= 1.5f;
    }

    if (!_last_result.emergency_avoidance) {
        total_avoidance_force = limit_vector(total_avoidance_force, base_max_force);
    } else {
        // 紧急情况下根据TTC调整避撞力倍数
        float ttc = _last_result.time_to_collision;
        float max_force_multiplier;

        if (is_high_speed) {
            max_force_multiplier = (ttc > 0.0f && ttc < 0.5f) ? 8.0f :
                                   (ttc < 1.0f) ? 6.0f : (ttc < 2.0f) ? 4.0f : 3.0f;
        } else if (is_low_speed) {
            max_force_multiplier = (ttc > 0.0f && ttc < 0.5f) ? 3.0f :
                                   (ttc < 1.0f) ? 2.0f : 1.5f;
        } else {
            max_force_multiplier = (ttc > 0.0f && ttc < 0.5f) ? 5.0f :
                                   (ttc < 1.0f) ? 4.0f : 3.0f;
        }

        total_avoidance_force = limit_vector(total_avoidance_force, base_max_force * max_force_multiplier);
    }

    // 速度融合策略
    matrix::Vector3f desired_vel_xy(desired_vel(0), desired_vel(1), 0.0f);
    matrix::Vector3f avoidance_vel_xy = total_avoidance_force;
    matrix::Vector3f safe_vel_xy;

    float blend_factor = is_high_speed ? 0.85f : (is_low_speed ? 0.3f : 0.6f);
    float ttc = _last_result.time_to_collision;

    if (_last_result.emergency_avoidance && is_high_speed && ttc < 3.0f) {
        // 高速紧急避撞：完全使用避撞速度
        safe_vel_xy = avoidance_vel_xy;
        float avoidance_speed = safe_vel_xy.norm();
        float min_avoidance_speed = 4.0f + current_speed * 0.5f;
        if (avoidance_speed < min_avoidance_speed && avoidance_speed > 0.1f) {
            safe_vel_xy = safe_vel_xy.normalized() * min_avoidance_speed;
        }
    } else if (_last_result.emergency_avoidance && is_low_speed && ttc < 1.5f) {
        // 低速紧急避撞：温和融合
        safe_vel_xy = 0.5f * (desired_vel_xy + avoidance_vel_xy * 0.5f) + 0.5f * current_vel_xy;
    } else {
        safe_vel_xy = blend_factor * (desired_vel_xy + avoidance_vel_xy) + (1.0f - blend_factor) * current_vel_xy;
    }

    // 速度限制
    float max_vel = _config.max_safe_velocity;
    if (_last_result.emergency_avoidance && is_high_speed && ttc < 3.0f) {
        max_vel = 15.0f + current_speed;
    } else if (is_low_speed) {
        max_vel = math::min(max_vel, 3.0f);
    }

    if (safe_vel_xy.norm() > max_vel) {
        safe_vel_xy = safe_vel_xy.normalized() * max_vel;
    }

    // 保持Z方向速度不变
    _last_result.safe_target_velocity(0) = safe_vel_xy(0);
    _last_result.safe_target_velocity(1) = safe_vel_xy(1);
    _last_result.safe_target_velocity(2) = desired_vel(2);

    // 位置修正量
    if (_last_result.emergency_avoidance && is_high_speed && ttc < 3.0f) {
        float correction_scale = 2.0f + 4.0f * (3.0f - ttc) / 3.0f;
        float min_correction_time = 1.0f + current_speed * 0.2f;
        _last_result.position_correction = safe_vel_xy * math::max(dt, min_correction_time) * correction_scale;
    } else if (is_low_speed) {
        _last_result.position_correction = safe_vel_xy * dt * 0.3f;
    } else {
        _last_result.position_correction = safe_vel_xy * dt;
    }
    _last_result.position_correction(2) = 0.0f;

    return _last_result;
}

matrix::Vector3f
VelocityObstacleController::calculate_velocity_obstacle(
    const matrix::Vector3f& relative_pos,
    const matrix::Vector3f& relative_vel,
    float distance)
{
    if (distance < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    // 归一化相对位置
    matrix::Vector3f relative_dir = relative_pos.normalized();

    // 计算接近速度（closing speed）
    // closing_speed > 0 表示两机正在接近
    float closing_speed = -relative_dir.dot(relative_vel);

    // 如果正在远离，不需要避撞
    if (closing_speed <= 0.0f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    // 计算时间到碰撞
    float ttc = distance / closing_speed;

    // 基于TTC和距离计算避撞力
    float force_magnitude = 0.0f;

    if (distance < _config.danger_radius) {
        // 危险范围内：强烈避撞
        force_magnitude = _config.repulsive_gain * 3.0f * (1.0f / distance);
    } else if (distance < _config.safety_radius) {
        // 安全范围内：中等避撞
        force_magnitude = _config.repulsive_gain * 2.0f * (1.0f / distance);
    } else if (distance < _config.max_avoidance_distance) {
        // 避障范围内：渐弱避撞
        float normalized_distance = (distance - _config.safety_radius) /
                                   (_config.max_avoidance_distance - _config.safety_radius);
        force_magnitude = _config.repulsive_gain * 0.5f * (1.0f - normalized_distance);
    }

    // TTC修正：时间越短，避撞力越强
    if (ttc > 0.0f && ttc < 1.0f) {
        force_magnitude *= 1.5f + 4.5f * (1.0f - ttc);
    } else if (ttc >= 1.0f && ttc < 3.0f) {
        force_magnitude *= 1.2f + 0.3f * (3.0f - ttc) / 2.0f;
    } else if (ttc >= 3.0f && ttc < 5.0f) {
        force_magnitude *= 1.0f + 0.2f * (5.0f - ttc) / 2.0f;
    }

    // 径向避撞力
    matrix::Vector3f radial_force = relative_dir * force_magnitude * 0.3f;

    // 横向避撞力（顺时针）
    matrix::Vector3f tangential_dir(relative_dir(1), -relative_dir(0), 0.0f);
    float tangential_vel = fabsf(relative_vel.dot(tangential_dir));
    float tangential_magnitude = _config.tangential_gain * 0.75f *
                                (1.0f - distance / _config.max_avoidance_distance) *
                                (1.0f + tangential_vel / 2.0f);

    return radial_force + tangential_dir * tangential_magnitude * 2.0f;
}

matrix::Vector3f
VelocityObstacleController::calculate_radial_repulsive_force_2d(
    const matrix::Vector3f& obstacle_pos,
    const matrix::Vector3f& current_pos,
    float distance_xy)
{
    if (distance_xy < 0.1f || distance_xy >= _config.max_avoidance_distance) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    matrix::Vector3f direction_xy = current_pos - obstacle_pos;
    direction_xy(2) = 0.0f;
    float dir_magnitude = direction_xy.norm();
    if (dir_magnitude < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }
    direction_xy = direction_xy.normalized();

    // 立方衰减公式
    float s = (_config.max_avoidance_distance - distance_xy) / _config.max_avoidance_distance;
    s = fmaxf(0.0f, fminf(1.0f, s));
    s = s * s * s;

    float repulsive_magnitude = _config.repulsive_gain * s *
                                (_config.max_avoidance_distance - distance_xy) /
                                (distance_xy * distance_xy);

    matrix::Vector3f force_xy = direction_xy * repulsive_magnitude * 0.15f;
    force_xy(2) = 0.0f;
    return force_xy;
}

matrix::Vector3f
VelocityObstacleController::calculate_clockwise_tangential_force_2d(
    const matrix::Vector3f& obstacle_pos,
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& obstacle_vel,
    const matrix::Vector3f& current_vel,
    float distance_xy)
{
    if (distance_xy < 0.1f || distance_xy >= _config.max_avoidance_distance) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    matrix::Vector3f relative_pos_xy = current_pos - obstacle_pos;
    relative_pos_xy(2) = 0.0f;

    if (relative_pos_xy.norm() < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    matrix::Vector3f dir_xy = relative_pos_xy.normalized();

    // 计算顺时针方向
    matrix::Vector3f tangential_dir;
    tangential_dir(0) = dir_xy(1);
    tangential_dir(1) = -dir_xy(0);
    tangential_dir(2) = 0.0f;

    // 计算相对速度
    matrix::Vector3f relative_vel_xy = current_vel - obstacle_vel;
    relative_vel_xy(2) = 0.0f;

    // 计算接近速度
    float closing_speed = -relative_vel_xy.dot(dir_xy);

    // 基于距离和速度的切向力
    float base_scale = 1.0f;
    if (distance_xy < 1.5f) {
        base_scale = 6.0f;
    } else if (distance_xy < 2.5f) {
        base_scale = 4.0f;
    } else if (distance_xy < _config.safety_radius) {
        base_scale = 3.0f;
    } else {
        base_scale = 2.0f;
    }

    float speed_boost = 1.0f;
    if (closing_speed > 0.1f) {
        speed_boost = fmaxf(1.0f, fminf(1.0f + 0.3f * closing_speed, 2.5f));
    }

    float s = (_config.max_avoidance_distance - distance_xy) / _config.max_avoidance_distance;
    s = fmaxf(0.0f, fminf(1.0f, s));
    s = s * s * s;

    float tangential_magnitude = _config.tangential_gain * s * base_scale * speed_boost;

    return tangential_dir * tangential_magnitude * 2.0f;
}

matrix::Vector3f
VelocityObstacleController::limit_vector(const matrix::Vector3f& vec, float max_magnitude)
{
    float magnitude = vec.norm();

    if (magnitude <= max_magnitude) {
        return vec;
    }

    if (magnitude < 0.1f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    return vec.normalized() * max_magnitude;
}

float
VelocityObstacleController::calculate_ttc(
    const matrix::Vector3f& relative_pos,
    const matrix::Vector3f& relative_vel)
{
    float distance = relative_pos.norm();
    if (distance < 0.1f) {
        return 0.0f;
    }

    matrix::Vector3f relative_dir = relative_pos.normalized();
    float closing_speed = -relative_dir.dot(relative_vel);

    if (closing_speed <= 0.0f) {
        return 999.0f;  // 正在远离
    }

    return distance / closing_speed;
}

