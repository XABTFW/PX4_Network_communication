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
    
    //  计算当前速度的XY分量（在循环外定义，后续会用到）
    matrix::Vector3f current_vel_xy(current_vel(0), current_vel(1), 0.0f);

    // 遍历所有其他飞机
    for (int i = 0; i < max_aircraft_count; i++) {
        const OtherVehiclePosition& aircraft = other_aircraft[i];

        // 跳过无效数据和自己
        if (!aircraft.valid || aircraft.mavid == current_vehicle_id) {
            continue;
        }
        
        // 检查是否应该避开主机（根据配置）
        if (!_config.enable_leader_avoidance && aircraft.is_leader) {
            PX4_INFO("[跳过主机] 不避开主机%d（配置：enable_leader_avoidance=false）", aircraft.mavid);
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

        // 只在避障范围内处理
        // 修复：主机避撞使用更大的检测距离（1.5倍），确保主从避撞有效
        float effective_max_distance = aircraft.is_leader ? 
            (_config.max_avoidance_distance * 1.5f) : _config.max_avoidance_distance;
        if (distance_xy < effective_max_distance && distance_xy > 0.1f) {
            // 计算时间到碰撞（TTC）
            float ttc = calculate_ttc(relative_pos_xy, relative_vel_xy);
            if (ttc < min_ttc) {
                min_ttc = ttc;
                _last_result.time_to_collision = ttc;
            }

            //  关键：判断飞机是否在路径上（会碰撞）
            // 通过预测轨迹判断：如果未来会碰撞，说明在路径上
            bool on_path = false;  // 是否在路径上
            float path_collision_risk = 0.0f;  // 路径碰撞风险 [0, 1]
            
            // 使用期望速度方向预测路径（更准确）
            matrix::Vector3f desired_vel_xy(desired_vel(0), desired_vel(1), 0.0f);
            float desired_speed = desired_vel_xy.norm();
            
            // 减弱预测：只在TTC很短且速度很快时才预测路径
            if (ttc > 0.0f && ttc < 2.0f && desired_speed > 1.5f) {
                // TTC有效且速度较快，说明正在快速接近，可能在路径上
                // 计算相对速度在相对位置方向的投影（接近速度）
                matrix::Vector3f relative_dir = relative_pos_xy.normalized();
                float closing_speed = -relative_dir.dot(relative_vel_xy);
                
                // 提高接近速度阈值，避免误判
                if (closing_speed > 0.5f) {
                    // 正在快速接近，使用期望速度预测路径
                    // 缩短预测时间到1秒，避免过度预测
                    float min_future_distance = distance_xy;
                    float critical_time = 0.0f;
                    
                    for (float t = 0.1f; t <= 1.0f; t += 0.2f) {
                        // 使用期望速度预测本机未来位置
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
                    
                    // 提高阈值：预测最小距离 < 危险半径才说明真正在路径上
                    // 修复：主机避撞使用更大的危险半径（1.5倍），确保主从避撞更敏感
                    float effective_danger_radius = aircraft.is_leader ? 
                        (_config.danger_radius * 1.5f) : _config.danger_radius;
                    if (min_future_distance < effective_danger_radius && critical_time < 1.0f) {
                        on_path = true;
                        // 路径碰撞风险：距离越近，风险越高
                        path_collision_risk = 1.0f - (min_future_distance / effective_danger_radius);
                        path_collision_risk = math::constrain(path_collision_risk, 0.0f, 1.0f);
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

            //  根据是否在路径上调整避撞力（减弱避撞力）
            // 修复：主机避撞始终使用正常强度，不受编队状态影响
            float force_scale = 1.0f;
            if (aircraft.is_leader) {
                // 主机避撞：始终使用正常或增强的避撞力，确保主从避撞有效
                if (on_path) {
                    // 主机在路径上：使用增强避撞力（2.0-3.0倍）
                    force_scale = 2.0f + 1.0f * path_collision_risk;  // [2.0, 3.0]
                    PX4_WARN("[主机路径避撞] 主机%d在路径上！路径碰撞风险=%.2f 增强避撞力%.1fx 预测最小距离=%.2fm", 
                             aircraft.mavid, (double)path_collision_risk, (double)force_scale,
                             (double)(distance_xy * (1.0f - path_collision_risk)));
                } else {
                    // 主机不在路径上：使用正常避撞力（0.5-1.0倍），确保主机避撞始终有效
                    float distance_factor = distance_xy / _config.max_avoidance_distance;  // [0, 1]
                    force_scale = 0.5f + 0.5f * (1.0f - distance_factor);  // [0.5, 1.0]
                    PX4_INFO("[主机非路径避撞] 主机%d不在路径上，使用正常避撞力%.2fx 距离=%.2fm", 
                             aircraft.mavid, (double)force_scale, (double)distance_xy);
                }
            } else {
                // 从机避撞：根据路径状态调整避撞力
                if (on_path) {
                    // 路径上的飞机：适度增强避撞力（1.5-2.5倍），减弱以避免过度避让
                    force_scale = 1.5f + 1.0f * path_collision_risk;  // [1.5, 2.5] 从[2.0, 4.0]降低
                    PX4_WARN("[路径避撞] 飞机%d在路径上！路径碰撞风险=%.2f 增强避撞力%.1fx 预测最小距离=%.2fm", 
                             aircraft.mavid, (double)path_collision_risk, (double)force_scale,
                             (double)(distance_xy * (1.0f - path_collision_risk)));
                } else {
                    // 非路径上的飞机：大幅减弱避撞力（0.05-0.2倍），避免过度避让
                    // 距离越远，避撞力越小
                    // 注意：这里使用已经在上面声明的 effective_max_distance 变量
                    float distance_factor = distance_xy / effective_max_distance;  // [0, 1]
                    force_scale = 0.05f + 0.15f * (1.0f - distance_factor);  // [0.05, 0.2] 从[0.1, 0.3]降低
                    PX4_INFO("[非路径避撞] 飞机%d不在路径上，减弱避撞力%.2fx 距离=%.2fm", 
                             aircraft.mavid, (double)force_scale, (double)distance_xy);
                }
            }

            // 融合速度障碍力和APF力（速度障碍优先）
            // ⭐⭐⭐ 调整径向力和横向力的比例：减小径向斥力，增大横向避撞力
            float radial_scale = 0.3f;  // 径向力减小到30%（从0.5f降低）
            float tangential_scale = 2.5f;  // 横向力增大到2.5倍（从1.5f提高）
            
            float vo_weight = 0.7f;  // 速度障碍权重
            // 分离径向力和横向力，分别调整
            matrix::Vector3f adjusted_radial = radial_force * radial_scale;
            matrix::Vector3f adjusted_tangential = tangential_force * tangential_scale;
            
            matrix::Vector3f combined_force = (vo_force * vo_weight + 
                                             (adjusted_radial + adjusted_tangential) * (1.0f - vo_weight)) * force_scale;

            total_avoidance_force += combined_force;

            // 更新统计信息
            _last_result.repulsive_vector += radial_force;
            _last_result.tangential_vector += tangential_force;
            _last_result.avoided_aircraft_count++;

            //  检查是否需要紧急避撞（高速对冲时TTC很短）
            //  关键改进：更早触发紧急避撞，提高响应速度
            // 当TTC < 4.0秒或距离 < 危险半径时，触发紧急避撞（从3.0秒提高到4.0秒，更早触发）
            // 更早触发可以给避撞更多时间，提高成功率，特别是高速状态下
            // 这样可以更早地应用避撞指令，提高响应速度
            if (distance_xy < _config.danger_radius || (ttc > 0.0f && ttc < 4.0f)) {
                _last_result.emergency_avoidance = true;
            }
        }
    }

    //  限制避撞力大小（减弱避撞力，但高速时保持足够强度）
    if (!_last_result.emergency_avoidance) {
        total_avoidance_force = limit_vector(total_avoidance_force, _config.max_avoidance_force);
    } else {
        // 紧急情况下允许更大的避撞力（减弱：从6-10倍降低到4-7倍）
        // 如果TTC极短（<0.5秒），允许更大的避撞力
        float max_force_multiplier = 4.0f;  // 默认4倍（从6倍降低）
        if (_last_result.time_to_collision > 0.0f && _last_result.time_to_collision < 0.5f) {
            // 极短TTC（<0.5秒）：允许7倍避撞力（从10倍降低），确保能快速避让
            max_force_multiplier = 7.0f;
        } else if (_last_result.time_to_collision < 1.0f) {
            // 短TTC（0.5-1秒）：允许6倍避撞力（从8倍降低）
            max_force_multiplier = 6.0f;
        } else if (_last_result.time_to_collision < 1.5f) {
            // 中等TTC（1-1.5秒）：允许5倍避撞力（从7倍降低）
            max_force_multiplier = 5.0f;
        } else if (_last_result.time_to_collision < 2.0f) {
            // 较长TTC（1.5-2秒）：允许4倍避撞力（从6倍降低）
            max_force_multiplier = 4.0f;
        }
        total_avoidance_force = limit_vector(total_avoidance_force, 
                                           _config.max_avoidance_force * max_force_multiplier);
        PX4_WARN("[避撞力增强] TTC=%.2fs 避撞力倍数=%.1fx 避撞力=%.2fm/s", 
                 (double)_last_result.time_to_collision, (double)max_force_multiplier,
                 (double)total_avoidance_force.norm());
    }

    //  关键：将避撞力融合为安全的目标速度
    // 不是直接替换速度，而是与期望速度融合
    matrix::Vector3f desired_vel_xy(desired_vel(0), desired_vel(1), 0.0f);
    matrix::Vector3f avoidance_vel_xy = total_avoidance_force;

    //  速度融合：考虑当前速度和期望速度
    // 紧急情况下，更偏向避撞速度而非期望速度
    float blend_factor = _config.velocity_blend_factor;
    matrix::Vector3f safe_vel_xy;
    
    //  高速对冲时：完全忽略期望速度，只使用避撞速度
    // 更早触发：TTC < 3.0秒就完全使用避撞速度（从2.5秒提高到3.0秒，更早触发，改善高速避撞效果）
    if (_last_result.emergency_avoidance && _last_result.time_to_collision < 3.0f) {
        // 高速对冲时：完全替换为避撞速度，忽略期望速度和当前速度
        // 确保避撞效果不被期望速度抵消
        safe_vel_xy = avoidance_vel_xy;
        
        //  确保避撞速度足够大，至少6m/s（高速对冲时需要足够的速度）
        float avoidance_speed = safe_vel_xy.norm();
        if (avoidance_speed < 6.0f && avoidance_speed > 0.1f) {
            safe_vel_xy = safe_vel_xy.normalized() * 6.0f;  // 至少6m/s
            PX4_WARN("[避撞速度增强] 避撞速度%.2fm/s太小，增强到6.0m/s", (double)avoidance_speed);
        }
        
        PX4_WARN("[高速对冲] 完全使用避撞速度，忽略期望速度！避撞速度=%.2fm/s TTC=%.2fs", 
                 (double)safe_vel_xy.norm(), (double)_last_result.time_to_collision);
    } else {
        // 正常情况：融合期望速度和避撞速度
        safe_vel_xy = blend_factor * (desired_vel_xy + avoidance_vel_xy) + 
                     (1.0f - blend_factor) * current_vel_xy;
    }

    //  限制安全速度大小（高速对冲时不限制避撞速度）
    float safe_vel_magnitude = safe_vel_xy.norm();
    float max_vel = _config.max_safe_velocity;
    
    //  关键：高速对冲时，不限制避撞速度，确保避撞效果
    // 更早触发：TTC < 3.0秒就不限制（从2.5秒提高到3.0秒，更早触发，改善高速避撞效果）
    if (_last_result.emergency_avoidance && _last_result.time_to_collision < 3.0f) {
        // 高速对冲时：允许更大的避撞速度，不限制
        // 避撞速度可能达到15-20m/s，这是必要的，因为需要快速避让
        max_vel = 20.0f;  // 高速对冲时允许更大的避撞速度（提高到20m/s）
        PX4_WARN("[高速对冲] 不限制避撞速度，允许达到%.2fm/s以确保避撞效果 TTC=%.2fs", 
                 (double)max_vel, (double)_last_result.time_to_collision);
    }
    
    if (safe_vel_magnitude > max_vel) {
        safe_vel_xy = safe_vel_xy.normalized() * max_vel;
    }

    // 保持Z方向速度不变
    _last_result.safe_target_velocity(0) = safe_vel_xy(0);
    _last_result.safe_target_velocity(1) = safe_vel_xy(1);
    _last_result.safe_target_velocity(2) = desired_vel(2);  // Z方向保持期望速度

    //  将安全速度转换为位置修正量（用于位置控制）
    // 高速对冲时，需要更大的位置修正量才能有效避让，但减弱修正倍数
    // 更早触发：TTC < 3.0秒就使用增强的位置修正（从2.5秒提高到3.0秒，更早触发）
    if (_last_result.emergency_avoidance && _last_result.time_to_collision < 3.0f) {
        // 高速对冲时：基于TTC和避撞力计算位置修正量
        // 需要足够的位移才能在短时间内避开
        float ttc = _last_result.time_to_collision;
        // TTC越小，修正量越大 [2, 5]（从[3, 8]降低，减弱避撞力）
        float correction_scale = 2.0f + 3.0f * (3.0f - ttc) / 3.0f;  // ttc=0时5倍, ttc=3.0时2倍（从8倍降低到5倍）
        // 至少保证1.5秒内的位移足够大（从2秒降低到1.5秒）
        float min_correction_time = 1.5f;  // 至少1.5秒的位移
        _last_result.position_correction = safe_vel_xy * math::max(dt, min_correction_time) * correction_scale;
        PX4_WARN("[高速对冲位置修正] TTC=%.2fs 修正量=%.2fm 修正速度=%.2fm/s 倍数=%.1fx", 
                 (double)ttc, (double)_last_result.position_correction.norm(), 
                 (double)safe_vel_xy.norm(), (double)correction_scale);
    } else {
        // 正常情况：使用标准的位置修正
        _last_result.position_correction = safe_vel_xy * dt;
    }
    _last_result.position_correction(2) = 0.0f;  // Z方向不修正

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

    //  TTC修正：时间越短，避撞力越强（减弱避撞力，但保持高速时的响应）
    // 当TTC < 1秒时，需要较强避撞力，但减弱倍数
    if (ttc > 0.0f && ttc < 1.0f) {
        // 极短TTC：指数增强避撞力 [1.5, 6]（从[2, 10]降低）
        float ttc_factor = 1.5f + 4.5f * (1.0f - ttc);  // ttc=0时因子=6, ttc=1时因子=1.5（从10降低到6）
        force_magnitude *= ttc_factor;
    } else if (ttc >= 1.0f && ttc < 3.0f) {
        // 短TTC：线性增强 [1.2, 1.5]（从[1.5, 2]降低）
        float ttc_factor = 1.2f + 0.3f * (3.0f - ttc) / 2.0f;
        force_magnitude *= ttc_factor;
    } else if (ttc >= 3.0f && ttc < 5.0f) {
        // 中等TTC：轻微增强 [1, 1.2]（从[1, 1.5]降低）
        float ttc_factor = 1.0f + 0.2f * (5.0f - ttc) / 2.0f;
        force_magnitude *= ttc_factor;
    }

    // 径向避撞力减小到30%（从0.5f降低），减小径向斥力
    matrix::Vector3f radial_force = relative_dir * force_magnitude * 0.3f;

    // 计算横向避撞力（顺时针，增大）
    matrix::Vector3f tangential_dir;
    tangential_dir(0) = relative_dir(1);   // 顺时针旋转90度
    tangential_dir(1) = -relative_dir(0);
    tangential_dir(2) = 0.0f;

    // 横向力大小：基于相对速度的横向分量（增大1.5倍）
    float tangential_vel = fabsf(relative_vel.dot(tangential_dir));
    float tangential_magnitude = _config.tangential_gain * 0.75f *  // 从0.5f增加到0.75f
                                (1.0f - distance / _config.max_avoidance_distance) *
                                (1.0f + tangential_vel / 2.0f);

    matrix::Vector3f tangential_force = tangential_dir * tangential_magnitude * 2.0f;  // 增大到2.0倍（从1.5f提高）

    return radial_force + tangential_force;
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
    direction_xy = direction_xy.normalized();

    // 立方衰减公式
    float s = (_config.max_avoidance_distance - distance_xy) / _config.max_avoidance_distance;
    s = fmaxf(0.0f, fminf(1.0f, s));
    s = s * s * s;

    float repulsive_magnitude = _config.repulsive_gain * s * 
                                (_config.max_avoidance_distance - distance_xy) / 
                                (distance_xy * distance_xy);

    // ⭐⭐⭐ 径向力减小到15%（从0.2f降低），减小径向斥力
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

    // 横向力增大到2.0倍（从1.5f提高），增强横向避撞力
    return tangential_dir * tangential_magnitude * 2.0f;
}

matrix::Vector3f
VelocityObstacleController::limit_vector(const matrix::Vector3f& vec, float max_magnitude)
{
    float magnitude = vec.norm();

    if (magnitude <= max_magnitude) {
        return vec;
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

bool
VelocityObstacleController::needs_avoidance(
    const OtherVehiclePosition* other_aircraft,
    int max_aircraft_count,
    const matrix::Vector3f& current_pos,
    const matrix::Vector3f& current_vel,
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
        relative_pos(2) = 0.0f;  // 只考虑XY
        float distance = relative_pos.norm();

        if (distance < _config.safety_radius) {
            return true;
        }
    }

    return false;
}


