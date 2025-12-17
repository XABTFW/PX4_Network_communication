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
 * @file velocity_obstacle_controller.hpp
 * @brief 基于速度障碍（Velocity Obstacle）的避撞控制器
 *
 * 此控制器实现速度感知的避撞算法：
 * - 基于速度障碍方法，考虑相对速度
 * - 将避撞向量融合为安全的目标速度
 * - 支持高速状态下的稳健避撞
 * - 输出位置修正量而非直接速度指令
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include "position_sharing.hpp"

/**
 * @class VelocityObstacleController
 * @brief 基于速度障碍的避撞控制器
 */
class VelocityObstacleController {
public:
    /**
     * @brief 速度障碍配置参数
     */
    struct VOConfig {
        float safety_radius = 2.5f;              // 安全半径 (米)
        float danger_radius = 1.2f;              // 危险半径 (米)
        float max_avoidance_distance = 7.0f;     // 最大避障距离 (米) - 硬性上限，超过7米不触发任何避撞
        float repulsive_gain = 2.5f;              // 斥力增益
        float tangential_gain = 2.0f;             // 横向切力增益
        float max_avoidance_force = 3.0f;        // 最大避撞力 (米/秒)
        float max_safe_velocity = 10.0f;         // 最大安全速度 (米/秒)
        float velocity_blend_factor = 0.7f;       // 速度融合因子 [0,1]
        bool enable_avoidance = true;             // 启用避撞
        bool enable_leader_avoidance = true;      // 是否避开主机
        bool clockwise_tangential = true;        // 顺时针横向避撞
    };

    /**
     * @brief 避撞结果信息
     */
    struct AvoidanceResult {
        matrix::Vector3f safe_target_velocity{0.0f, 0.0f, 0.0f};  // 安全目标速度
        matrix::Vector3f position_correction{0.0f, 0.0f, 0.0f};   // 位置修正量
        matrix::Vector3f repulsive_vector{0.0f, 0.0f, 0.0f};      // 斥力向量
        matrix::Vector3f tangential_vector{0.0f, 0.0f, 0.0f};    // 横向切力向量
        int avoided_aircraft_count = 0;                            // 避开的飞机数量
        float min_distance = 999.0f;                               // 最近距离
        float time_to_collision = 999.0f;                          // 碰撞时间 (秒)
        bool emergency_avoidance = false;                          // 紧急避撞标志
    };

    VelocityObstacleController() = default;
    ~VelocityObstacleController() = default;

    /**
     * @brief 设置配置参数
     * @param config 配置
     */
    void set_config(const VOConfig& config) { _config = config; }

    /**
     * @brief 获取当前配置
     * @return 当前配置
     */
    const VOConfig& get_config() const { return _config; }

    /**
     * @brief 计算安全的目标速度和位置修正量
     *
     * 基于速度障碍方法，考虑当前速度、目标速度和相对速度，
     * 计算安全的目标速度，并转换为位置修正量。
     *
     * @param current_pos 当前位置
     * @param current_vel 当前速度
     * @param desired_vel 期望速度（来自位置控制）
     * @param other_aircraft 其他飞机位置数组
     * @param max_aircraft_count 最大飞机数量
     * @param current_vehicle_id 本机ID
     * @param dt 时间步长 (秒)，用于位置平滑
     * @return 避撞结果
     */
    AvoidanceResult calculate_safe_velocity(
        const matrix::Vector3f& current_pos,
        const matrix::Vector3f& current_vel,
        const matrix::Vector3f& desired_vel,
        const OtherVehiclePosition* other_aircraft,
        int max_aircraft_count,
        uint8_t current_vehicle_id,
        float dt = 0.1f
    );

    /**
     * @brief 获取上次避撞的详细信息
     * @return 避撞结果信息
     */
    const AvoidanceResult& get_last_result() const { return _last_result; }

    /**
     * @brief 重置避撞状态
     */
    void reset() {
        _last_result = AvoidanceResult{};
    }


private:
    VOConfig _config;
    AvoidanceResult _last_result{};

    /**
     * @brief 计算速度障碍（Velocity Obstacle）
     *
     * 基于相对位置和相对速度，计算速度障碍区域
     *
     * @param relative_pos 相对位置（从障碍物指向自己）
     * @param relative_vel 相对速度
     * @param distance 距离
     * @return 避撞速度修正量
     */
    matrix::Vector3f calculate_velocity_obstacle(
        const matrix::Vector3f& relative_pos,
        const matrix::Vector3f& relative_vel,
        float distance
    );

    /**
     * @brief 计算2D径向斥力向量 (仅XY平面)
     */
    matrix::Vector3f calculate_radial_repulsive_force_2d(
        const matrix::Vector3f& obstacle_pos,
        const matrix::Vector3f& current_pos,
        float distance_xy
    );

    /**
     * @brief 计算2D顺时针横向切力向量 (仅XY平面)
     */
    matrix::Vector3f calculate_clockwise_tangential_force_2d(
        const matrix::Vector3f& obstacle_pos,
        const matrix::Vector3f& current_pos,
        const matrix::Vector3f& obstacle_vel,
        const matrix::Vector3f& current_vel,
        float distance_xy
    );

    /**
     * @brief 限制向量大小
     */
    matrix::Vector3f limit_vector(const matrix::Vector3f& vec, float max_magnitude);

    /**
     * @brief 计算时间到碰撞（Time To Collision）
     */
    float calculate_ttc(
        const matrix::Vector3f& relative_pos,
        const matrix::Vector3f& relative_vel
    );
};

