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
 * @file apf_controller.hpp
 * @author PX4 Development Team
 * @brief 简化版APF避撞控制器，只产生斥力避免碰撞
 *
 * 此控制器实现纯斥力避撞算法：
 * - 顺时针横向侧切力避免碰撞
 * - 斥力从所有飞机（包括主机）
 * - 不产生引力，不影响主从跟随
 * - 可配置安全参数
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include "position_sharing.hpp"

/**
 * @class APFController
 * @brief 简化APF避撞控制器，只产生斥力
 */
class APFController {
public:
    /**
     * @brief APF配置参数
     */
    struct APFConfig {
        float safety_radius = 2.5f;              // 安全半径 (米) - 适配7米编队
        float danger_radius = 1.2f;              // 危险半径 (米) - 适配7米编队
        float max_avoidance_distance = 5.0f;      // 最大避障距离 (米) - 适配7米编队
        float repulsive_gain = 2.5f;             // 斥力增益
        float tangential_gain = 2.0f;            // 横向切力增益
        float max_avoidance_force = 3.0f;        // 最大避撞力 (米/秒)
        bool enable_avoidance = true;            // 启用避撞
        bool enable_leader_avoidance = true;     // 是否避开主机
        bool clockwise_tangential = true;       // 顺时针横向避撞
    };

    /**
     * @brief 避撞结果信息
     */
    struct AvoidanceResult {
        matrix::Vector3f avoidance_vector{0.0f, 0.0f, 0.0f};  // 总避撞向量
        matrix::Vector3f repulsive_vector{0.0f, 0.0f, 0.0f}; // 斥力向量
        matrix::Vector3f tangential_vector{0.0f, 0.0f, 0.0f}; // 横向切力向量
        int avoided_aircraft_count = 0;                        // 避开的飞机数量
        float min_distance = 999.0f;                          // 最近距离
        bool emergency_avoidance = false;                     // 紧急避撞标志
    };

    APFController() = default;
    ~APFController() = default;

    /**
     * @brief 设置APF配置参数
     * @param config APF配置
     */
    void set_config(const APFConfig& config) { _config = config; }

    /**
     * @brief 获取当前配置
     * @return 当前配置
     */
    const APFConfig& get_config() const { return _config; }

    /**
     * @brief 计算避撞向量（仅斥力，不修改目标位置）
     * @param current_pos 当前位置 (局部坐标系)
     * @param other_aircraft 其他飞机位置数组
     * @param max_aircraft_count 最大飞机数量
     * @param current_vehicle_id 本机ID
     * @return 避撞向量 (需要加到速度指令中)
     */
    matrix::Vector3f calculate_avoidance_vector(
        const matrix::Vector3f& current_pos,
        const OtherVehiclePosition* other_aircraft,
        int max_aircraft_count,
        uint8_t current_vehicle_id
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
    APFConfig _config;
    AvoidanceResult _last_result{};


    /**
     * @brief 限制向量大小
     * @param vec 原向量
     * @param max_magnitude 最大大小
     * @return 限制后的向量
     */
    matrix::Vector3f limit_vector(const matrix::Vector3f& vec, float max_magnitude);

    /**
     * @brief 计算2D径向斥力向量 (仅XY平面)
     * @param obstacle_pos 障碍物位置
     * @param current_pos 当前位置
     * @param distance_xy XY平面距离
     * @return 2D径向斥力向量
     */
    matrix::Vector3f calculate_radial_repulsive_force_2d(
        const matrix::Vector3f& obstacle_pos,
        const matrix::Vector3f& current_pos,
        float distance_xy
    );

    /**
     * @brief 计算2D顺时针横向切力向量 (仅XY平面)
     * @param obstacle_pos 障碍物位置
     * @param current_pos 当前位置
     * @param obstacle_vel 障碍物速度
     * @param distance_xy XY平面距离
     * @return 2D横向切力向量
     */
    matrix::Vector3f calculate_clockwise_tangential_force_2d(
        const matrix::Vector3f& obstacle_pos,
        const matrix::Vector3f& current_pos,
        const matrix::Vector3f& obstacle_vel,
        float distance_xy
    );
};