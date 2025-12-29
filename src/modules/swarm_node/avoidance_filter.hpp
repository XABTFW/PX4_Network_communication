/**
 * @file avoidance_filter.hpp
 * @brief 避撞过滤器 - 根据角色过滤需要避撞的目标
 *
 * 过滤规则：
 * - 头号主机：不避撞任何飞机
 * - 次要主机：只避撞其他主机
 * - 从机：避撞所有飞机
 */

#pragma once

#include "position_sharing.hpp"
#include "role_manager.hpp"

/**
 * @brief 避撞过滤器类
 *
 * 根据飞机角色过滤需要进行避撞检测的目标
 */
class AvoidanceFilter
{
public:
    AvoidanceFilter() = default;
    ~AvoidanceFilter() = default;

    /**
     * @brief 检查当前角色是否需要执行避撞
     * @param role 当前飞机角色
     * @return true: 需要执行避撞; false: 不需要
     */
    bool should_perform_avoidance(SwarmRole role) const;

    /**
     * @brief 过滤避撞目标
     * @param role 当前飞机角色
     * @param all_vehicles 所有飞机位置数组
     * @param vehicle_count 飞机数量
     * @param self_id 本机ID
     * @param filtered_vehicles [out] 过滤后的飞机列表
     * @return 需要避撞的飞机数量
     */
    int filter_targets(
        SwarmRole role,
        const OtherVehiclePosition *all_vehicles,
        int vehicle_count,
        int self_id,
        OtherVehiclePosition *filtered_vehicles
    );

    /**
     * @brief 获取上次过滤后的目标数量
     */
    int get_filtered_count() const { return _last_filtered_count; }

private:
    /**
     * @brief 判断是否需要对指定目标进行避撞
     * @param role 当前飞机角色
     * @param target 目标飞机信息
     * @return true: 需要避撞该目标; false: 不需要
     */
    bool should_avoid_target(SwarmRole role, const OtherVehiclePosition &target) const;

    int _last_filtered_count{0};
};
