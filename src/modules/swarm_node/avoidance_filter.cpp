/**
 * @file avoidance_filter.cpp
 * @brief 避撞过滤器实现
 */

#include "avoidance_filter.hpp"

bool AvoidanceFilter::should_perform_avoidance(SwarmRole role) const
{
    // 头号主机不需要避撞
    return role != SwarmRole::PRIMARY_LEADER;
}

bool AvoidanceFilter::should_avoid_target(SwarmRole role, const OtherVehiclePosition &target) const
{
    // 无效目标不避撞
    if (!target.valid) {
        return false;
    }

    switch (role) {
    case SwarmRole::PRIMARY_LEADER:
        // 头号主机：不避撞任何飞机
        return false;

    case SwarmRole::SECONDARY_LEADER:
        // 次要主机：只避撞其他主机
        return target.is_leader;

    case SwarmRole::FOLLOWER:
        // 从机：避撞所有飞机
        return true;

    default:
        return true;
    }
}

int AvoidanceFilter::filter_targets(
    SwarmRole role,
    const OtherVehiclePosition *all_vehicles,
    int vehicle_count,
    int self_id,
    OtherVehiclePosition *filtered_vehicles)
{
    int filtered_count = 0;

    // 头号主机直接返回0，不需要避撞任何目标
    if (role == SwarmRole::PRIMARY_LEADER) {
        _last_filtered_count = 0;
        return 0;
    }

    for (int i = 0; i < vehicle_count; i++) {
        // 跳过无效数据
        if (!all_vehicles[i].valid) {
            continue;
        }

        // 跳过自己
        if (all_vehicles[i].mavid == self_id) {
            continue;
        }

        // 根据角色判断是否需要避撞该目标
        if (should_avoid_target(role, all_vehicles[i])) {
            filtered_vehicles[filtered_count] = all_vehicles[i];
            filtered_count++;
        }
    }

    _last_filtered_count = filtered_count;
    return filtered_count;
}
