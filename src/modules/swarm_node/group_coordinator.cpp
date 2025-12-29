/**
 * @file group_coordinator.cpp
 * @brief 组间协调器实现
 */

#include "group_coordinator.hpp"

void GroupCoordinator::set_self_info(int vehicle_id, int group_id, bool is_leader)
{
    _vehicle_id = vehicle_id;
    _group_id = group_id;
    _is_leader = is_leader;
}

bool GroupCoordinator::is_my_group_leader(const uav_info_s &uav_info) const
{
    // 检查是否为同组主机：
    // 1. 组号相同
    // 2. 是主机
    // 3. 不是自己
    return (uav_info.group_id == static_cast<uint32_t>(_group_id)) &&
           (uav_info.is_leader) &&
           (uav_info.mavid != static_cast<uint32_t>(_vehicle_id));
}

bool GroupCoordinator::try_update_leader(const uav_info_s &uav_info)
{
    // 检查是否为同组主机
    if (!is_my_group_leader(uav_info)) {
        return false;
    }

    // 检查数据有效性
    if (!PX4_ISFINITE(uav_info.lat) || !PX4_ISFINITE(uav_info.lon)) {
        return false;
    }

    // 检测主机切换
    if (_has_valid_leader && _leader_info.mavid != uav_info.mavid) {
        // 主机ID发生变化，可能是组内主机切换
        // 这里可以添加日志或其他处理
    }

    _leader_info = uav_info;
    _has_valid_leader = true;
    return true;
}

void GroupCoordinator::clear_leader()
{
    _leader_info = uav_info_s{};
    _has_valid_leader = false;
}
