/**
 * @file group_coordinator.cpp
 * @brief 组间协调器实现
 */

#include "group_coordinator.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

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

bool GroupCoordinator::detect_other_leader_in_group(const uav_info_s &uav_info) const
{
    // 检测同组是否有其他主机（用于多主机冲突检测）
    return is_my_group_leader(uav_info);
}

bool GroupCoordinator::try_update_leader(const uav_info_s &uav_info)
{
    // 检查是否为同组主机
    if (!is_my_group_leader(uav_info)) {
        // 如果收到的消息来自当前主机但 is_leader=0，说明主机已切换
        if (_has_valid_leader && uav_info.mavid == _leader_info.mavid && !uav_info.is_leader) {
            PX4_INFO("[主机切换] 检测到旧主机ID=%d已变为从机，清除主机信息", uav_info.mavid);
            _leader_info = uav_info_s{};
            _has_valid_leader = false;
            _leader_update_time = 0;
        }
        return false;
    }

    // 检查数据有效性
    if (!PX4_ISFINITE(uav_info.lat) || !PX4_ISFINITE(uav_info.lon)) {
        return false;
    }

    uint64_t current_time = hrt_absolute_time();

    // 主机切换检测：如果收到新主机的消息，且当前主机ID不同
    if (_has_valid_leader && _leader_info.mavid != uav_info.mavid) {
        // 检查当前主机是否超时
        bool current_leader_timeout = (current_time - _leader_info.timestamp) > LEADER_TIMEOUT_US;

        if (!current_leader_timeout) {
            // 当前主机信号正常，但收到了新主机的消息
            // 这说明组内发生了主机切换，应该切换到新主机
            PX4_INFO("[主机切换] 检测到新主机ID=%d，旧主机ID=%d，切换到新主机",
                     uav_info.mavid, _leader_info.mavid);
        }
        // 无论是否超时，都切换到新主机（因为新主机的 is_leader=true）
    }

    _leader_info = uav_info;
    _has_valid_leader = true;
    _leader_update_time = current_time;
    return true;
}

bool GroupCoordinator::has_valid_leader() const
{
    if (!_has_valid_leader) {
        return false;
    }

    // 检查主机信息是否过期
    uint64_t current_time = hrt_absolute_time();
    if ((current_time - _leader_info.timestamp) > LEADER_TIMEOUT_US) {
        return false;
    }

    return true;
}

void GroupCoordinator::clear_leader()
{
    _leader_info = uav_info_s{};
    _has_valid_leader = false;
    _leader_update_time = 0;
}
