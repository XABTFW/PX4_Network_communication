/**
 * @file role_manager.cpp
 * @brief 角色管理器实现
 */

#include "role_manager.hpp"

void RoleManager::update(bool is_leader, int group_id, int primary_group)
{
    SwarmRole old_role = _role;

    _is_leader = is_leader;
    _group_id = group_id;
    _primary_group = primary_group;

    // 根据主机标识和组号判断角色
    if (is_leader) {
        if (group_id == primary_group) {
            // 第1组（或指定的头号组）的主机 = 头号主机
            _role = SwarmRole::PRIMARY_LEADER;
        } else {
            // 其他组的主机 = 次要主机
            _role = SwarmRole::SECONDARY_LEADER;
        }
    } else {
        // 非主机 = 从机
        _role = SwarmRole::FOLLOWER;
    }

    // 检测角色是否发生变化
    _role_changed = (old_role != _role);
}

const char* RoleManager::get_role_name() const
{
    switch (_role) {
    case SwarmRole::PRIMARY_LEADER:
        return "头号主机";
    case SwarmRole::SECONDARY_LEADER:
        return "次要主机";
    case SwarmRole::FOLLOWER:
        return "从机";
    default:
        return "未知";
    }
}
