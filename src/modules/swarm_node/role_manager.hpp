/**
 * @file role_manager.hpp
 * @brief 角色管理器 - 判断飞机在多组编队中的角色
 *
 * 角色类型：
 * - PRIMARY_LEADER: 头号主机（第1组主机，无避撞）
 * - SECONDARY_LEADER: 次要主机（其他组主机，只对主机避撞）
 * - FOLLOWER: 从机（对所有飞机避撞）
 */

#pragma once

#include <cstdint>

/**
 * @brief 飞机角色类型枚举
 */
enum class SwarmRole {
    PRIMARY_LEADER,    // 头号主机（第1组主机，无避撞）
    SECONDARY_LEADER,  // 次要主机（其他组主机，主机间避撞）
    FOLLOWER           // 从机（全避撞）
};

/**
 * @brief 角色管理器类
 *
 * 根据组号和主机标识判断当前飞机的角色
 */
class RoleManager
{
public:
    RoleManager() = default;
    ~RoleManager() = default;

    /**
     * @brief 更新角色信息
     * @param is_leader 是否为主机（来自 SWARM_SET_LEADER 参数）
     * @param group_id 组号（来自 SWARM_GROUP_ID 参数）
     * @param primary_group 头号主机所在组号，默认=1
     */
    void update(bool is_leader, int group_id, int primary_group = 1);

    /**
     * @brief 获取当前角色
     */
    SwarmRole get_role() const { return _role; }

    /**
     * @brief 是否为头号主机
     */
    bool is_primary_leader() const { return _role == SwarmRole::PRIMARY_LEADER; }

    /**
     * @brief 是否为次要主机
     */
    bool is_secondary_leader() const { return _role == SwarmRole::SECONDARY_LEADER; }

    /**
     * @brief 是否为从机
     */
    bool is_follower() const { return _role == SwarmRole::FOLLOWER; }

    /**
     * @brief 是否为任意主机（头号或次要）
     */
    bool is_any_leader() const { return _is_leader; }

    /**
     * @brief 获取组号
     */
    int get_group_id() const { return _group_id; }

    /**
     * @brief 获取头号主机组号
     */
    int get_primary_group() const { return _primary_group; }

    /**
     * @brief 检测角色是否发生变化（用于状态机重置）
     */
    bool role_changed() const { return _role_changed; }

    /**
     * @brief 清除角色变化标志
     */
    void clear_role_changed() { _role_changed = false; }

    /**
     * @brief 获取角色名称字符串（用于日志）
     */
    const char* get_role_name() const;

private:
    SwarmRole _role{SwarmRole::FOLLOWER};
    bool _is_leader{false};
    int _group_id{1};
    int _primary_group{1};
    bool _role_changed{false};
};
