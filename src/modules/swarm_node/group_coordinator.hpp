/**
 * @file group_coordinator.hpp
 * @brief 组间协调器 - 处理组内跟随和组间关系
 *
 * 功能：
 * - 从机查找并跟随同组主机
 * - 验证主机信息是否属于同组
 */

#pragma once

#include <uORB/topics/uav_info.h>
#include <px4_platform_common/defines.h>

/**
 * @brief 组间协调器类
 *
 * 负责管理组内的主从关系，确保从机只跟随同组的主机
 */
class GroupCoordinator
{
public:
    GroupCoordinator() = default;
    ~GroupCoordinator() = default;

    /**
     * @brief 设置本机信息
     * @param vehicle_id 本机ID
     * @param group_id 本机组号
     * @param is_leader 本机是否为主机
     */
    void set_self_info(int vehicle_id, int group_id, bool is_leader);

    /**
     * @brief 检查收到的飞机信息是否为同组主机
     * @param uav_info 收到的飞机信息
     * @return true: 是同组主机; false: 不是
     */
    bool is_my_group_leader(const uav_info_s &uav_info) const;

    /**
     * @brief 尝试更新同组主机信息
     * @param uav_info 收到的飞机信息
     * @return true: 更新成功（是同组主机）; false: 不是同组主机
     */
    bool try_update_leader(const uav_info_s &uav_info);

    /**
     * @brief 获取同组主机信息
     */
    const uav_info_s& get_leader_info() const { return _leader_info; }

    /**
     * @brief 检查是否有有效的同组主机
     */
    bool has_valid_leader() const { return _has_valid_leader; }

    /**
     * @brief 清除主机信息（用于主机切换时）
     */
    void clear_leader();

    /**
     * @brief 获取本机组号
     */
    int get_group_id() const { return _group_id; }

    /**
     * @brief 获取当前跟随的主机ID
     */
    int get_leader_id() const { return _has_valid_leader ? _leader_info.mavid : 0; }

private:
    int _vehicle_id{0};
    int _group_id{1};
    bool _is_leader{false};

    uav_info_s _leader_info{};
    bool _has_valid_leader{false};
};
