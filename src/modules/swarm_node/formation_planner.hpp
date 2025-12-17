#pragma once

#include <matrix/matrix/math.hpp>
#include "position_sharing.hpp"

/**
 * @brief 编队路径规划器 - 预测性避撞
 *
 * 针对编队切换时的交叉对冲场景，提前规划绕行路径
 */
class FormationPlanner {
public:
    struct PlannerConfig {
        float prediction_time = 3.0f;        // 预测未来3秒
        float collision_threshold = 2.0f;    // 碰撞阈值2米
        float detour_distance = 3.0f;        // 绕行距离3米
        float max_detection_distance = 7.0f; // 最大检测距离7米，超过此距离不进行预测避撞
        bool enable_planning = true;
    };

    struct PlanResult {
        matrix::Vector3f waypoint;           // 中间航点（绕行点）
        bool needs_detour = false;           // 是否需要绕行
        bool direct_path_safe = true;        // 直达路径是否安全
        int conflict_aircraft_id = -1;       // 冲突飞机ID
        float time_to_collision = 999.0f;    // 预计碰撞时间
    };

    FormationPlanner() = default;
    ~FormationPlanner() = default;

    void set_config(const PlannerConfig& config) { _config = config; }
    const PlannerConfig& get_config() const { return _config; }

    /**
     * @brief 规划编队切换路径，检测并避免对冲碰撞
     *
     * @param current_pos 当前位置
     * @param target_pos 目标位置（编队位置）
     * @param current_vel 当前速度
     * @param other_aircraft 其他飞机数组
     * @param max_aircraft_count 最大飞机数量
     * @param current_vehicle_id 本机ID
     * @return PlanResult 规划结果，包含绕行航点
     */
    PlanResult plan_formation_path(
        const matrix::Vector3f& current_pos,
        const matrix::Vector3f& target_pos,
        const matrix::Vector3f& current_vel,
        const OtherVehiclePosition* other_aircraft,
        int max_aircraft_count,
        uint8_t current_vehicle_id
    );

    const PlanResult& get_last_result() const { return _last_result; }

private:
    PlannerConfig _config;
    PlanResult _last_result;

    /**
     * @brief 预测碰撞检测
     */
    bool predict_collision(
        const matrix::Vector3f& my_pos,
        const matrix::Vector3f& my_vel,
        const matrix::Vector3f& other_pos,
        const matrix::Vector3f& other_vel,
        float& time_to_collision,
        float& min_distance
    );

    /**
     * @brief 生成顺时针绕行航点
     */
    matrix::Vector3f generate_detour_waypoint(
        const matrix::Vector3f& current_pos,
        const matrix::Vector3f& target_pos,
        const matrix::Vector3f& conflict_pos
    );
};

