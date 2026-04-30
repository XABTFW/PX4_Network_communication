# DYT Guidance 重锁优化改进文档

## 问题描述

原有逻辑过于严格，导致以下问题：
1. 轻微拖动摄像头后，目标短暂失锁就立即进入 LostHold
2. 一进入 LostHold 就立刻居中云台并重新搜索
3. 用户体验：明明目标还在画面里，系统却放弃跟踪重新搜索

## 核心改进思路

**"锁上以后不要轻易脱离，只有确认真的丢了才重新搜索"**

## 具体改动

### 1. 放宽视场阈值 (TARGET_MAX_LOS_RAD)

**位置**: 第 59 行

```cpp
// 修改前
static constexpr float TARGET_MAX_LOS_RAD{0.45f};  // 约 25.8°

// 修改后
static constexpr float TARGET_MAX_LOS_RAD{0.65f};  // 约 37.2°
```

**原因**: 0.45 rad 太窄，轻微拖动摄像头后目标框还在画面里，但 los_x_rad 或 los_y_rad 超过阈值导致 `target_geometry_valid()` 失败。

---

### 2. 增加锁定保持时间戳

**位置**: 类成员变量声明区

```cpp
int _lock_streak{0};
int _relock_streak{0};
int _lost_streak{0};
int _bad_lock_streak{0};
hrt_abstime _last_usable_time{0};    // 新增：最后一次强可用时间
hrt_abstime _last_locked_time{0};   // 新增：最后一次锁定时间
```

**作用**: 记录最近一次成功锁定的时间，用于判断是否在宽限期内。

---

### 3. 新增宽松的锁定保持判断函数

**位置**: 函数声明和实现

```cpp
bool target_hold_usable(hrt_abstime now) const;
```

**实现逻辑**:
```cpp
bool DytGuidance::target_hold_usable(hrt_abstime now) const
{
    constexpr hrt_abstime LOCK_GRACE_TIME = 1500_ms;

    // 1. 强可用（target_usable() == true），当然可保持
    if (target_usable()) {
        return true;
    }

    // 2. 最近刚锁过，不要因为短暂抖动马上丢
    if (_last_usable_time > 0 && (now - _last_usable_time) <= LOCK_GRACE_TIME) {
        return true;
    }

    // 3. tracking_state 还没完全 LOCKED，但有目标框/hint，也认为还可以等待重锁
    if (target_lock_candidate() || target_hint_detected()) {
        return true;
    }

    // 4. 如果只是 frame 短暂不新鲜，但最近锁定过，也先保持
    if (_last_locked_time > 0 && (now - _last_locked_time) <= LOCK_GRACE_TIME) {
        return true;
    }

    return false;
}
```

**关键点**:
- 1.5 秒宽限期内的短暂失锁不会触发 LostHold
- 有候选目标或 hint 时继续尝试重锁，不进入搜索

---

### 4. 在主循环中更新时间戳

**位置**: `Run()` 函数开始处

```cpp
update_params_if_needed();
update_subscriptions();

const hrt_abstime now = hrt_absolute_time();
const bool activation_request = activation_requested();
const bool intercept_request = aux_switch_active(_param_int_aux.get());

// 新增：更新锁定时间戳
if (target_usable()) {
    _last_usable_time = now;
}

if (target_locked() && target_fresh()) {
    _last_locked_time = now;
}
```

---

### 5. 修改 TrackFollow 状态逻辑

**修改前**:
```cpp
case TaskState::TrackFollow:
    if (!target_usable()) {
        ++_lost_streak;
        if (_lost_streak >= _param_relock_frames.get()) {
            enter_lost_hold(...);
        }
    } else {
        _lost_streak = 0;
        if (intercept_request && intercept_allowed()) {
            enter_state(TaskState::TrackIntercept);
        }
    }
    break;
```

**修改后**:
```cpp
case TaskState::TrackFollow:
    if (target_usable()) {
        // 强锁定：正常跟踪
        _lost_streak = 0;
        if (intercept_request && intercept_allowed()) {
            enter_state(TaskState::TrackIntercept);
        }

    } else if (target_hold_usable(now)) {
        // 已经锁上过，当前只是短暂掉帧/状态抖动/geometry 抖动
        // 不进入 LostHold，不重新搜索
        _lost_streak = 0;

        // 如果还有候选目标或 hint，尝试让载荷重锁
        if (target_lock_candidate() || target_hint_detected()) {
            update_hint_autolock(now);
        }

    } else {
        // 真正丢失：累计丢失帧数
        ++_lost_streak;
        if (_lost_streak >= _param_relock_frames.get()) {
            enter_lost_hold(...);
        }
    }
    break;
```

**关键改进**:
- 三级判断：强锁定 → 保持锁定 → 真正丢失
- 保持锁定期间不累计 `_lost_streak`
- 有候选目标时主动发送 AUTO_LOCK 命令

---

### 6. 修改 TrackIntercept 状态逻辑

**改动与 TrackFollow 相同**，确保拦截模式下也不会因为轻微拖动就丢失目标。

```cpp
case TaskState::TrackIntercept:
    if (target_usable()) {
        _lost_streak = 0;
        if (!intercept_request || !intercept_allowed()) {
            enter_state(TaskState::TrackFollow);
        }

    } else if (target_hold_usable(now)) {
        // 拦截中也不要因为短暂抖动马上丢锁
        _lost_streak = 0;
        if (target_lock_candidate() || target_hint_detected()) {
            update_hint_autolock(now);
        }

    } else {
        ++_lost_streak;
        if (_lost_streak >= _param_relock_frames.get()) {
            enter_lost_hold(...);
        }
    }
    break;
```

---

### 7. 延迟 LostHold 的云台居中和搜索

**修改前** (`enter_state` 中的 LostHold 分支):
```cpp
else if (new_state == TaskState::LostHold) {
    // ...
    clear_observations();
    capture_hold_setpoint();
    reset_search_scan(_state_enter_time);  // 立即开始搜索
}
```

**修改后**:
```cpp
else if (new_state == TaskState::LostHold) {
    // ...
    clear_observations();
    capture_hold_setpoint();

    // 不要一进入 LostHold 就立刻居中云台
    // 先在原地等待一段时间，让载荷有机会自己恢复锁定
    _search_pause_until = _state_enter_time +
        static_cast<hrt_abstime>(math::max(_param_center_ms.get(), int32_t{0})) * 1000ULL;
    _next_scan_time = _search_pause_until;
    _scan_segment_target_deg = NAN;
}
```

**关键点**:
- 不再立即调用 `reset_search_scan()`
- 不再立即发送 `CMD_CENTER_GIMBAL`
- 先等待 `DYTG_CTRMS` 参数设定的时间
- 等待期间载荷有机会自己恢复锁定

---

### 8. 恢复 reset_search_scan 的立即搜索逻辑

**位置**: `reset_search_scan()` 函数

```cpp
void DytGuidance::reset_search_scan(hrt_abstime now)
{
    _search_phase = SearchPhase::LocalReacquire;
    _search_phase_start_time = now;
    _local_reacquire_index = 0;

    _scan_row = 0;
    _scan_yaw_deg = PX4_ISFINITE(_last_seen_yaw_deg) ? _last_seen_yaw_deg : 0.f;
    _scan_pitch_deg = PX4_ISFINITE(_last_seen_pitch_deg) ? _last_seen_pitch_deg : 0.f;
    _scan_segment_target_deg = NAN;
    _search_pause_until = 0;
    _next_scan_time = now;  // 立即开始（当被调用时）
}
```

**原因**: 延迟逻辑已经移到 `enter_state(LostHold)` 中，`reset_search_scan` 被调用时应该立即开始搜索。

---

## 行为变化对比

### 修改前
```
目标锁定中
  ↓
轻轻拖动摄像头
  ↓
target_usable() 瞬间 false
  ↓
lost_streak 达阈值（3帧）
  ↓
进入 LostHold
  ↓
立刻 CMD_CENTER_GIMBAL
  ↓
云台居中，重置搜索
  ↓
用户感觉：系统放弃了目标
```

### 修改后
```
目标锁定中
  ↓
轻轻拖动摄像头
  ↓
target_usable() 短暂 false
  ↓
target_hold_usable() 仍然 true（在宽限期内）
  ↓
保持 TrackFollow 状态
  ↓
发现候选目标 → 发送 AUTO_LOCK
  ↓
载荷重新锁定
  ↓
继续跟踪（用户无感知）
```

**只有真正丢失超过 1.5 秒才会**:
```
  ↓
进入 LostHold
  ↓
等待 DYTG_CTRMS（例如 1000ms）
  ↓
仍未恢复 → 开始局部重捕获搜索
  ↓
仍未找到 → 前方蛇形搜索
  ↓
仍未找到 → 全局蛇形搜索
```

---

## 推荐参数设置

修改代码后，建议调整以下参数以获得最佳效果：

| 参数 | 原默认值 | 推荐值 | 说明 |
|------|---------|--------|------|
| `DYTG_RELOCKN` | 3 | **10 或 15** | 允许更多帧的短暂异常 |
| `DYTG_MAXAGE` | 0.25s | **0.8s** | 允许更长的目标数据延迟 |
| `DYTG_MAXJIT` | 0.12s | **0.5s** | 允许更大的帧间隔抖动 |
| `DYTG_CTRMS` | 400ms | **1000 或 1500ms** | 进入 LostHold 后等待更长时间再搜索 |

**计算示例**:
- 30fps 视频，`DYTG_RELOCKN = 10` → 允许约 333ms 的短暂异常
- 20fps 视频，`DYTG_RELOCKN = 10` → 允许约 500ms 的短暂异常

---

## 测试验证

### 测试场景 1：轻微拖动摄像头
**预期**: 目标短暂失锁后自动重锁，不进入搜索模式

### 测试场景 2：目标被短暂遮挡
**预期**: 1.5 秒内恢复则继续跟踪，超过 1.5 秒才进入 LostHold

### 测试场景 3：目标真正丢失
**预期**:
1. 先在原地等待 `DYTG_CTRMS` 时间
2. 然后开始局部重捕获（围绕最后位置搜索）
3. 失败后进入前方蛇形搜索
4. 最后进入全局蛇形搜索

### 测试场景 4：边缘误锁
**预期**: 检测到边缘误锁后强制重新搜索（原有逻辑保持）

---

## 总结

这次改进的核心是**增加锁定容错性**，通过以下机制实现：

1. ✅ 放宽几何有效性阈值（TARGET_MAX_LOS_RAD: 0.45 → 0.65）
2. ✅ 增加 1.5 秒锁定保持宽限期
3. ✅ 三级判断：强锁定 → 保持锁定 → 真正丢失
4. ✅ 保持锁定期间主动尝试重锁（AUTO_LOCK）
5. ✅ 延迟 LostHold 的云台居中和搜索启动

**最终效果**: 锁上以后不轻易脱离，只有确认真的丢了才重新搜索。
