# DYT 丢失后归中不动问题修复

## 问题分析

丢失目标后云台归中但不继续搜索，主要原因：

1. **超时计算问题**：`DYTG_LOSTMS` 包含了归中等待时间，如果 `DYTG_LOSTMS <= DYTG_CTRMS`，还没开始扫描就超时退出
2. **缺少自动锁定重试**：丢失后的搜索阶段没有周期性重发 `CMD_AUTO_LOCK`
3. **hint 分支卡死**：`target_hint_detected()` 一直为真时会永久暂停扫描

## 修复内容

### 1. LostHold 状态超时逻辑改进

**修改位置**：`Run()` 函数中的 `TaskState::LostHold` 分支

**改动**：
- 归中完成后才开始计算丢失超时时间
- `DYTG_LOSTMS` 现在表示"归中完成之后还能搜索多久"
- 归中期间不会触发超时退出

```cpp
const bool center_done = (now - _state_enter_time) >= center_delay;

if (center_done && (now - _state_enter_time) > center_delay + lost_timeout) {
    abort_guidance(_lost_reason);
} else if (center_done) {
    update_search_scan(now);
    retry_autolock(now);  // 新增：周期性重发自动锁定
}
```

### 2. 丢失后周期性重发自动锁定

**修改位置**：`TaskState::LostHold` 分支

**改动**：
- 在搜索阶段调用 `retry_autolock(now)`
- 确保 DYT 在扫描过程中能够自动锁定新发现的目标

### 3. hint 分支防卡死

**修改位置**：`update_search_scan()` 函数

**改动**：
- 只有刚发送自动锁定命令时才暂停
- 如果等待时间过后还没锁上，继续扫描
- 避免 hint 位一直为真导致云台永久停住

```cpp
if (target_hint_detected() && !target_locked()) {
    const bool sent_lock = update_hint_autolock(now);

    if (sent_lock) {
        // 刚发送命令，暂停等待
        _search_pause_until = now + SEARCH_HINT_LOCK_HOLD;
        return;
    }

    if (_last_hint_lock_time != 0 && (now - _last_hint_lock_time) < SEARCH_HINT_LOCK_HOLD) {
        // 还在等待期内
        return;
    }

    // 超时未锁定，继续扫描
}
```

## 参数建议

建议参数设置：
- `DYTG_CTRMS` = 1500 ~ 2500 ms（归中时间）
- `DYTG_LOSTMS` = 15000 ~ 30000 ms（归中后的搜索时间）
- `DYTG_RTRYMS` = 500 ~ 1000 ms（自动锁定重试间隔）

## 验证方法

1. 查看状态：
```bash
dyt_guidance status
```

2. 检查关键信息：
- `last command`：应该看到从 0x2B(归中) → 0x26(扫描角度) → 0x0F(自动锁定) 的循环
- `state`：应该保持在 LostHold(4) 状态直到重新锁定或超时

3. 如果 `last command` 一直是 0x2B 或很快变成停止跟踪，说明参数设置有问题
