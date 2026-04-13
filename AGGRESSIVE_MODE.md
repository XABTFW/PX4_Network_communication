# 激进打击模式配置

## 快速设置（复制粘贴）

```bash
# 启动模块
proportional_navigation start

# 激进模式参数
param set PN_MAX_VEL 80.0        # 最大速度 80 m/s (288 km/h)
param set PN_MAX_ACCEL 40.0      # 最大加速度 40 m/s²
param set PN_NO_VEL_LIMIT 0      # 启用速度限制（80 m/s）

# 或者极限模式（无速度限制）
param set PN_NO_VEL_LIMIT 1      # 禁用速度限制
param set PN_MAX_ACCEL 45.0      # 最大加速度 45 m/s²

# 执行打击
proportional_navigation engage_polar 100 0 30 10 0 30

# 等待2秒后解锁
commander arm
commander mode offboard
```

## 性能对比

### 默认模式（修改前）
- 最大速度：20 m/s
- 最大加速度：15 m/s²
- 初始速度：0 m/s
- 起飞速度：2 m/s
- 到达最大速度时间：1.3秒
- 100米距离打击时间：~8秒

### 高速模式（修改后）
- 最大速度：50 m/s
- 最大加速度：40 m/s
- 初始速度：20 m/s
- 起飞速度：5 m/s
- 到达最大速度时间：0.75秒
- 100米距离打击时间：~3秒

### 激进模式（推荐）
- 最大速度：80 m/s
- 最大加速度：40 m/s²
- 初始速度：20 m/s
- 起飞速度：5 m/s
- 到达最大速度时间：1.5秒
- 100米距离打击时间：~2秒

### 极限模式（无速度限制）
- 最大速度：无限制（受时间和加速度限制）
- 最大加速度：45 m/s²
- 初始速度：20 m/s
- 起飞速度：5 m/s
- 理论最大速度：>100 m/s（实际受空气阻力限制）
- 100米距离打击时间：~1.5秒

## 动能对比（假设2kg无人机）

| 模式 | 速度 | 动能 | 相对提升 |
|-----|------|------|---------|
| 默认 | 20 m/s | 400 J | 1x |
| 高速 | 50 m/s | 2500 J | 6.25x |
| 激进 | 80 m/s | 6400 J | 16x |
| 极限 | 100 m/s | 10000 J | 25x |

## 实时监控命令

```bash
# 查看状态（每秒更新）
watch -n 1 "proportional_navigation status"

# 或者在PX4 Shell中
proportional_navigation status
```

## 参数说明

### PN_MAX_VEL（最大速度）
- 默认：50 m/s
- 范围：5-100 m/s
- 推荐：
  - 测试：30 m/s
  - 实战：80 m/s
  - 极限：100 m/s

### PN_MAX_ACCEL（最大加速度）
- 默认：40 m/s²
- 范围：5-50 m/s²
- 推荐：
  - 保守：20 m/s²
  - 激进：40 m/s²
  - 极限：45 m/s²

### PN_NO_VEL_LIMIT（禁用速度限制）
- 0：启用速度限制（使用 PN_MAX_VEL）
- 1：禁用速度限制（无限加速）

## 修改内容总结

1. **默认最大加速度**：15 → 40 m/s²
2. **默认最大速度**：20 → 50 m/s
3. **速度上限范围**：30 → 100 m/s
4. **初始速度**：0 → 20 m/s（起飞完成后）
5. **起飞速度**：2 → 5 m/s
6. **新增无速度限制模式**

## 测试场景

### 场景1：近距离快速打击
```bash
param set PN_MAX_VEL 60.0
param set PN_MAX_ACCEL 40.0
proportional_navigation engage_polar 40 0 30 0 0 30
```
预期：1秒内命中

### 场景2：中距离高速打击
```bash
param set PN_MAX_VEL 80.0
param set PN_MAX_ACCEL 40.0
proportional_navigation engage_polar 100 0 30 10 0 30
```
预期：2秒内命中

### 场景3：远距离极限打击
```bash
param set PN_NO_VEL_LIMIT 1
param set PN_MAX_ACCEL 45.0
proportional_navigation engage_polar 200 0 30 15 0 30
```
预期：3秒内命中，速度可能超过100 m/s

### 场景4：运动目标拦截
```bash
param set PN_MAX_VEL 80.0
param set PN_MAX_ACCEL 40.0
proportional_navigation engage_polar 150 45 25 20 90 30
```
目标：150米，东北方向，25米高，20m/s向东飞行
预期：3秒内拦截

## 安全建议

1. **仿真测试**：先在 Gazebo 仿真中测试
2. **逐步提高**：从低速开始，逐步提高参数
3. **空域检查**：确保有足够的安全空域
4. **电池监控**：高速飞行耗电极大
5. **GPS频率**：建议使用10Hz以上的GPS
6. **飞控限制**：注意飞控的最大倾角限制

## 故障排查

### 问题：速度上不去
```bash
# 检查参数
param show PN_MAX_VEL
param show PN_MAX_ACCEL
param show PN_NO_VEL_LIMIT

# 查看实时状态
proportional_navigation status
```

### 问题：加速太慢
- 提高 PN_MAX_ACCEL 到 40-45
- 检查飞控是否有倾角限制
- 确认 Offboard 模式正常工作

### 问题：起飞后悬停
- 确认目标位置设置正确
- 检查距离是否太近（< 5米）
- 查看日志中的 "Initial velocity" 信息

## 恢复默认

```bash
param set PN_MAX_VEL 50.0
param set PN_MAX_ACCEL 40.0
param set PN_NO_VEL_LIMIT 0
```
