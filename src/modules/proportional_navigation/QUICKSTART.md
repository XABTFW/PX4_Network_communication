# PPNIACG 快速入门

## 5分钟上手指南

### 1. 编译模块

```bash
# 在PX4根目录
make px4_sitl_default
```

### 2. 启动仿真

```bash
make px4_sitl_default gazebo
```

### 3. 启动模块

在PX4控制台：
```bash
proportional_navigation start
```

### 4. 配置参数（垂直打击示例）

```bash
# 设置90度垂直打击
param set PN_IMPACT_ANG_V -90.0

# 启用铅垂面约束
param set PN_CONSTR_MODE 1

# 设置速度和加速度
param set PN_MAX_VEL 20.0
param set PN_MAX_ACCEL 15.0
```

### 5. 执行任务

```bash
# 打击目标位置 (x=50m, y=30m, z=-10m)
proportional_navigation engage 50.0 30.0 -10.0
```

### 6. 监控状态

```bash
proportional_navigation status
```

## 常用场景

### 场景A：垂直打击
```bash
param set PN_IMPACT_ANG_V -90.0
param set PN_CONSTR_MODE 1
proportional_navigation engage 50.0 0.0 0.0
```

### 场景B：45度角打击
```bash
param set PN_IMPACT_ANG_V -45.0
param set PN_CONSTR_MODE 1
proportional_navigation engage 80.0 40.0 -5.0
```

### 场景C：快速模式（无角度约束）
```bash
param set PN_CONSTR_MODE 0
param set PN_MAX_VEL 25.0
proportional_navigation engage 60.0 30.0 -10.0
```

## 中止任务

```bash
proportional_navigation abort
```

## 故障排除

### 问题：模块启动失败
```bash
# 检查是否已编译
ls build/px4_sitl_default/src/modules/proportional_navigation/
```

### 问题：任务无法启动
```bash
# 查看状态
proportional_navigation status

# 检查是否有其他任务在运行
```

### 问题：碰撞角不准确
```bash
# 增大初始距离
# 调整导引系数
param set PNAV_GAIN 4.0
```

## 下一步

- 阅读 [README.md](README.md) 了解详细功能
- 阅读 [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md) 了解实现细节
- 阅读 [SUMMARY.md](SUMMARY.md) 了解完整总结

## 重要提示

⚠️ **仅适用于固定目标**
⚠️ **先在仿真测试**
⚠️ **确保初始距离 > 20m**
⚠️ **确保初始速度 > 5 m/s**


## 俯冲仰头拉升（新功能）

### 快速体验拉升功能

```bash
# 1. 启动模块
proportional_navigation start

# 2. 配置俯冲拉升
param set PN_IMPACT_ANG_V -45.0    # 45度俯冲
param set PN_CONSTR_MODE 1       # 铅垂面约束
param set PN_ENABLE_PULLUP 1         # 启用拉升
param set PN_PULLUP_DIST 5.0         # 距目标5米拉升
param set PN_PULLUP_ALT -30.0        # 拉升到30米
param set PN_PULLUP_ACCEL 10.0       # 拉升加速度

# 3. 执行任务
proportional_navigation engage 50.0 30.0 0.0

# 4. 观察状态变化
# ENGAGING → PULLUP → COMPLETE
proportional_navigation status
```

### 拉升模式对比

**无拉升（直接打击）**：
```bash
param set PN_ENABLE_PULLUP 0
proportional_navigation engage 50.0 30.0 -10.0
```

**有拉升（俯冲后拉起）**：
```bash
param set PN_ENABLE_PULLUP 1
param set PN_PULLUP_DIST 5.0
param set PN_PULLUP_ALT -30.0
proportional_navigation engage 50.0 30.0 0.0
```

详细说明请参考 [PULLUP_GUIDE.md](PULLUP_GUIDE.md)
