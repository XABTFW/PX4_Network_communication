# DM-IMU L1 驱动重构总结

## 修改目标
将 DM-IMU 驱动从"旁路消息发布"改为"PX4 标准 IMU 驱动"，使其能够正确接入 PX4 主 IMU 链路。

## 核心问题修复

### 1. 使用 PX4 标准 IMU 发布器
**之前**: 手动发布 `sensor_accel_s`、`sensor_gyro_s`、`sensor_combined_s`、`vehicle_angular_velocity_s`、`vehicle_attitude_s`

**现在**: 使用 `PX4Accelerometer` 和 `PX4Gyroscope` 标准类
- 自动生成 `vehicle_imu` 消息（PX4 主 IMU 链路）
- 自动处理校准、选择、融合等标准流程
- 删除了不应该在驱动层发布的 `vehicle_attitude`

### 2. 修正 CAN 过滤器
**之前**: 接收所有 CAN 帧 (`can_mask = 0`)

**现在**: 只接收 IMU 响应帧
- 响应 ID = `(device_id << 4) | 0x01`
- 例如 device_id=0x01 时，只接收 0x011

### 3. 增加帧验证函数
新增 `is_valid_dm_frame()` 函数：
- 检查 DLC 是否为 8
- 检查 CAN ID 是否匹配
- 检查数据类型是否有效（ACCEL/GYRO/EULER/QUAT）

### 4. 修正错误计数逻辑
**之前**:
- 超时就累加 `_error_count++`
- 收到数据后不清零
- 任意帧都会更新时间戳

**现在**:
- 收到有效帧时清零 `_error_count = 0`
- 只有有效的 DM-IMU 帧才更新时间戳
- 错误计数上限为 10

### 5. 修正状态语义
**之前**: `is_active()` 只在 STATE_ACTIVE 时返回 true，导致 REQUEST 模式显示 "Active: no"

**现在**: `is_running()` 在 STATE_ACTIVE 或 STATE_REQUEST 时都返回 true

## 代码变更清单

### dm_imu_l1.hpp
- 引入 `PX4Accelerometer.hpp` 和 `PX4Gyroscope.hpp`
- 删除 5 个 uORB 发布器
- 添加 `PX4Accelerometer _px4_accel` 和 `PX4Gyroscope _px4_gyro`
- 添加 `is_valid_dm_frame()` 函数声明
- 修改 `is_active()` 为 `is_running()`
- 删除 `publish_sensor_combined()` 函数
- 添加数据接收计数器

### dm_imu_l1.cpp
- 构造函数初始化 `_px4_accel(0)` 和 `_px4_gyro(0)`
- `open_can_socket()` 中配置 PX4 标准发布器（device_id、range、scale）
- `setup_can_filter()` 改为只接收 IMU 响应帧
- `can_receive()` 中增加帧验证，收到有效帧时清零错误计数
- `parse_accel_data()` 改用 `_px4_accel.update()`
- `parse_gyro_data()` 改用 `_px4_gyro.update()`
- `parse_euler_data()` 和 `parse_quat_data()` 删除姿态发布，仅保留内部调试
- 删除 `publish_sensor_combined()` 函数
- 修正超时检测逻辑

### dm_imu_l1_main.cpp
- `status()` 函数显示 "Running" 而不是 "Active"

## 预期效果

1. **vehicle_imu 消息正常发布**: PX4 系统能识别并选中该 IMU
2. **错误计数正确**: 收到数据后清零，不会一直累积
3. **CAN 总线效率提升**: 只接收需要的帧，不处理无关数据
4. **符合 PX4 标准**: 可以被 EKF2 等估计器正常使用
5. **姿态链路正确**: 不在驱动层发布姿态，由 EKF2 统一输出

## 测试建议

1. 重新编译并烧录固件
2. 启动驱动: `dm_imu_l1 start -i 0x01 -m 0x11`
3. 检查状态: `dm_imu_l1 status` (应显示 Running: yes, Error count: 0)
4. 检查 IMU 选择: `sensors status` (应看到 CAN IMU 被选中)
5. 监听 vehicle_imu: `listener vehicle_imu` (应看到来自 CAN IMU 的数据)
6. 检查 EKF2: `ekf2 status` (应正常使用该 IMU)
