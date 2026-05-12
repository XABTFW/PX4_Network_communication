# DM-IMU L1 CAN Driver for PX4

达妙科技 IMU L1 的 PX4 CAN 总线驱动程序

## 功能特性

### 数据输出
- ✅ 加速度计数据 (sensor_accel)
- ✅ 陀螺仪数据 (sensor_gyro)
- ✅ 姿态数据 (vehicle_attitude) - 支持欧拉角和四元数
- ✅ 角速度数据 (vehicle_angular_velocity)
- ✅ 组合传感器数据 (sensor_combined)

### 工作模式
- ✅ 主动模式 (Active Mode) - IMU 自动连续发送数据
- ✅ 请求模式 (Request Mode) - 按需请求数据

### 配置功能
- ✅ 加速度计校准
- ✅ 陀螺仪校准
- ✅ 磁力计校准
- ✅ 设置零点
- ✅ CAN ID 配置
- ✅ 波特率配置
- ✅ 参数保存到 Flash
- ✅ 恢复出厂设置
- ✅ IMU 重启

### 错误处理
- ✅ 超时检测
- ✅ 错误计数
- ✅ 自动恢复机制
- ✅ 状态管理

## 编译配置

### 1. 将驱动添加到 PX4 源码

将 `dm_imu_l1` 文件夹复制到 PX4 源码的以下位置：
```
PX4-Autopilot/src/drivers/imu/dm_imu_l1/
```

### 2. 启用驱动

编辑你的板级配置文件（例如 `boards/px4/fmu-v5/default.px4board`），添加：
```cmake
CONFIG_DRIVERS_IMU_DM_IMU_L1=y
```

或者使用 menuconfig：
```bash
make px4_fmu-v5 menuconfig
# 导航到: Device Drivers -> IMU Drivers -> DM-IMU L1 CAN driver
```

### 3. 编译固件

```bash
make px4_fmu-v5
```

## 使用方法

### 启动驱动

使用默认配置启动（CAN ID: 0x10, 设备: /dev/can0）：
```bash
dm_imu_l1 start
```

使用自定义配置启动：
```bash
dm_imu_l1 start -i 0x20 -m 0x00 -d /dev/can1
```

参数说明：
- `-i <hex>`: CAN ID (十六进制，默认 0x10)
- `-m <hex>`: 主站 ID (十六进制，默认 0x00)
- `-d <path>`: CAN 设备路径 (默认 /dev/can0)

### 查看状态

```bash
dm_imu_l1 status
```

### 传感器校准

校准加速度计（IMU 需保持静止水平）：
```bash
dm_imu_l1 accel_cal
```

校准陀螺仪（IMU 需保持完全静止）：
```bash
dm_imu_l1 gyro_cal
```

校准磁力计：
```bash
dm_imu_l1 mag_cal
```

### 设置零点

将当前姿态设置为零点：
```bash
dm_imu_l1 set_zero
```

### 模式切换

切换到主动模式（连续输出）：
```bash
dm_imu_l1 active_mode
```

切换到请求模式（按需输出）：
```bash
dm_imu_l1 request_mode
```

### 参数管理

保存当前参数到 Flash：
```bash
dm_imu_l1 save
```

恢复出厂设置：
```bash
dm_imu_l1 restore
```

### 重启 IMU

```bash
dm_imu_l1 reboot
```

### 停止驱动

```bash
dm_imu_l1 stop
```

## 自动启动配置

要在系统启动时自动启动驱动，编辑启动脚本（例如 `ROMFS/px4fmu_common/init.d/rcS`）：

```bash
# 启动 DM-IMU L1
dm_imu_l1 start -i 0x10 -d /dev/can0

# 等待传感器初始化
usleep 100000
```

或者创建自定义启动脚本 `ROMFS/px4fmu_common/init.d-posix/airframes/99999_custom`：

```bash
#!/bin/sh
#
# Custom airframe with DM-IMU L1
#

. ${R}etc/init.d/rc.mc_defaults

# 启动 DM-IMU
dm_imu_l1 start -i 0x10

# 其他配置...
```

## 数据验证

### 查看传感器数据

查看加速度计数据：
```bash
listener sensor_accel
```

查看陀螺仪数据：
```bash
listener sensor_gyro
```

查看姿态数据：
```bash
listener vehicle_attitude
```

查看组合传感器数据：
```bash
listener sensor_combined
```

## 故障排查

### 驱动无法启动

1. 检查 CAN 设备是否存在：
   ```bash
   ls -l /dev/can*
   ```

2. 检查 CAN 总线配置：
   ```bash
   # 确保 CAN 总线已启动
   ifconfig can0
   ```

3. 检查 IMU 的 CAN ID 是否正确

### 没有数据输出

1. 检查驱动状态：
   ```bash
   dm_imu_l1 status
   ```

2. 检查 IMU 是否在主动模式：
   ```bash
   dm_imu_l1 active_mode
   ```

3. 检查 CAN 总线连接和终端电阻

### 数据超时

1. 检查错误计数：
   ```bash
   dm_imu_l1 status
   ```

2. 尝试重启驱动：
   ```bash
   dm_imu_l1 stop
   dm_imu_l1 start
   ```

3. 检查 CAN 总线波特率是否匹配

## 技术规格

### CAN 协议

**当前实现的协议版本**（基于 MC02_CAN收发例程）：
- 标准 CAN 2.0A (11-bit ID)
- 默认波特率：1 Mbps
- 数据帧长度：8 字节
- 命令帧格式：`[0xCC][reg_id][cmd][0xDD][data0-3]`
- 数据帧格式：`[data_type][reserved][data0-5]`
- 发送 ID：设备 CAN ID
- 接收 ID：设备 CAN ID + 0x10

**注意**：官方手册附录三提供了另一种请求协议：
- 使用固定 CAN ID `0x6FF` 发送请求
- 数据格式：`[can_id_low][can_id_high][reg][0xCC]` (4字节)
- 这种协议可能用于不同的固件版本或使用场景

如果你的 IMU 固件使用手册中的协议，需要修改 `can_send_cmd()` 函数。
当前实现已在实际硬件上验证可用。

### 数据范围

- 加速度：±235.2 m/s²
- 陀螺仪：±34.88 rad/s
- 俯仰角：±90°
- 横滚角：±180°
- 偏航角：±180°
- 温度：0-60°C

### 性能

- 更新频率：最高 200 Hz
- 最小发布间隔：5 ms
- 超时检测：1 秒

## 开发说明

### 文件结构

```
dm_imu_l1/
├── CMakeLists.txt          # 编译配置
├── Kconfig                 # 内核配置
├── README.md               # 本文档
├── dm_imu_l1.hpp          # 驱动头文件
├── dm_imu_l1.cpp          # 驱动实现
└── dm_imu_l1_main.cpp     # 主程序入口
```

### 关键类和方法

**DmImuL1 类**：
- `init()`: 初始化驱动
- `start()`: 启动数据采集
- `stop()`: 停止驱动
- `calibrate_*()`: 校准功能
- `set_*()`: 配置功能
- `request_*()`: 数据请求（被动模式）

### 扩展开发

如需添加新功能：

1. 在 `dm_imu_l1.hpp` 中添加方法声明
2. 在 `dm_imu_l1.cpp` 中实现方法
3. 在 `dm_imu_l1_main.cpp` 中添加命令行接口
4. 更新本 README 文档

## 参考资料

- [达妙科技官网](https://www.dm-robot.com/)
- [PX4 开发指南](https://docs.px4.io/main/en/development/development.html)
- [PX4 驱动开发](https://docs.px4.io/main/en/middleware/drivers.html)

## 技术支持

- QQ 群：320296121
- GitHub Issues: [提交问题](https://github.com/your-repo/issues)

## 许可证

本驱动遵循 BSD-3-Clause 许可证，与 PX4 项目保持一致。

## 更新日志

### v1.0.0 (2025-01-XX)
- ✅ 初始版本
- ✅ 支持加速度计、陀螺仪、姿态数据
- ✅ 支持主动和请求模式
- ✅ 完整的校准和配置功能
- ✅ 错误处理和超时检测
- ✅ sensor_combined 发布
