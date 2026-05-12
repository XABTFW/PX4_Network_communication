# DM-IMU L1 快速参考

## 命令速查表

### 基本操作
```bash
dm_imu_l1 start              # 启动驱动（默认配置）
dm_imu_l1 start -i 0x20      # 指定 CAN ID 启动
dm_imu_l1 status             # 查看状态
dm_imu_l1 stop               # 停止驱动
```

### 传感器校准
```bash
dm_imu_l1 accel_cal          # 校准加速度计（保持水平静止）
dm_imu_l1 gyro_cal           # 校准陀螺仪（保持完全静止）
dm_imu_l1 mag_cal            # 校准磁力计
dm_imu_l1 set_zero           # 设置当前姿态为零点
```

### 模式切换
```bash
dm_imu_l1 active_mode        # 主动模式（连续输出）
dm_imu_l1 request_mode       # 请求模式（按需输出）
```

### 参数管理
```bash
dm_imu_l1 save               # 保存参数到 Flash
dm_imu_l1 restore            # 恢复出厂设置
dm_imu_l1 reboot             # 重启 IMU
```

## 数据查看

### 查看传感器数据
```bash
listener sensor_accel        # 加速度计
listener sensor_gyro         # 陀螺仪
listener vehicle_attitude    # 姿态（四元数）
listener vehicle_angular_velocity  # 角速度
listener sensor_combined     # 组合数据
```

## CAN 协议速查

### 寄存器 ID
| ID | 名称 | 功能 |
|----|------|------|
| 0 | REBOOT_IMU | 重启 |
| 1 | ACCEL_DATA | 加速度数据 |
| 2 | GYRO_DATA | 陀螺仪数据 |
| 3 | EULER_DATA | 欧拉角数据 |
| 4 | QUAT_DATA | 四元数据 |
| 5 | SET_ZERO | 设置零点 |
| 6 | ACCEL_CALI | 加速度校准 |
| 7 | GYRO_CALI | 陀螺仪校准 |
| 8 | MAG_CALI | 磁力计校准 |
| 11 | CHANGE_ACTIVE | 切换模式 |
| 254 | SAVE_PARAM | 保存参数 |
| 255 | RESTORE_SETTING | 恢复出厂 |

### 数据范围
| 数据类型 | 范围 |
|---------|------|
| 加速度 | ±235.2 m/s² |
| 陀螺仪 | ±34.88 rad/s |
| 俯仰角 | ±90° |
| 横滚角 | ±180° |
| 偏航角 | ±180° |

### CAN 帧格式

**命令帧** (8 字节):
```
[0xCC][reg_id][cmd][0xDD][data0][data1][data2][data3]
```

**数据帧** (8 字节):
```
[data_type][data0][data1][data2][data3][data4][data5][data6]
```

## 故障排查

### 驱动无法启动
```bash
# 检查 CAN 设备
ls -l /dev/can*

# 检查驱动状态
dm_imu_l1 status
```

### 没有数据输出
```bash
# 切换到主动模式
dm_imu_l1 active_mode

# 检查数据
listener sensor_accel
```

### 数据异常
```bash
# 重新校准
dm_imu_l1 accel_cal
dm_imu_l1 gyro_cal

# 保存参数
dm_imu_l1 save
```

## 启动脚本示例

在 `etc/init.d/rcS` 中添加：
```bash
# 启动 DM-IMU L1
dm_imu_l1 start -i 0x10 -d /dev/can0
usleep 100000
```

## 技术支持

- QQ 群：320296121
- 厂家官网：https://www.dm-robot.com/
