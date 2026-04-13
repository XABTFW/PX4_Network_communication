# DM-IMU L1 PX4 集成完整指南

## 📦 第一步：集成到 PX4 源码

### 1.1 复制驱动文件

将整个 `dm_imu_l1` 文件夹复制到 PX4 源码目录：

```bash
# 假设你的 PX4 源码在 ~/PX4-Autopilot
cp -r dm_imu_l1 ~/PX4-Autopilot/src/drivers/imu/
```

### 1.2 启用驱动编译

编辑你的板级配置文件，例如：
- `boards/px4/fmu-v5/default.px4board`
- `boards/px4/fmu-v6x/default.px4board`
- 或你使用的具体板型配置文件

添加以下行：
```cmake
CONFIG_DRIVERS_IMU_DM_IMU_L1=y
```

或者使用 menuconfig：
```bash
cd ~/PX4-Autopilot
make px4_fmu-v5 menuconfig

# 导航到：
# Device Drivers -> IMU Drivers -> DM-IMU L1 CAN driver
# 按 Y 启用
```

### 1.3 编译固件

```bash
cd ~/PX4-Autopilot
make px4_fmu-v5  # 替换为你的板型
```

编译成功后会生成固件文件：
```
build/px4_fmu-v5_default/px4_fmu-v5_default.px4
```

---

## 🔥 第二步：烧录固件

### 2.1 使用 QGroundControl 烧录

1. 打开 QGroundControl
2. 连接飞控
3. 进入 "Vehicle Setup" -> "Firmware"
4. 选择 "Custom firmware file"
5. 选择编译好的 `.px4` 文件
6. 等待烧录完成

### 2.2 使用命令行烧录

```bash
cd ~/PX4-Autopilot
make px4_fmu-v5 upload
```

---

## 🔌 第三步：硬件连接

### 3.1 CAN 总线连接

将 IMU 连接到飞控的 CAN 接口：

```
IMU          飞控 CAN
CAN_H   ->   CAN_H
CAN_L   ->   CAN_L
GND     ->   GND
VCC     ->   5V
```

### 3.2 终端电阻

确保 CAN 总线两端有 120Ω 终端电阻。

### 3.3 检查 IMU 指示灯

- **绿灯呼吸**：正常工作模式 ✅
- **黄灯呼吸**：设置模式
- **红灯常亮**：错误状态 ❌

---

## 🚀 第四步：启动驱动（重要！）

### 4.1 通过 MAVLink 控制台启动

连接 QGroundControl，打开 MAVLink 控制台：

**QGroundControl** -> **Analyze Tools** -> **MAVLink Console**

输入以下命令：

```bash
# 使用默认配置启动（CAN ID: 0x10, 设备: /dev/can0）
dm_imu_l1 start

# 或指定 CAN ID 和设备
dm_imu_l1 start -i 0x10 -d /dev/can0
```

**预期输出**：
```
INFO  [dm_imu_l1] DM-IMU initialized on /dev/can0, CAN ID: 0x10, MST ID: 0x00
INFO  [dm_imu_l1] Switching to active mode
INFO  [dm_imu_l1] DM-IMU started in active mode
```

### 4.2 检查驱动状态

```bash
dm_imu_l1 status
```

**预期输出**：
```
INFO  [dm_imu_l1] State: 2
INFO  [dm_imu_l1] Error count: 0
INFO  [dm_imu_l1] Active: yes
```

状态说明：
- State: 0 = 未初始化
- State: 1 = 已初始化
- State: 2 = 主动模式 ✅
- State: 3 = 请求模式
- State: 4 = 错误状态

---

## 📊 第五步：验证数据输出

### 5.1 查看加速度计数据

```bash
listener sensor_accel
```

**预期输出**：
```
TOPIC: sensor_accel
    timestamp: 123456789
    device_id: 16777232
    x: 0.123
    y: -0.456
    z: 9.81
    error_count: 0
```

如果看到数据持续更新，说明驱动工作正常！✅

### 5.2 查看陀螺仪数据

```bash
listener sensor_gyro
```

**预期输出**：
```
TOPIC: sensor_gyro
    timestamp: 123456789
    device_id: 33554448
    x: 0.001
    y: -0.002
    z: 0.000
    error_count: 0
```

### 5.3 查看姿态数据

```bash
listener vehicle_attitude
```

**预期输出**：
```
TOPIC: vehicle_attitude
    timestamp: 123456789
    q[0]: 1.000  (w)
    q[1]: 0.000  (x)
    q[2]: 0.000  (y)
    q[3]: 0.000  (z)
```

### 5.4 查看组合数据

```bash
listener sensor_combined
```

---

## 🔧 第六步：传感器校准

### 6.1 校准加速度计

**重要**：IMU 必须保持水平静止！

```bash
dm_imu_l1 accel_cal
```

等待 3-5 秒，直到 IMU 指示灯停止闪烁。

### 6.2 校准陀螺仪

**重要**：IMU 必须保持完全静止！

```bash
dm_imu_l1 gyro_cal
```

等待 3-5 秒。

### 6.3 保存参数

```bash
dm_imu_l1 save
```

**预期输出**：
```
INFO  [dm_imu_l1] Saving parameters to flash...
```

---

## 🤖 第七步：配置自动启动

### 7.1 创建启动脚本

编辑或创建文件：`ROMFS/px4fmu_common/init.d/rcS`

在合适的位置添加：

```bash
#
# 启动 DM-IMU L1
#
if param compare SYS_AUTOSTART 4001
then
    # 启动驱动
    dm_imu_l1 start -i 0x10 -d /dev/can0
    
    # 等待初始化
    usleep 100000
fi
```

### 7.2 或使用自定义机架

创建文件：`ROMFS/px4fmu_common/init.d/airframes/99999_custom`

```bash
#!/bin/sh
#
# @name Custom Quadcopter with DM-IMU
# @type Quadrotor x
# @class Copter
#

. ${R}etc/init.d/rc.mc_defaults

# 启动 DM-IMU
dm_imu_l1 start -i 0x10 -d /dev/can0
usleep 100000

# 其他配置...
set MIXER quad_x
set PWM_OUT 1234
```

### 7.3 重新编译并烧录

```bash
make px4_fmu-v5
make px4_fmu-v5 upload
```

---

## 🐛 故障排查

### 问题 1：驱动启动失败

**症状**：
```
ERROR [dm_imu_l1] Failed to open CAN device: /dev/can0
```

**解决方法**：
```bash
# 检查 CAN 设备是否存在
ls -l /dev/can*

# 如果不存在，检查 CAN 驱动是否启动
dmesg | grep can

# 尝试手动启动 CAN
can start
```

### 问题 2：没有数据输出

**症状**：`listener sensor_accel` 没有数据

**检查步骤**：

1. 确认驱动已启动：
```bash
dm_imu_l1 status
```

2. 检查是否在主动模式：
```bash
dm_imu_l1 active_mode
```

3. 检查硬件连接：
   - CAN_H 和 CAN_L 是否接反
   - 终端电阻是否正确
   - IMU 指示灯状态

4. 检查 CAN ID 是否正确：
```bash
# 如果 IMU 的 CAN ID 不是 0x10，需要指定
dm_imu_l1 stop
dm_imu_l1 start -i 0x20  # 替换为实际 ID
```

### 问题 3：数据超时

**症状**：
```
WARN  [dm_imu_l1] IMU data timeout (error count: 1)
```

**解决方法**：

1. 检查 CAN 总线波特率：
```bash
# 确保飞控和 IMU 波特率一致（默认 1Mbps）
```

2. 检查 CAN 总线负载：
```bash
# 如果总线上有其他设备，可能需要调整
```

3. 重启驱动：
```bash
dm_imu_l1 stop
dm_imu_l1 start
```

### 问题 4：数据异常

**症状**：加速度或陀螺仪数据明显错误

**解决方法**：

1. 重新校准：
```bash
dm_imu_l1 accel_cal
dm_imu_l1 gyro_cal
dm_imu_l1 save
```

2. 恢复出厂设置：
```bash
dm_imu_l1 restore
dm_imu_l1 reboot
```

3. 等待 IMU 重启后重新启动驱动：
```bash
sleep 3
dm_imu_l1 start
```

---

## 📝 完整测试流程示例

```bash
# 1. 启动驱动
dm_imu_l1 start -i 0x10 -d /dev/can0

# 2. 检查状态
dm_imu_l1 status

# 3. 查看数据（每个命令按 Ctrl+C 退出）
listener sensor_accel
listener sensor_gyro
listener vehicle_attitude

# 4. 校准传感器
dm_imu_l1 accel_cal
sleep 5
dm_imu_l1 gyro_cal
sleep 5

# 5. 保存参数
dm_imu_l1 save

# 6. 再次检查数据
listener sensor_accel
```

---

## 🎯 成功标志

如果看到以下情况，说明驱动工作正常：

✅ 驱动启动无错误
✅ `dm_imu_l1 status` 显示 State: 2, Active: yes
✅ `listener sensor_accel` 有持续更新的数据
✅ `listener sensor_gyro` 有持续更新的数据
✅ `listener vehicle_attitude` 有姿态数据
✅ IMU 指示灯为绿灯呼吸
✅ 错误计数为 0 或很小

---

## 📞 获取帮助

如果遇到问题：

1. 查看完整日志：
```bash
dmesg
```

2. 查看 PX4 日志：
```bash
# 在 SD 卡上查找 .ulg 文件
# 使用 FlightPlot 或 PlotJuggler 分析
```

3. 联系技术支持：
   - 达妙科技 QQ 群：320296121
   - PX4 论坛：https://discuss.px4.io/

---

## 🚁 集成到飞行控制

驱动正常工作后，PX4 会自动使用 IMU 数据进行：
- 姿态估计
- 位置估计
- 飞行控制

你可以在 QGroundControl 中查看：
- **Flight View**：实时姿态显示
- **Analyze Tools -> MAVLink Inspector**：查看所有传感器数据
- **Vehicle Setup -> Sensors**：传感器状态

---

## ⚠️ 重要提示

1. **首次使用必须校准**传感器
2. **每次更换安装位置**后需要重新校准
3. **定期检查**错误计数，如果持续增加说明有问题
4. **飞行前**务必确认数据正常
5. **备份**原有的 IMU 配置，以便回退

---

## 📚 相关文档

- `README.md` - 完整使用文档
- `QUICK_REFERENCE.md` - 快速参考
- `PROTOCOL_NOTES.md` - 协议说明
- `CHANGELOG.md` - 更新日志

祝你飞行顺利！🚀
