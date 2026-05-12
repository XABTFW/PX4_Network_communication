# DJ_MK 驱动集成指南

本文档说明如何将 DJ_MK 驱动集成到 Pixhawk 6C 固件中。

## 1. 启用驱动编译

### 方法 A: 修改板级配置文件

编辑 `boards/px4/fmu-v6c/default.px4board` 文件，添加：

```cmake
CONFIG_DRIVERS_ACTUATORS_DJ_MK=y
```

### 方法 B: 使用 menuconfig

```bash
cd PX4-Autopilot
make px4_fmu-v6c_default menuconfig
```

导航到：
```
Device Drivers → Actuators → dj_mk
```

按 `Y` 启用，然后保存并退出。

## 2. 配置 CAN 总线

### 2.1 启用 CAN1 接口

在启动脚本中（通常是 `ROMFS/px4fmu_common/init.d/rcS` 或板级启动脚本），添加：

```bash
# 初始化 CAN1
if param compare -s CAN_ENABLE 1
then
    # 设置 CAN1 波特率为 250kbps
    can start 250000
fi
```

### 2.2 禁用 DroneCAN/UAVCAN

确保以下参数设置为禁用状态：

```bash
param set UAVCAN_ENABLE 0
param set CYPHAL_ENABLE 0
```

## 3. 添加驱动到启动脚本

### 3.1 创建启动脚本

在 `ROMFS/px4fmu_common/init.d/` 目录下创建或编辑启动脚本，添加：

```bash
# 启动 DJ_MK 舵机驱动
if param compare DJ_MK_ENABLE 1
then
    dj_mk start
fi
```

### 3.2 或者在 rc.sensors 中添加

编辑 `ROMFS/px4fmu_common/init.d/rc.sensors`，在适当位置添加：

```bash
# DJ_MK servo driver
if param compare DJ_MK_ENABLE 1
then
    if dj_mk start
    then
        echo "DJ_MK: started"
    else
        echo "DJ_MK: start failed"
    fi
fi
```

## 4. 设置参数

### 4.1 通过 QGroundControl

1. 连接飞控到 QGroundControl
2. 进入 Parameters 页面
3. 搜索 "DJ_MK"
4. 设置以下参数：
   - `DJ_MK_ENABLE` = 1 (启用驱动)
   - `DJ_MK_SERVO_ID` = 1 (舵机ID)
   - `DJ_MK_SPEED` = 256 (舵机速度)

### 4.2 通过 MAVLink 控制台

```bash
param set DJ_MK_ENABLE 1
param set DJ_MK_SERVO_ID 1
param set DJ_MK_SPEED 256
param save
```

## 5. 配置混控器

### 5.1 创建自定义混控器文件

创建文件 `ROMFS/px4fmu_common/mixers/dj_mk_servo.main.mix`：

```
# DJ_MK Servo Mixer
# 将第一个执行器输出映射到舵机

# Servo 1
M: 1
S: 0 0  10000  10000      0 -10000  10000
```

### 5.2 在启动脚本中加载混控器

```bash
# 加载 DJ_MK 混控器
if param compare DJ_MK_ENABLE 1
then
    mixer load /dev/pwm_output0 /etc/mixers/dj_mk_servo.main.mix
fi
```

## 6. 硬件连接

### 6.1 CAN 接口引脚

Pixhawk 6C CAN1 接口：
- CAN1_H: CAN 高电平信号
- CAN1_L: CAN 低电平信号
- GND: 地线
- VCC: 5V 电源（可选，用于供电）

### 6.2 连接舵机

1. 将舵机的 CAN_H 连接到飞控的 CAN1_H
2. 将舵机的 CAN_L 连接到飞控的 CAN1_L
3. 连接地线 (GND)
4. 为舵机提供独立供电（推荐）

### 6.3 终端电阻

如果舵机是 CAN 总线的最后一个设备，需要添加 120Ω 终端电阻。

## 7. 编译和烧录

```bash
# 清理之前的编译
make clean

# 编译固件
make px4_fmu-v6c_default

# 烧录固件
make px4_fmu-v6c_default upload
```

## 8. 测试

### 8.1 检查驱动状态

```bash
dj_mk status
```

### 8.2 手动测试

```bash
# 启动驱动
dj_mk start

# 发送测试命令（需要修改代码添加测试命令）
# 或者通过 actuator_test 命令测试
actuator_test set -m 0 -v 0.5
```

### 8.3 查看日志

```bash
dmesg
```

## 9. 故障排除

### 问题 1: 驱动无法启动

**症状**: `dj_mk start` 返回错误

**解决方案**:
1. 检查 `/dev/can0` 是否存在: `ls -l /dev/can*`
2. 检查 CAN 驱动是否加载: `lsmod | grep can`
3. 检查内核日志: `dmesg | grep can`

### 问题 2: 舵机无响应

**症状**: 驱动运行但舵机不动

**解决方案**:
1. 使用 CAN 分析仪检查总线上的数据
2. 确认舵机ID配置正确
3. 检查舵机供电
4. 验证 CAN 波特率 (250kbps)
5. 检查 CAN 线缆连接和终端电阻

### 问题 3: CAN 总线错误

**症状**: 大量 CAN 错误日志

**解决方案**:
1. 检查 CAN_H 和 CAN_L 是否接反
2. 检查终端电阻是否正确
3. 检查总线负载（设备数量）
4. 降低发送频率

## 10. 高级配置

### 10.1 多舵机支持

如需控制多个舵机，修改 `dj_mk.cpp` 中的 `Run()` 函数：

```cpp
// 控制多个舵机
for (int i = 0; i < num_servos; i++) {
    float output_value = actuator_outputs.output[i];
    uint16_t pwm_value = 1500 + (int16_t)(output_value * 500.0f);
    uint16_t position = pwm_to_position(pwm_value);
    send_servo_command(i + 1, position, _servo_speed);
}
```

### 10.2 添加反馈读取

实现 CAN 接收函数来读取舵机状态反馈。

### 10.3 自定义控制频率

修改 `SAMPLE_RATE` 常量来调整控制频率（默认 50Hz）。

## 11. 参考资料

- [PX4 驱动开发文档](https://docs.px4.io/main/en/middleware/drivers.html)
- [NuttX CAN 驱动](https://nuttx.apache.org/docs/latest/components/drivers/special/can.html)
- ASMG-MD 舵机用户手册
- Pixhawk 6C 硬件文档
