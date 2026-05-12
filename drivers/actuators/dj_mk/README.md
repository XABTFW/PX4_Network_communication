# DJ_MK - ASMG-MD Servo Driver

## 概述

这是一个用于通过 CAN 总线控制 ASMG-MD 系列舵机的 PX4 原生驱动模块。

## 硬件配置

- **飞控**: Pixhawk 6C
- **舵机**: ASMG-MD 系列
- **CAN 端口**: CAN1
- **波特率**: 250kbps
- **协议**: 不使用 DroneCAN/UAVCAN

## 通信协议

### CAN 帧格式

- **帧类型**: 扩展帧 (EXT)
- **帧 ID**: `0x18EF0201`
- **数据长度**: 8 字节
- **数据格式**: `FF FF [ID] 01 [POS_H] [POS_L] [SPEED_H] [SPEED_L]`

### 字段说明

| 字节 | 说明 | 值 |
|------|------|-----|
| 0-1  | 帧头 | `0xFF 0xFF` |
| 2    | 舵机ID | `0x01` (1号舵机) |
| 3    | 命令类型 | `0x01` (位置控制) |
| 4    | 位置高字节 | `0x00-0x7F` |
| 5    | 位置低字节 | `0x00-0xFF` |
| 6    | 速度高字节 | `0x01` (推荐) |
| 7    | 速度低字节 | `0x00` (推荐) |

### 位置范围

- **最小位置**: `0x0000`
- **中间位置**: `0x2000` (对应 1500us PWM)
- **最大位置**: `0x7FFF`

## 使用方法

### 1. 编译驱动

在 PX4 固件中启用该模块：

```bash
# 在 boards/px4/fmu-v6c/default.px4board 中添加
CONFIG_DRIVERS_ACTUATORS_DJ_MK=y
```

或者在 menuconfig 中启用：

```bash
make px4_fmu-v6c_default menuconfig
# 导航到: Device Drivers -> Actuators -> dj_mk
```

### 2. 启动驱动

在启动脚本中添加：

```bash
dj_mk start
```

或者在 MAVLink 控制台中手动启动：

```bash
dj_mk start
```

### 3. 参数配置

可用参数：

- `DJ_MK_ENABLE`: 启用驱动 (0=禁用, 1=启用)
- `DJ_MK_SERVO_ID`: 舵机ID (1-255)
- `DJ_MK_SPEED`: 舵机速度 (0-65535, 默认256)

### 4. 测试

使用 MAVLink 命令测试舵机：

```bash
# 查看驱动状态
dj_mk status

# 停止驱动
dj_mk stop
```

## PWM 映射

驱动会自动将 PWM 值映射到舵机位置：

- 1000us → 0x0000 (最小位置)
- 1500us → 0x2000 (中间位置)
- 2000us → 0x7FFF (最大位置)

## 注意事项

1. 确保 CAN1 端口已正确配置并启用
2. 确保 CAN 波特率设置为 250kbps
3. 确保已关闭 DroneCAN/UAVCAN 功能
4. 舵机ID必须与实际硬件ID匹配

## 故障排除

### 驱动无法启动

- 检查 `/dev/can0` 设备是否存在
- 检查 CAN 总线是否正确初始化
- 查看系统日志: `dmesg`

### 舵机无响应

- 检查 CAN 线缆连接
- 检查舵机供电
- 使用 CAN 分析仪验证帧格式
- 确认舵机ID配置正确

## 开发者信息

### 文件结构

```
src/drivers/actuators/dj_mk/
├── dj_mk.cpp           # 主驱动实现
├── dj_mk.hpp           # 头文件
├── dj_mk_params.c      # 参数定义
├── CMakeLists.txt      # 构建配置
├── Kconfig             # 内核配置
├── module.yaml         # 模块配置
└── README.md           # 本文档
```

### 扩展功能

如需添加更多功能，可以修改：

1. 支持多个舵机：修改 `Run()` 函数中的循环逻辑
2. 添加反馈读取：实现 CAN 接收函数
3. 添加错误检测：解析舵机返回的状态帧

## 参考资料

- ASMG-MD 舵机数据手册
- PX4 驱动开发指南
- NuttX CAN 驱动文档
