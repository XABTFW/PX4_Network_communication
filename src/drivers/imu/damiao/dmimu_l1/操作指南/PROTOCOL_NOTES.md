# DM-IMU L1 协议版本说明

## 协议版本对比

达妙 IMU 存在两种不同的 CAN 通信协议，可能对应不同的固件版本或使用场景。

### 版本 1：例程协议（当前实现）

**来源**：MC02_CAN收发例程 (`dm_imu.c`)

**命令帧格式** (8字节)：
```
[0xCC][reg_id][cmd][0xDD][data0][data1][data2][data3]
```

**发送方式**：
```c
msg.cm_hdr.ch_id = _can_id;      // 使用设备的 CAN ID
msg.cm_hdr.ch_dlc = 8;            // 8字节数据
```

**特点**：
- 直接寻址，每个设备有独立的 CAN ID
- 支持读写操作（cmd: 0=读, 1=写）
- 可携带 4 字节数据
- 响应 ID = 发送 ID + 0x10

**示例代码**：
```cpp
int DmImuL1::can_send_cmd(uint8_t reg_id, uint8_t cmd, uint32_t data)
{
    struct can_msg_s msg;
    msg.cm_hdr.ch_id = _can_id;
    msg.cm_hdr.ch_dlc = 8;
    
    DmImuCmdFrame *frame = (DmImuCmdFrame *)msg.cm_data;
    frame->header = 0xCC;
    frame->reg_id = reg_id;
    frame->cmd = cmd;
    frame->delimiter = 0xDD;
    frame->data = data;
    
    write(_can_fd, &msg, sizeof(msg));
}
```

---

### 版本 2：手册协议（未实现）

**来源**：官方手册附录三 (`IMU_RequestData`)

**命令帧格式** (4字节)：
```
[can_id_low][can_id_high][reg][0xCC]
```

**发送方式**：
```c
tx_header.StdId = 0x6FF;          // 固定的广播ID
tx_header.DLC = 4;                // 4字节数据
```

**特点**：
- 广播式请求，使用固定 ID 0x6FF
- 通过数据内容指定目标设备
- 只支持请求操作
- 数据长度更短

**示例代码**（如需实现）：
```cpp
int DmImuL1::can_send_request_v2(uint8_t reg_id)
{
    struct can_msg_s msg;
    msg.cm_hdr.ch_id = 0x6FF;     // 固定广播ID
    msg.cm_hdr.ch_dlc = 4;
    
    uint16_t target_id = (_can_id << 8) | _mst_id;
    msg.cm_data[0] = (uint8_t)target_id;
    msg.cm_data[1] = (uint8_t)(target_id >> 8);
    msg.cm_data[2] = reg_id;
    msg.cm_data[3] = 0xCC;
    
    write(_can_fd, &msg, sizeof(msg));
}
```

---

## 数据帧格式（两种协议通用）

**加速度/陀螺仪/欧拉角** (8字节)：
```
[data_type][reserved][data0_L][data0_H][data1_L][data1_H][data2_L][data2_H]
```

**四元数** (8字节，压缩格式)：
```
[data_type][w13:6][w5:0,x13:8][x7:0][y13:6][y5:0,z13:8][z7:0]
```

数据类型：
- 1 = 加速度
- 2 = 陀螺仪
- 3 = 欧拉角
- 4 = 四元数

---

## 如何切换协议

如果你的 IMU 固件使用手册中的协议（版本2），需要修改以下内容：

### 1. 修改命令发送函数

在 `dm_imu_l1.cpp` 中替换 `can_send_cmd()` 函数：

```cpp
int DmImuL1::can_send_cmd(uint8_t reg_id, uint8_t cmd, uint32_t data)
{
    if (_can_fd < 0) {
        return -1;
    }
    
    struct can_msg_s msg;
    memset(&msg, 0, sizeof(msg));
    
    // 使用手册协议
    msg.cm_hdr.ch_id = 0x6FF;  // 固定广播ID
    msg.cm_hdr.ch_dlc = 4;
    msg.cm_hdr.ch_extid = 0;
    
    uint16_t target_id = (_can_id << 8) | _mst_id;
    msg.cm_data[0] = (uint8_t)target_id;
    msg.cm_data[1] = (uint8_t)(target_id >> 8);
    msg.cm_data[2] = reg_id;
    msg.cm_data[3] = 0xCC;
    
    int ret = write(_can_fd, &msg, sizeof(msg));
    
    if (ret != sizeof(msg)) {
        PX4_ERR("CAN send failed");
        return -1;
    }
    
    return 0;
}
```

### 2. 注意事项

- 手册协议只支持请求操作，不支持写入数据
- 所有配置功能（校准、参数设置等）可能需要不同的实现方式
- 需要确认你的 IMU 固件版本支持哪种协议

---

## CRC16 校验（可选）

官方手册附录四提供了 CRC16 校验函数，但在例程中未使用。

如需添加 CRC 校验，可以参考手册实现：

```cpp
const uint16_t CRC16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    // ... 完整表格见手册附录四
};

uint16_t get_crc16(uint8_t *ptr, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        uint8_t index = (crc >> 8 ^ ptr[i]);
        crc = ((crc << 1) ^ CRC16_table[index]);
    }
    return crc;
}
```

---

## 推荐做法

1. **优先使用当前实现**（版本1）- 已在实际硬件上验证
2. **如果遇到通信问题**，尝试切换到版本2协议
3. **咨询厂家技术支持**，确认你的固件版本使用哪种协议
4. **CRC 校验**目前可选，如有数据可靠性问题再考虑添加

---

## 参考资料

- MC02_CAN收发例程：`dm_imu.c`, `dm_imu.h`
- 官方手册：DM-IMU-L1 使用说明书 V1.2
  - 附录二：线性映射表及转换函数
  - 附录三：CAN 解析例程
  - 附录四：CRC16 校验程序

---

## 技术支持

如有疑问，请联系：
- 达妙科技 QQ 群：320296121
- 官网：https://www.dm-robot.com/
