/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>

#include <sys/socket.h>
#include <net/if.h>
#include <netpacket/can.h>
#include <nuttx/can.h>

// Forward declarations for NuttX network functions
__BEGIN_DECLS
struct net_driver_s;
FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname);
int netdev_ifup(FAR struct net_driver_s *dev);
__END_DECLS

using namespace time_literals;

/**
 * ASMG-MD 舵机 CAN 控制帧结构
 * 帧ID: 0x18EF0201 (扩展帧)
 * 数据长度: 8字节
 * 格式: FF FF 01 01 POSH POSL SPEEDH SPEEDL
 */
typedef struct __attribute__((packed))
{
	uint8_t header[2];      // FF FF - 帧头
	uint8_t servo_id;       // 01 - 舵机ID
	uint8_t cmd_type;       // 01 - 位置控制命令
	uint16_t position;      // 位置值 (0x0000 ~ 0x7FFF, 中间位置 0x2000)
	uint16_t speed;         // 速度值 (推荐 0x0100)
} ServoControlFrame;

// Device state
enum DeviceState {
	STATE_UNINITIALIZED = 0,
	STATE_INITIALIZED = 1,
	STATE_RUNNING = 2,
	STATE_ERROR = 3
};

class DjMk : public ModuleBase<DjMk>, public px4::ScheduledWorkItem
{
public:
	DjMk(const char *can_dev = "can1");
	virtual ~DjMk();

	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]);
	static int task_spawn(int argc, char *argv[]);

	int init();
	int start();

private:
	static constexpr uint32_t SAMPLE_RATE{50};  // 50Hz 控制频率
	static constexpr uint32_t CAN_FRAME_ID{0x18EF0201};  // 发送帧ID（主机->舵机）
	static constexpr uint32_t CAN_REPLY_ID{0x18EF0102};  // 接收帧ID（舵机->主机）
	static constexpr uint16_t SERVO_POS_MIN{0x0000};     // 最小位置
	static constexpr uint16_t SERVO_POS_MAX{0x7FFF};     // 最大位置
	static constexpr uint16_t SERVO_POS_CENTER{0x2000}; // 中间位置
	static constexpr uint16_t SERVO_SPEED_DEFAULT{0x0100}; // 默认速度

	void Run() override;

	/**
	 * 打开 CAN socket
	 */
	int open_can_socket();

	/**
	 * 发送舵机控制命令
	 * @param servo_id 舵机ID (1-255)
	 * @param position 位置值 (0x0000 ~ 0x7FFF)
	 * @param speed 速度值
	 * @return 成功返回发送字节数，失败返回负值
	 */
	int16_t send_servo_command(uint8_t servo_id, uint16_t position, uint16_t speed);

	/**
	 * 将PWM值 (1000-2000us) 映射到舵机位置 (0x0000-0x7FFF)
	 */
	uint16_t pwm_to_position(uint16_t pwm_value);

	// CAN 设备
	int _can_fd{-1};
	const char *_can_dev;

	// 设备状态
	DeviceState _state{STATE_UNINITIALIZED};
	uint64_t _socket_open_time{0};

	// 订阅执行器输出
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};

public:
	// 公开成员变量供 custom_command 访问
	int32_t _servo_id{1};
	int32_t _servo_speed{256};

	// 用于 custom_command -> Run() 跨任务通信
	volatile bool _cmd_pending{false};       // 是否有待执行的命令
	volatile bool _query_mode{false};        // 是否为查询模式
	volatile bool _raw_mode{false};          // 是否为原始数据模式
	volatile uint16_t _cmd_position{0x2000}; // 待发送的位置值
	volatile uint16_t _cmd_speed{256};       // 待发送的速度值
	volatile uint8_t _cmd_servo_id{1};       // 待发送的舵机ID
	volatile uint8_t _raw_data[8];           // 原始数据缓冲区

	// 查询响应跟踪
	volatile bool _waiting_for_response{false};     // 是否正在等待响应
	volatile bool _query_response_received{false};  // 是否收到响应
	volatile uint64_t _query_start_time{0};         // 查询开始时间
	volatile bool _continuous_query_mode{false};    // 持续查询模式
	volatile uint64_t _last_query_time{0};          // 上次查询时间

	// 统计信息
	volatile uint32_t _rx_frame_count{0};           // 接收到的总帧数
	volatile uint32_t _tx_frame_count{0};           // 发送的总帧数
	volatile uint32_t _query_sent_count{0};         // 发送的查询次数
	volatile uint32_t _query_response_count{0};     // 收到的查询响应次数
};
