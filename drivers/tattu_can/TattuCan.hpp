/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <nuttx/can/can.h>//can总线驱动头文件

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/battery_status.h>

using namespace time_literals;

typedef struct __attribute__((packed))
{
	int16_t     manufacturer;//电池制造商id
	uint16_t    sku;//电池型号
	uint16_t    voltage;//电池电压，单位为0.1V
	int16_t     current;//电池电流，单位为0.1A
	int16_t     temperature;//电池温度，单位为0.1摄氏度
	uint16_t    remaining_percent;//电池剩余电量百分比，单位为0.1%
	uint16_t    cycle_life;//电池循环寿命，单位为0.1%
	int16_t     health_status;//电池健康状态，0-100%，单位为0.1%
	uint16_t    cell_1_voltage;//电池单体电压，单位为0.1V
	uint16_t    cell_2_voltage;//电池单体电压，单位为0.1V
	uint16_t    cell_3_voltage;
	uint16_t    cell_4_voltage;
	uint16_t    cell_5_voltage;
	uint16_t    cell_6_voltage;
	uint16_t    cell_7_voltage;
	uint16_t    cell_8_voltage;
	uint16_t    cell_9_voltage;
	uint16_t    cell_10_voltage;
	uint16_t    cell_11_voltage;
	uint16_t    cell_12_voltage;
	uint16_t    standard_capacity;//电池标准容量，单位为mAh
	uint16_t    remaining_capacity_mah;//电池剩余容量，单位为mAh
	uint32_t    error_info;//电池错误信息，bit0-7为过压错误，bit8-15为欠压错误，bit16-23为过流错误，bit24-31为过温错误
} Tattu12SBatteryMessage;//电池信息结构体，包含电池的各种参数和状态信息

typedef struct {
	uint64_t timestamp_usec;//CAN帧接收时间戳（微秒）
	uint32_t extended_can_id;//CAN帧的扩展ID
	size_t      payload_size;//CAN帧数据长度
	const void *payload;//CAN帧数据指针，指向Tattu12SBatteryMessage结构体
} CanFrame;//CAN帧结构体，包含CAN帧的基本信息和数据内容
//TattuCan类，继承自ModuleBase和ScheduledWorkItem，用于实现电池信息的CAN总线通信和处理
class TattuCan : public ModuleBase<TattuCan>, public px4::ScheduledWorkItem
{
public:
	TattuCan();

	virtual ~TattuCan();

	// 打印模块用法（如help命令）
	static int print_usage(const char *reason = nullptr);
	// 处理自定义命令（如模块启动/停止）
	static int custom_command(int argc, char *argv[]);
	// 生成模块任务（PX4任务调度）
	static int task_spawn(int argc, char *argv[]);
	// 启动模块（打开CAN设备、启动定时任务）
	int start();
	// 从 CAN 总线接收一帧数据，并填充到CanFrame结构体
	int16_t receive(CanFrame *received_frame);

private:
	static constexpr uint32_t SAMPLE_RATE{100}; // 采样率，单位为Hz
	static constexpr size_t TAIL_BYTE_START_OF_TRANSFER{128};// CAN帧尾部字节值，表示传输开始
	static constexpr size_t TAIL_BYTE_END_OF_TRANSFER{64};// CAN帧尾部字节值，表示传输结束

	void Run() override;// 定时任务的主函数，负责接收CAN帧并发布电池状态

	int _fd{-1};// CAN设备文件描述符，初始化为-1表示未打开
	bool _initialized{false};// 模块初始化状态标志，初始值为false表示未初始化
	// uORB发布器，用于发布电池状态消息，关联到battery_status主题
	uORB::Publication<battery_status_s> _battery_status_pub{ORB_ID::battery_status};
};
