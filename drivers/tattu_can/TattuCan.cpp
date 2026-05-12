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

/**
 * @file TattuCan.cpp
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 *
 * Driver for the Tattu 12S 1600mAh Smart Battery connected over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Tattu datasheet DOC 001 REV D, which is highly
 * specific to the 12S 1600mAh battery. Other models of Tattu batteries
 * will NOT work with this driver in its current form.
 *
 */

#include "TattuCan.hpp"

extern orb_advert_t mavlink_log_pub;// MAVLink日志发布器，用于发布日志消息
//uavcan 是 PX4 中 CAN 相关的工作队列
TattuCan::TattuCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)//
{
}

TattuCan::~TattuCan()
{
}
// 定时任务的主函数，负责接收CAN帧并发布电池状态
void TattuCan::Run()
{
	// 如果模块应该退出，调用exit_and_cleanup函数清理资源并退出
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}
	// 如果模块尚未初始化，尝试打开CAN设备文件/dev/can0，并将文件描述符保存在_fd成员变量中
	if (!_initialized) {

		_fd = ::open("/dev/can0", O_RDWR);

		if (_fd < 0) {// 如果打开失败，打印错误信息并返回
			PX4_INFO("FAILED TO OPEN /dev/can0");
			return;
		}

		_initialized = true;// 将初始化标志设置为true，表示模块已成功初始化
	}
	// 定义一个64字节的缓冲区data，用于存储从CAN总线接收的数据
	uint8_t data[64] {};
	//定义一个CanFrame结构体变量received_frame，用于存储接收的CAN帧信息，并将payload指针指向data缓冲区
	CanFrame received_frame{};
	received_frame.payload = &data;
	////定义一个Tattu12SBatteryMessage结构体变量tattu_message，用于存储解析后的电池信息
	Tattu12SBatteryMessage tattu_message = {};
	//// 循环调用receive函数接收CAN帧，直到没有更多数据可读
	while (receive(&received_frame) > 0) {

		//找到传输开始的CAN帧，判断条件是：接收的CAN帧数据长度为8字节，并且最后一个字节的值等于TAIL_BYTE_START_OF_TRANSFER（128），如果不满足条件则继续接收下一帧
		if ((received_frame.payload_size == 8) && ((uint8_t *)received_frame.payload)[7] == TAIL_BYTE_START_OF_TRANSFER) {
		} else {
			continue;
		}

		// 从接收的CAN帧数据中提取电池信息，首先将前2字节（manufacturer和sku）复制到tattu_message结构体中，然后根据接收的CAN帧数据长度和内容继续复制剩余的电池信息，直到接收到表示传输结束的CAN帧（最后一个字节值为TAIL_BYTE_END_OF_TRANSFER，64）
		size_t offset = 5;// 前2字节为manufacturer和sku，后3字节为电池信息的前5个参数（voltage、current、temperature、remaining_percent、cycle_life），共5字节，因此offset初始值为5
		memcpy(&tattu_message, &(((uint8_t *)received_frame.payload)[2]), offset);
		// 循环调用receive函数继续接收CAN帧，并将接收到的电池信息复制到tattu_message结构体中，直到接收到表示传输结束的CAN帧（最后一个字节值为TAIL_BYTE_END_OF_TRANSFER，64）
		while (receive(&received_frame) > 0) {

			size_t payload_size = received_frame.payload_size - 1;// 每帧数据长度减去1字节的尾部标志
			offset += payload_size;// 更新偏移量，准备复制下一段电池信息
			//将接收到的电池信息复制到tattu_message结构体中，偏移量为offset，复制长度为payload_size
			memcpy(((char *)&tattu_message) + offset, received_frame.payload, payload_size);
		}
		// 定义一个battery_status_s结构体变量battery_status，用于存储要发布的电池状态信息
		battery_status.timestamp = hrt_absolute_time();
		battery_status.connected = true;// 将电池连接状态设置为true，表示电池已连接
		battery_status.cell_count = 12;// 将电池单体数量设置为12，表示这是一个12S电池
		//将电池制造商ID转换为字符串并存储在battery_status的serial_number成员中
		sprintf(battery_status.serial_number, "%d", tattu_message.manufacturer);
		// 将电池型号转换为8位无符号整数并存储在battery_status的id成员中
		battery_status.id = static_cast<uint8_t>(tattu_message.sku);

		battery_status.cycle_count = tattu_message.cycle_life;// 将电池循环寿命直接赋值给battery_status的cycle_count成员
		battery_status.state_of_health = static_cast<uint16_t>(tattu_message.health_status);// 将电池健康状态直接赋值给battery_status的state_of_health成员

		battery_status.voltage_v = static_cast<float>(tattu_message.voltage) / 1000.0f;// 将电池电压转换为伏特并存储在battery_status的voltage_v成员中，单位为V
		battery_status.current_a = static_cast<float>(tattu_message.current) / 1000.0f;// 将电池电流转换为安培并存储在battery_status的current_a成员中，单位为A
		battery_status.remaining = static_cast<float>(tattu_message.remaining_percent) / 100.0f;// 将电池剩余电量百分比转换为0-1之间的值并存储在battery_status的remaining成员中，单位为%
		battery_status.temperature = static_cast<float>(tattu_message.temperature);// 将电池温度直接赋值给battery_status的temperature成员中，单位为摄氏度
		battery_status.capacity = tattu_message.standard_capacity;// 将电池标准容量直接赋值给battery_status的capacity成员中，单位为mAh
		battery_status.voltage_cell_v[0] = static_cast<float>(tattu_message.cell_1_voltage) / 1000.0f;// 将电池单体电压转换为伏特并存储在battery_status的voltage_cell_v数组中，单位为V
		battery_status.voltage_cell_v[1] = static_cast<float>(tattu_message.cell_2_voltage) / 1000.0f;
		battery_status.voltage_cell_v[2] = static_cast<float>(tattu_message.cell_3_voltage) / 1000.0f;
		battery_status.voltage_cell_v[3] = static_cast<float>(tattu_message.cell_4_voltage) / 1000.0f;
		battery_status.voltage_cell_v[4] = static_cast<float>(tattu_message.cell_5_voltage) / 1000.0f;
		battery_status.voltage_cell_v[5] = static_cast<float>(tattu_message.cell_6_voltage) / 1000.0f;
		battery_status.voltage_cell_v[6] = static_cast<float>(tattu_message.cell_7_voltage) / 1000.0f;
		battery_status.voltage_cell_v[7] = static_cast<float>(tattu_message.cell_8_voltage) / 1000.0f;
		battery_status.voltage_cell_v[8] = static_cast<float>(tattu_message.cell_9_voltage) / 1000.0f;
		battery_status.voltage_cell_v[9] = static_cast<float>(tattu_message.cell_10_voltage / 1000.0f);
		battery_status.voltage_cell_v[10] = static_cast<float>(tattu_message.cell_11_voltage) / 1000.0f;
		battery_status.voltage_cell_v[11] = static_cast<float>(tattu_message.cell_12_voltage) / 1000.0f;

		_battery_status_pub.publish(battery_status);
	}
}

int16_t TattuCan::receive(CanFrame *received_frame)// 从 CAN 总线接收一帧数据，并填充到CanFrame结构体
{
	// 如果CAN设备文件描述符无效或接收帧指针为空，打印错误信息并返回-1
	if ((_fd < 0) || (received_frame == nullptr)) {
		PX4_INFO("fd < 0");
		return -1;
	}

	struct pollfd fds {};// 定义一个pollfd结构体，用于监视CAN设备文件描述符的事件

	fds.fd = _fd;// 将CAN设备文件描述符赋值给pollfd结构体的fd成员

	fds.events = POLLIN;// 设置pollfd结构体的events成员为POLLIN，表示我们关心可读事件，即有数据可读

	// 调用poll函数，监视CAN设备文件描述符的事件，参数依次为：pollfd结构体数组、结构体数量（1）和超时时间（0，表示立即返回）
	::poll(&fds, 1, 0);


	if (fds.revents & POLLIN) {// 如果poll函数返回的事件中包含POLLIN，表示CAN设备文件描述符有数据可读

		// Try to read.
		struct can_msg_s receive_msg;// 定义一个can_msg_s结构体变量，用于存储从CAN总线接收的数据
		// 调用read函数，从CAN设备文件描述符中读取数据，参数依次为：文件描述符、接收缓冲区和缓冲区大小，返回值为实际读取的字节数
		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));
		// 如果读取失败或读取的字节数不在预期范围内，打印错误信息并返回-1
		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			PX4_INFO("error");
			return -1;

		} else {
			// 将接收到的CAN消息的ID赋值给received_frame结构体的extended_can_id成员
			received_frame->extended_can_id = receive_msg.cm_hdr.ch_id;
			// 将接收到的CAN消息的数据长度赋值给received_frame结构体的payload_size成员
			received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
			// 将接收到的CAN消息的数据内容复制到received_frame结构体的payload成员指向的缓冲区中，复制的字节数为数据长度
			memcpy((void *)received_frame->payload, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
			// 返回实际读取的字节数
			return nbytes;
		}
	}

	return 0;// 如果没有数据可读，返回0
}

int TattuCan::start()
{
	//启动时存在竞态条件，因此添加了一个延迟，以确保模块正确启动并开始接收CAN帧
	uint32_t delay_us = 500000;// 延迟500毫秒，单位为微秒
	// 调用ScheduleOnInterval函数，设置定时任务的执行间隔为1000毫秒除以采样率（即每100毫秒执行一次），并在启动时延迟500毫秒开始执行
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;// 返回PX4_OK表示模块启动成功
}
// 生成模块任务（PX4任务调度）
int TattuCan::task_spawn(int argc, char *argv[])
{
	TattuCan *instance = new TattuCan();// 创建TattuCan类的实例，如果创建失败，打印错误信息并返回PX4_ERROR

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);// 将创建的实例存储在ModuleBase类的静态成员_object中，以便其他函数可以访问该实例
	// 将ModuleBase类的静态成员_task_id设置为task_id_is_work_queue，表示该模块将作为工作队列任务运行
	_task_id = task_id_is_work_queue;

	instance->start();// 调用实例的start函数，启动模块
	return 0;// 返回0表示任务生成成功
}
// 打印模块用法（如help命令）
int TattuCan::print_usage(const char *reason)
{
	if (reason) {// 如果传入了reason参数，打印该参数作为错误信息
		printf("%s\n\n", reason);
	}
// 打印模块描述信息，使用PRINT_MODULE_DESCRIPTION宏，传入一个字符串参数，描述模块的功能和用途
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for reading data from the Tattu 12S 16000mAh smart battery.

)DESCR_STR");
	// 打印模块用法信息，使用PRINT_MODULE_USAGE_COMMAND宏，传入一个字符串参数，描述模块的命令行用法，例如如何启动模块
	PRINT_MODULE_USAGE_NAME("tattu_can", "system");
	// 打印模块命令信息，使用PRINT_MODULE_USAGE_COMMAND宏，传入一个字符串参数，描述模块的命令行用法，例如如何启动模块
	PRINT_MODULE_USAGE_COMMAND("start");
	// 打印模块默认命令信息，使用PRINT_MODULE_USAGE_DEFAULT_COMMANDS宏，打印模块的默认命令行用法，例如help命令
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;// 返回0表示打印用法成功
}
// 处理自定义命令（如模块启动/停止）
int TattuCan::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}
// 生成模块任务（PX4任务调度）
extern "C" __EXPORT int tattu_can_main(int argc, char *argv[])
{
	return TattuCan::main(argc, argv);
}
