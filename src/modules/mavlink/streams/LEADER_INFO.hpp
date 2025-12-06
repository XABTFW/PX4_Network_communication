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

#ifndef LEADER_INFO_HPP
#define LEADER_INFO_HPP

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/geo/geo.h>
#include <drivers/drv_hrt.h>


class MavlinkStreamLeaderInfo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLeaderInfo(mavlink); }

	static constexpr const char *get_name_static() { return "LEADER_INFO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_LEADER_INFO; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _lpos_sub.advertised() ? MAVLINK_MSG_ID_LEADER_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit  MavlinkStreamLeaderInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _leader_id_sub{ORB_ID(leader_id)};
	uORB::Subscription _tra_pos_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _leader_status_sub{ORB_ID(vehicle_status)};


	bool send() override
	{


	double leader_lat,leader_lon;
	leader_id_s _leader_id;
	_leader_id_sub.copy(&_leader_id);


       // bool set_as_leader = _leader_id.leader;

	//if(set_as_leader){
		if ( _lpos_sub.updated() && _lpos_sub.updated()) {
			vehicle_local_position_s sp_local_pos;
			trajectory_setpoint_s sp_tra_pos;
			vehicle_status_s _leader_status;
			_lpos_sub.copy(&sp_local_pos);
			_tra_pos_sub.copy(&sp_tra_pos);
			_leader_status_sub.copy(&_leader_status);


			MapProjection _global_local_proj_ref{};
			_global_local_proj_ref.initReference(sp_local_pos.ref_lat,sp_local_pos.ref_lon,hrt_absolute_time());
			_global_local_proj_ref.reproject(sp_tra_pos.position[0],sp_tra_pos.position[1],leader_lat,leader_lon);


			mavlink_leader_info_t msg = {};
			msg.lat = leader_lat;
			msg.lon = leader_lon;
			msg.rel_alt = sp_tra_pos.position[2];
			msg.vx = sp_tra_pos.velocity[0];
			msg.vy = sp_tra_pos.velocity[1];
			msg.vz = sp_tra_pos.velocity[2];
			msg.yaw = sp_local_pos.heading;
			msg.yaw_speed = sp_local_pos.delta_heading;

			if(_leader_status.arming_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND){
				msg.land = 1;

				}
			else{
				msg.land = 0;
			    }


			 // 获取 MAV_SYS_ID 参数
			int32_t mav_sys_id = -1;  // 初始化变量
			param_t param_id = param_find("MAV_SYS_ID");  // 查找 MAV_SYS_ID 参数
			if (param_id != PARAM_INVALID) {
			    param_get(param_id, &mav_sys_id);  // 获取 MAV_SYS_ID 的值
			    //PX4_INFO("MAV_SYS_ID is %d", mav_sys_id);  // 输出 MAV_SYS_ID 值
			    msg.mavid = mav_sys_id;  // 将 MAV_SYS_ID 的值赋给 msg.mavid
			} else {
			    PX4_ERR("MAV_SYS_ID not found");
			    msg.mavid = 0;  // 如果找不到 MAV_SYS_ID，则设置为默认值 0
			}   

			mavlink_msg_leader_info_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
	//}
		return false;
	}
};

#endif // ESTIMATOR_STATUS_HPP
