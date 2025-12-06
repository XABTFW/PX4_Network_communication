#include "control_instance.h"
control_instance* control_instance::instance = nullptr;



bool control_instance::Change_offborad()
{
	_vehicle_status_sub.copy(&_status);
	_vehicle_local_position_sub.copy(&_vehicle_local_position);

	_command.target_system = _status.system_id;
	_command.target_component = _status.component_id;

	sp_local.x = _vehicle_local_position.x;
	sp_local.y = _vehicle_local_position.y;
	sp_local.z = _vehicle_local_position.z - 0.1f;
	sp_local.timestamp = hrt_absolute_time();

	ocm.timestamp = hrt_absolute_time();
	ocm.position = true;
	ocm.velocity = false;
	ocm.acceleration = false;

	_trajectory_setpoint_pub.publish(sp_local);

	for (int i = 0; i < 20; ++i) {
	_offboard_control_mode_pub.publish(ocm);

	if (i == 0) {
	_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	_command.param1 = 1.0f;
	_command.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
	_command.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(_command);
	}

	px4_usleep(50000);

	_vehicle_status_sub.copy(&_status);
	if (_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
	PX4_INFO("Entered OFFBOARD");
	return true;
	}
	}

	PX4_WARN("Failed to enter OFFBOARD mode");
	return false;
}



bool control_instance::Change_Auto()
{
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _command.param1=1.0f;
        _command.param2=PX4_CUSTOM_MAIN_MODE_AUTO;
        _command.param3=PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);

int ret = 10;
while (ret-->0)
{
        /* code */
        _vehicle_status_sub.copy(&_status);
        if(_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION){
        return true;
        }
         px4_usleep(100000);
           }
        return false;


}




bool control_instance::Auto_hold()
{
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _command.param1=1.0f;
        _command.param2=PX4_CUSTOM_MAIN_MODE_AUTO;
        _command.param3=PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);

int rett = 10;
while (rett-->0)
{
        /* code */
        _vehicle_status_sub.copy(&_status);
         if(_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER){
        return true;
        }
         px4_usleep(100000);
           }
        return false;

}


bool control_instance::Change_position_mode()
 {
        //mdc 2025-modifily
      //  bool _isChangeposition_mode =false;
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _command.param1=1.0f;
        _command.param2=PX4_CUSTOM_MAIN_MODE_POSCTL;
        _command.param3=PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);
        //等待模式切换
int ret = 10;
while (ret-->0)
{
        /* code */
        _vehicle_status_sub.copy(&_status);
         if(_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL){
             return true;
                      }
         px4_usleep(100000);
           }
        return false;


}

//解锁无人机
bool control_instance::Arm_vehicle()
{
        _command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        _command.param1 = 1.0f;
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);

        int ret = 10;
while (ret-->0)
{
        /* code */
        _vehicle_status_sub.copy(&_status);
         if(_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED)
        {
        return true;
        }
         px4_usleep(100000);
           }
        return false;


}



bool control_instance::Control_posxyz(float x,float y,float z)
{
 memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=x;
                sp_local.y=y;
                sp_local.z=z;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                Pubctl();
                 _vehicle_local_position_sub.copy(&_vehicle_local_position);
                 _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
                                 if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)
                                 {
                if((abs(_vehicle_local_position.x-sp_local.x)<fw_xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<fw_xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<fw_z_rad))
                {
                return true;
                }
                else
                {
                return false;
                }
                                 }
else
{
                if((abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))
                {
                return true;
                }
                else
                {
                return false;
                }
}
}



bool control_instance::Control_pos_vel_yaw(matrix::Vector3f pos, matrix::Vector3f vel, float yaw ,float yaw_speed)
{
        //更改后

 memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));

if(!PX4_ISFINITE(pos(0))|| !PX4_ISFINITE(pos(1)) || !PX4_ISFINITE(pos(2)))
{

        return false;
}
if(fabsf(pos(0))>1e6f || fabsf(pos(1))>1e6f)
{

        return false;
}

 memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));

                sp_local.x=pos(0);
                sp_local.y=pos(1);
                sp_local.z=pos(2);


                sp_local.vx=vel(0);
                sp_local.vy=vel(1);
                sp_local.vz=vel(2);


                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;


                sp_local.yaw = yaw;
                sp_local.yawspeed = yaw_speed;
                ocm.position=true;
                ocm.velocity=true;
                ocm.attitude=true;
                ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                Pubctl();
                 _vehicle_local_position_sub.copy(&_vehicle_local_position);
                 _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);

if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW){
                if((abs(_vehicle_local_position.x-sp_local.x)<fw_xy_rad)
                        &&(abs(_vehicle_local_position.y-sp_local.y)<fw_xy_rad)
                        &&(abs(_vehicle_local_position.z-sp_local.z)<fw_z_rad)){
                return true;
                }
                else
                {
                return false;
                }
                                 }
else
{
                if((abs(_vehicle_local_position.x-sp_local.x)<xy_rad)
                         &&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)
                         &&(abs(_vehicle_local_position.z-sp_local.z)<z_rad)){
                return true;
                }
                else
                {
                return false;
                }
}


}



bool control_instance::Control_posxyz_yawspeed(float x,float y,float z,float yawspeed)
{
return false;
}
bool control_instance::Control_posz_velxy(float vx,float vy,float z)
{
 return false;
}
bool control_instance::Control_posz_accxy(float ax,float ay,float z)
{
 return false;
}

//降落
bool control_instance::Change_land()
{
            _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
            _command.param1=1.0f;
            _command.param2=PX4_CUSTOM_MAIN_MODE_AUTO;
            _command.param3=PX4_CUSTOM_SUB_MODE_AUTO_LAND;
            _command.timestamp = hrt_absolute_time();
            _vehicle_status_sub.copy(&_status);
            _vehicle_command_pub.publish(_command);
                if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
        {
return true;
        }
        else
        {
        return false;
        }
}

bool control_instance::Control_lat_lon_alt(float lat,float lon,float alt)
{
        float x,y,z;
        vehicle_local_position_s local_pos{};
			_vehicle_local_position_sub.copy(&local_pos);

			if (!local_pos.xy_global || !local_pos.z_global) {
				return false;
			}

			MapProjection global_local_proj_ref{local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp};

			// global -> local
			// const double lat = target_global_int.lat_int / 1e7;
			// const double lon = target_global_int.lon_int / 1e7;
			global_local_proj_ref.project(lat, lon, x, y);

                       z =alt- local_pos.ref_alt;

 memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=x;
                sp_local.y=y;
                sp_local.z=z;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                time_tick=hrt_absolute_time();
                Pubctl();
                 _vehicle_local_position_sub.copy(&_vehicle_local_position);
                if((abs(_vehicle_local_position.x-sp_local.x)<xy_rad)&&(abs(_vehicle_local_position.y-sp_local.y)<xy_rad)&&(abs(_vehicle_local_position.z-sp_local.z)<z_rad))
                {
                return true;
                }
                else
                {
                return false;
                }
}



bool control_instance::hold_curr_pos(float time_s)

{
        vehicle_local_position_s local_pos{};
		_vehicle_local_position_sub.copy(&local_pos);
                memset(&sp_local,0,sizeof(vehicle_local_position_setpoint_s));
                sp_local.x=local_pos.x;
                sp_local.y=local_pos.y;
                sp_local.z=local_pos.z;
                sp_local.vx=(float)NAN;
                sp_local.vy=(float)NAN;
                sp_local.vz=(float)NAN;
                sp_local.acceleration[0]=(float)NAN;
                sp_local.acceleration[1]=(float)NAN;
                sp_local.acceleration[2]=(float)NAN;
                ocm.position=true;
                ocm.velocity=false;
                ocm.acceleration=false;
                Pubctl();
                if(!time_flag)
                {
                        time_tick=hrt_absolute_time();
                        time_flag=true;
                }
                if((hrt_absolute_time()-time_tick)>time_s*1000000)
                {
                return true;
                time_flag=true;
                }
                else
                {
                return false;
                }
                PX4_INFO("HOLD_CURR_POS");

}


void control_instance::Pubctl()
{
if(ocm.position||ocm.velocity||ocm.acceleration)
{
        ocm.timestamp = hrt_absolute_time();
        _offboard_control_mode_pub.publish(ocm);
        _vehicle_status_sub.copy(&_status);
if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
{
        sp_local.timestamp = hrt_absolute_time();
        _trajectory_setpoint_pub.publish(sp_local);
}
}
}

bool control_instance::Control_mc_to_fw()
{
        // _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        //     _command.param1=1.0f;
        //     _command.param2=PX4_CUSTOM_MAIN_MODE_POSCTL;
        //     _command.timestamp = hrt_absolute_time();
        //     _vehicle_command_pub.publish(_command);
        //     _vehicle_status_sub.copy(&_status);
        //         if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_POSCTL)
        // {
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
        _command.param1=(float)(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);
        // }
        _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
        PX4_INFO("mc_to_fw");
        if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW)
        {
//                 _vehicle_status_sub.copy(&_status);
//  _command.target_system = _status.system_id;
//         _command.target_component = _status.component_id;
//         ocm.timestamp = hrt_absolute_time();
//         _vehicle_local_position_sub.copy(&_vehicle_local_position);
//         sp_local.x=_vehicle_local_position.x+100;
//         sp_local.y=_vehicle_local_position.y;
//         sp_local.z=_vehicle_local_position.z-50;
//         sp_local.timestamp = hrt_absolute_time();
//         _trajectory_setpoint_pub.publish(sp_local);
//         ocm.position=true;
//         ocm.timestamp = hrt_absolute_time();
//         _offboard_control_mode_pub.publish(ocm);
//          _command.command =vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
//         _command.param1=1.0f;
//         _command.param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD;
//         _command.timestamp = hrt_absolute_time();
//         _vehicle_command_pub.publish(_command);

//                     if(_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
//                            {
                                 return true;
                           }
                         else
                                  {
                          return false;
                                 }
                        //   }
                        //         else
                        //  {
                        //  return false;
                        // }
}

bool control_instance::Control_fw_to_mc()
{
        _vehicle_status_sub.copy(&_status);
        _command.target_system = _status.system_id;
        _command.target_component = _status.component_id;
        _command.command =vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
        _command.param1=(float)(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
        _command.timestamp = hrt_absolute_time();
        _vehicle_command_pub.publish(_command);

                _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
        PX4_INFO("fw_to_mc");
        if(_vtol_vehicle_status.vehicle_vtol_state==vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC)
        {
                        return true;
        }
        else
        {
                         return false;
        }
}
