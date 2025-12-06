#include "control_node.h"
// mc_control_instance* mc_control_instance::instance = nullptr;
Control_Node::Control_Node() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

Control_Node::~Control_Node()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Control_Node::init()
{
	// execute Run() on every sensor_accel publication
	//if (!_sensor_accel_sub.registerCallback()) {
	//	PX4_ERR("callback registration failed");
	//	return false;
	//}

	// alternatively, Run on fixed interval
	 ScheduleOnInterval(200000_us); // 2000 us interval, 200 Hz rate

	return true;
}
bool Control_Node::takeoff()
{
control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-5);
return false;
}
bool Control_Node::arm_offboard()
{
return false;
}
bool Control_Node::control_node_init()
{
return true;
}


void Control_Node::start_control_node()
{

	switch(VTOL_STATE)
	{
	case vtol_ctl_state::mc_takeoff:
	PX4_INFO("mc_takeoff");
	if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50))
	{
		VTOL_STATE=vtol_ctl_state::mc_to_fw;
	}
	break;

	case vtol_ctl_state::mc_to_fw:
	PX4_INFO("mc_to_fw");
	if(control_instance::getInstance()->Control_mc_to_fw())
	{
	VTOL_STATE=vtol_ctl_state::fw_mission;
	}
	//control_instance::getInstance()->Control_posxyz(begin_x+500,begin_y,begin_z-50);
	break;

	case vtol_ctl_state::fw_mission:
	PX4_INFO("fw_mission");
	if(control_instance::getInstance()->Control_posxyz(begin_x+200,begin_y,begin_z-50))
	{
	VTOL_STATE=vtol_ctl_state::fw_to_mc;
	}
	break;

	case vtol_ctl_state::fw_to_mc:
	PX4_INFO("fw_to_mc");
	if(control_instance::getInstance()->Control_fw_to_mc())
	{
	VTOL_STATE=vtol_ctl_state::mc_land;
	}
	break;

	case vtol_ctl_state::mc_land:
	PX4_INFO("mc_land");
		if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-50))
	{
		control_instance::getInstance()->Change_land();
	}
	break;

	default:
	break;
	}
 	// _vtol_vehicle_status_sub.copy(&_vtol_vehicle_status);
 	// PX4_INFO("_vtol_vehicle_status=%d\n",_vtol_vehicle_status.vehicle_vtol_state);
}


void Control_Node::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}
	switch(STATE)
	{
	case state::INIT:
	recived_point=0;
	if(control_node_init())
	{
	STATE=state::ARM_OFFBOARD;
	}
	// PX4_INFO("INIT");
	break;
	case state::ARM_OFFBOARD:
	//PX4_INFO("ARM_OFFBOARD");
	if(control_instance::getInstance()->Change_offborad()&&control_instance::getInstance()->Arm_vehicle())
	{
	STATE=state::TAKEOFF;
	}
	break;
	case state::TAKEOFF:
	//PX4_INFO("TAKEOFF");
	if(control_instance::getInstance()->Control_posxyz(begin_x,begin_y,begin_z-5))
	{
		_sensor_gps_sub.copy(&_sensor_gps);
init_gps_time=_sensor_gps.time_utc_usec;
//PX4_INFO("_sensor_gps.time_utc_usec=%lld",_sensor_gps.time_utc_usec);
	STATE=state::CONTROL;
	}
	break;
	case state::CONTROL:
	//PX4_INFO("SWARM");
	//mc_control_instance::getInstance()->Control_lat_lon_alt(x,y,z);
	//start_control_node();
	break;
	case state::LAND:
	//PX4_INFO("LAND");
	control_instance::getInstance()->Change_land();
	break;
	case state::EMERGENCY:
	//PX4_INFO("EMERGENCY");
	break;
	default:
	break;
	}


	perf_end(_loop_perf);
}

int Control_Node::task_spawn(int argc, char *argv[])
{
	Control_Node *instance = new Control_Node();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Control_Node::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int Control_Node::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Control_Node::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int control_node_main(int argc, char *argv[])
{
	return Control_Node::main(argc, argv);
}
