#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
//#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include "control_instance.h"

//mc_control_instance* mc_control_instance::instance = nullptr;

class Control_Node : public ModuleBase<Control_Node>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Control_Node();
	~Control_Node() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;
	//mc_control_instance* instance = mc_control_instance::getInstance();
//mc_control_instance mc_ctl_instance;
bool takeoff();
bool control_node_init();
bool arm_offboard();
void start_control_node();
 vtol_vehicle_status_s _vtol_vehicle_status;
uint64_t init_gps_time;
enum state
{
INIT=0,
ARM_OFFBOARD,
TAKEOFF,
CONTROL,
LAND,
EMERGENCY
};
state STATE=INIT;

enum vtol_ctl_state
{
mc_takeoff=0,
mc_to_fw,
fw_mission,
fw_to_mc,
mc_land,
emerg
};
vtol_ctl_state VTOL_STATE=mc_takeoff;

    float begin_x;
    float begin_y;
    float begin_z;
vehicle_local_position_s _vehicle_local_position;
sensor_gps_s _sensor_gps;
int recived_point;

	// Publications
	// uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};


	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules Swarm when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};

	 uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)


	bool _armed{false};
};
