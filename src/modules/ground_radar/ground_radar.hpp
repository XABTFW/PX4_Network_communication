#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/radar_target.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

class GroundRadar : public ModuleBase<GroundRadar>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GroundRadar();
	~GroundRadar() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Publication<radar_target_s> _radar_target_pub{ORB_ID(radar_target)};

	matrix::Vector3f _last_position{0.f, 0.f, 0.f};
	matrix::Vector3f _current_velocity{0.f, 0.f, 0.f};
	hrt_abstime _last_update_time{0};

	bool _tracking_enabled{false};
	uint8_t _target_id{1};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GR_UPDATE_RATE>) _param_update_rate
	)
};
