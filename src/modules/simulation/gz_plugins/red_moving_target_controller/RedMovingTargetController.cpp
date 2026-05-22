/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "RedMovingTargetController.hpp"

#include <gz/plugin/Register.hh>

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace custom;

GZ_ADD_PLUGIN(
	RedMovingTargetController,
	gz::sim::System,
	RedMovingTargetController::ISystemConfigure,
	RedMovingTargetController::ISystemPreUpdate
)

void RedMovingTargetController::Configure(const gz::sim::Entity &entity,
		const std::shared_ptr<const sdf::Element> &sdf,
		gz::sim::EntityComponentManager &ecm,
		gz::sim::EventManager &eventMgr)
{
	_model = gz::sim::Model(entity);

	_center_x = getSdfDouble(sdf, "center_x", 0.);
	_center_y = getSdfDouble(sdf, "center_y", 0.);
	_z = getSdfDouble(sdf, "z", 2.);
	_half_range = std::max(getSdfDouble(sdf, "range", 200.) * 0.5, 1.);
	_boundary_margin = clamp(getSdfDouble(sdf, "boundary_margin", 25.), 1., _half_range * 0.45);
	_speed = std::max(getSdfDouble(sdf, "speed", 8.), 0.1);
	_turn_rate = GZ_DTOR(std::max(getSdfDouble(sdf, "turn_rate", 14.), 1.));
	_turn_accel = GZ_DTOR(std::max(getSdfDouble(sdf, "turn_accel", 18.), 1.));
	_heading_response = std::max(getSdfDouble(sdf, "heading_response", 0.8), 0.1);

	const double max_heading_change_deg = clamp(getSdfDouble(sdf, "max_heading_change", 55.), 1., 89.);
	_max_heading_change = GZ_DTOR(max_heading_change_deg);
	_min_heading_change = GZ_DTOR(clamp(getSdfDouble(sdf, "min_heading_change", 15.), 0., max_heading_change_deg));
	_min_leg_time = std::max(getSdfDouble(sdf, "min_leg_time", 6.), 0.5);
	_max_leg_time = std::max(getSdfDouble(sdf, "max_leg_time", 12.), _min_leg_time);

	const int seed = getSdfInt(sdf, "seed", 0);
	_rng.seed(seed > 0 ? seed : std::random_device{}());

	_x = _center_x;
	_y = _center_y;
	_heading = randomUniform(-GZ_PI, GZ_PI);
	_target_heading = _heading;
	_next_turn_time = 0.;

	_model.SetWorldPoseCmd(ecm, gz::math::Pose3d(_x, _y, _z, 0., 0., _heading));
}

void RedMovingTargetController::PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &ecm)
{
	if (_info.paused) {
		return;
	}

	const double sim_time = std::chrono::duration<double>(_info.simTime).count();

	if (_last_sim_time < 0.) {
		_last_sim_time = sim_time;
		return;
	}

	const double dt = clamp(sim_time - _last_sim_time, 0., 0.05);
	_last_sim_time = sim_time;

	if (dt <= 0.) {
		return;
	}

	if (sim_time >= _next_turn_time || nearBoundary()) {
		planNextHeading(sim_time);
	}

	const double heading_error = wrapPi(_target_heading - _heading);
	const double desired_rate = clamp(heading_error * _heading_response, -_turn_rate, _turn_rate);
	const double rate_error = desired_rate - _heading_rate;
	const double rate_step = _turn_accel * dt;
	_heading_rate += clamp(rate_error, -rate_step, rate_step);
	_heading = wrapPi(_heading + _heading_rate * dt);

	_x += std::cos(_heading) * _speed * dt;
	_y += std::sin(_heading) * _speed * dt;
	_x = clamp(_x, _center_x - _half_range, _center_x + _half_range);
	_y = clamp(_y, _center_y - _half_range, _center_y + _half_range);

	_model.SetWorldPoseCmd(ecm, gz::math::Pose3d(_x, _y, _z, 0., 0., _heading));
}

double RedMovingTargetController::getSdfDouble(const std::shared_ptr<const sdf::Element> &sdf,
		const char *name, double default_value) const
{
	return sdf->HasElement(name) ? sdf->Get<double>(name) : default_value;
}

int RedMovingTargetController::getSdfInt(const std::shared_ptr<const sdf::Element> &sdf,
		const char *name, int default_value) const
{
	return sdf->HasElement(name) ? sdf->Get<int>(name) : default_value;
}

double RedMovingTargetController::clamp(double value, double lower, double upper) const
{
	return std::max(lower, std::min(upper, value));
}

double RedMovingTargetController::wrapPi(double angle) const
{
	while (angle > GZ_PI) {
		angle -= 2. * GZ_PI;
	}

	while (angle < -GZ_PI) {
		angle += 2. * GZ_PI;
	}

	return angle;
}

bool RedMovingTargetController::nearBoundary() const
{
	const double x_min = _center_x - _half_range + _boundary_margin;
	const double x_max = _center_x + _half_range - _boundary_margin;
	const double y_min = _center_y - _half_range + _boundary_margin;
	const double y_max = _center_y + _half_range - _boundary_margin;
	return _x < x_min || _x > x_max || _y < y_min || _y > y_max;
}

void RedMovingTargetController::planNextHeading(double sim_time)
{
	double delta = 0.;

	if (nearBoundary()) {
		const double inward = std::atan2(_center_y - _y, _center_x - _x);
		const double desired = wrapPi(inward + randomUniform(-_max_heading_change, _max_heading_change));
		delta = clamp(wrapPi(desired - _heading), -_max_heading_change, _max_heading_change);

	} else {
		const double sign = randomUniform(0., 1.) < 0.5 ? -1. : 1.;
		delta = sign * randomUniform(_min_heading_change, _max_heading_change);
	}

	_target_heading = wrapPi(_heading + delta);
	_next_turn_time = sim_time + randomUniform(_min_leg_time, _max_leg_time);
}

double RedMovingTargetController::randomUniform(double lower, double upper)
{
	std::uniform_real_distribution<double> distribution(lower, upper);
	return distribution(_rng);
}
