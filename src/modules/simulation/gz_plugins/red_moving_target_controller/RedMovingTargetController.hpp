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

#pragma once

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

#include <random>

namespace custom
{
class RedMovingTargetController:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

private:
	double getSdfDouble(const std::shared_ptr<const sdf::Element> &sdf,
			    const char *name, double default_value) const;
	int getSdfInt(const std::shared_ptr<const sdf::Element> &sdf,
		      const char *name, int default_value) const;
	double clamp(double value, double lower, double upper) const;
	double wrapPi(double angle) const;
	bool nearBoundary() const;
	void planNextHeading(double sim_time);
	double randomUniform(double lower, double upper);

	gz::sim::Model _model{gz::sim::kNullEntity};

	double _center_x{0.};
	double _center_y{0.};
	double _z{2.};
	double _half_range{100.};
	double _boundary_margin{25.};
	double _speed{8.};
	double _turn_rate{GZ_DTOR(14.)};
	double _turn_accel{GZ_DTOR(18.)};
	double _heading_response{0.8};
	double _max_heading_change{GZ_DTOR(55.)};
	double _min_heading_change{GZ_DTOR(15.)};
	double _min_leg_time{6.};
	double _max_leg_time{12.};

	double _x{0.};
	double _y{0.};
	double _heading{0.};
	double _target_heading{0.};
	double _heading_rate{0.};
	double _next_turn_time{0.};
	double _last_sim_time{-1.};

	std::mt19937 _rng;
};
} // namespace custom
