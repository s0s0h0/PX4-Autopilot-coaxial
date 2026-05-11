/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "FunctionProviderBase.hpp"

#include <mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/topics/gripper.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <drivers/drv_hrt.h>

/**
 * @brief Function: Gripper output driver
 *
 * Priority (highest to lowest):
 *   1. manual_control_setpoint.aux1 is valid (RC signal present) -> direct analog passthrough,
 *      low-pass filtered at 5 Hz to smooth 50/100 Hz RC step artifacts
 *   2. gripper.normalized_position is finite -> analog command from software
 *   3. gripper.command GRAB/RELEASE          -> +1.0 / -1.0 (payload_deliverer compatibility)
 */
class FunctionGripper : public FunctionProviderBase
{
public:
	FunctionGripper()
	{
		_filter.setCutoffFreq(5.f);
		_filter.reset(-1.f);
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionGripper(); }

	void update() override
	{
		const hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain((_last_update_us > 0) ? (now - _last_update_us) * 1e-6f : 0.004f,
						 0.001f, 0.1f);
		_last_update_us = now;

		// Try RC aux1 passthrough first (smooth analog from RC transmitter)
		manual_control_setpoint_s mcs;

		if (_mcs_sub.update(&mcs) && mcs.valid && PX4_ISFINITE(mcs.aux1)) {
			_data = _filter.update(math::constrain(mcs.aux1, -1.f, 1.f), dt);
			return;
		}

		// Fall back to gripper topic (software commands from payload_deliverer etc.)
		gripper_s gripper;

		if (_gripper_sub.update(&gripper)) {
			float target;

			if (PX4_ISFINITE(gripper.normalized_position)) {
				target = math::constrain(gripper.normalized_position, -1.f, 1.f);

			} else if (gripper.command == gripper_s::COMMAND_GRAB) {
				target = 1.f;

			} else {
				target = -1.f;
			}

			_data = _filter.update(target, dt);
		}
	}

	float value(OutputFunction func) override { return _data; }

private:
	uORB::Subscription _mcs_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _gripper_sub{ORB_ID(gripper)};
	AlphaFilter<float> _filter;
	hrt_abstime _last_update_us{0};
	float _data{-1.f};
};
