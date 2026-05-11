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
#include <uORB/topics/gripper.h>

/**
 * @brief Function: Gripper output driver
 *
 * Reads the gripper uORB topic and maps it to a normalised servo value [-1, 1]:
 *   - If normalized_position is finite, use it directly (set by claw_controller)
 *   - COMMAND_GRAB    -> +1.0  (fallback for payload_deliverer compatibility)
 *   - COMMAND_RELEASE -> -1.0  (fallback)
 */
class FunctionGripper : public FunctionProviderBase
{
public:
	FunctionGripper() = default;
	static FunctionProviderBase *allocate(const Context &context) { return new FunctionGripper(); }

	void update() override
	{
		gripper_s gripper;

		if (_gripper_sub.update(&gripper)) {
			if (PX4_ISFINITE(gripper.normalized_position)) {
				_data = math::constrain(gripper.normalized_position, -1.f, 1.f);

			} else if (gripper.command == gripper_s::COMMAND_GRAB) {
				_data = 1.f;

			} else if (gripper.command == gripper_s::COMMAND_RELEASE) {
				_data = -1.f;
			}
		}
	}

	float value(OutputFunction func) override { return _data; }

private:
	uORB::Subscription _gripper_sub{ORB_ID(gripper)};
	float _data{-1.f};
};
