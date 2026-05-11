/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file ClawController.hpp
 *
 * RC-driven claw (mechanical gripper) controller.
 *
 * State machine:
 *   IDLE  --[grab switch]--> GRABBING --[CLAW_GRAB_TO timeout]--> GRABBED
 *   GRABBED --[release switch]--> RELEASING --[CLAW_REL_TO timeout]--> IDLE
 *   ANY STATE + landed --> LANDING_SUPPORT (resumes previous state on takeoff)
 *
 * Position parameters (all in normalised servo range [-1, 1]):
 *   CLAW_GRAB_POS  – claw position when grabbed
 *   CLAW_REL_POS   – claw position when released / idle
 *   CLAW_FLY_POS   – claw position during normal flight (GRABBING / RELEASING transit)
 *   CLAW_LAND_POS  – claw position used as landing support leg
 *
 * RC input: uses RC_MAP_AUX1 (QGC RC Setup page). The AUX1 mapped channel
 * value is read from rc_channels.function[FUNCTION_AUX_1].
 * Threshold: value >= CLAW_RC_THR -> grab, < CLAW_RC_THR -> release.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gripper.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

enum class ClawState : uint8_t {
	IDLE            = 0,
	GRABBING        = 1,
	GRABBED         = 2,
	RELEASING       = 3,
	LANDING_SUPPORT = 4,
};

class ClawController : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	ClawController();
	~ClawController() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:
	void Run() override;

	void publish_gripper(int8_t command, float position);
	const char *state_str() const;

	// ── uORB ───────────────────────────────────────────────────────────────
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<gripper_s> _gripper_pub{ORB_ID(gripper)};

	// ── Internal state ─────────────────────────────────────────────────────
	ClawState   _state{ClawState::IDLE};
	ClawState   _state_before_landing{ClawState::IDLE};
	hrt_abstime _state_entry_time{0};
	bool        _landed{false};

	// ── Parameters ─────────────────────────────────────────────────────────
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CLAW_RC_THR>)   _param_rc_thr,
		(ParamFloat<px4::params::CLAW_GRAB_TO>)  _param_grab_timeout_s,
		(ParamFloat<px4::params::CLAW_REL_TO>)   _param_rel_timeout_s,
		(ParamFloat<px4::params::CLAW_GRAB_POS>) _param_grab_pos,
		(ParamFloat<px4::params::CLAW_REL_POS>)  _param_rel_pos,
		(ParamFloat<px4::params::CLAW_FLY_POS>)  _param_fly_pos,
		(ParamFloat<px4::params::CLAW_LAND_POS>) _param_land_pos
	)
};
