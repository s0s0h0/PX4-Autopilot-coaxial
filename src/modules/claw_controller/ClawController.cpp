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

#include "ClawController.hpp"

ModuleBase::Descriptor ClawController::desc{task_spawn, custom_command, print_usage};

ClawController::ClawController()
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool ClawController::init()
{
	ScheduleOnInterval(50_ms); // 20 Hz
	return true;
}

void ClawController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	// ── Parameter update ───────────────────────────────────────────────────
	if (_parameter_update_sub.updated()) {
		parameter_update_s dummy;
		_parameter_update_sub.copy(&dummy);
		updateParams();
	}

	// ── Land detection ─────────────────────────────────────────────────────
	vehicle_land_detected_s land_detected;

	if (_land_detected_sub.update(&land_detected)) {
		const bool now_landed = land_detected.landed;

		if (now_landed && !_landed) {
			_state_before_landing = _state;
			_state = ClawState::LANDING_SUPPORT;
			publish_gripper(gripper_s::COMMAND_GRAB, _param_land_pos.get());

		} else if (!now_landed && _landed) {
			_state = _state_before_landing;

			switch (_state) {
			case ClawState::GRABBED:
				publish_gripper(gripper_s::COMMAND_GRAB, _param_grab_pos.get());
				break;

			case ClawState::GRABBING:
				publish_gripper(gripper_s::COMMAND_GRAB, _param_fly_pos.get());
				break;

			case ClawState::RELEASING:
				publish_gripper(gripper_s::COMMAND_RELEASE, _param_fly_pos.get());
				break;

			default: // IDLE
				publish_gripper(gripper_s::COMMAND_RELEASE, _param_rel_pos.get());
				break;
			}
		}

		_landed = now_landed;
	}

	if (_landed) {
		return;
	}

	// ── RC input ───────────────────────────────────────────────────────────
	rc_channels_s rc;

	if (!_rc_channels_sub.update(&rc)) {
		return;
	}

	if (rc.signal_lost) {
		return;
	}

	// Read the channel assigned to AUX1 via RC_MAP_AUX1 in QGC
	const float aux1_val = rc.channels[(int)rc_channels_s::FUNCTION_AUX_1];

	const bool want_grab = (aux1_val >= _param_rc_thr.get());

	const hrt_abstime now         = hrt_absolute_time();
	const hrt_abstime elapsed_us  = now - _state_entry_time;
	const hrt_abstime grab_to_us  = (hrt_abstime)(_param_grab_timeout_s.get() * 1e6f);
	const hrt_abstime rel_to_us   = (hrt_abstime)(_param_rel_timeout_s.get() * 1e6f);

	// ── State machine ──────────────────────────────────────────────────────
	switch (_state) {

	case ClawState::IDLE:
		if (want_grab) {
			_state = ClawState::GRABBING;
			_state_entry_time = now;
			publish_gripper(gripper_s::COMMAND_GRAB, _param_fly_pos.get());
		}

		break;

	case ClawState::GRABBING:
		if (elapsed_us >= grab_to_us) {
			_state = ClawState::GRABBED;
			publish_gripper(gripper_s::COMMAND_GRAB, _param_grab_pos.get());
		}

		break;

	case ClawState::GRABBED:
		if (!want_grab) {
			_state = ClawState::RELEASING;
			_state_entry_time = now;
			publish_gripper(gripper_s::COMMAND_RELEASE, _param_fly_pos.get());
		}

		break;

	case ClawState::RELEASING:
		if (elapsed_us >= rel_to_us) {
			_state = ClawState::IDLE;
			publish_gripper(gripper_s::COMMAND_RELEASE, _param_rel_pos.get());
		}

		break;

	case ClawState::LANDING_SUPPORT:
		break;
	}
}

void ClawController::publish_gripper(int8_t command, float position)
{
	gripper_s msg{};
	msg.timestamp           = hrt_absolute_time();
	msg.command             = command;
	msg.normalized_position = position;
	_gripper_pub.publish(msg);
}

const char *ClawController::state_str() const
{
	switch (_state) {
	case ClawState::IDLE:            return "IDLE";
	case ClawState::GRABBING:        return "GRABBING";
	case ClawState::GRABBED:         return "GRABBED";
	case ClawState::RELEASING:       return "RELEASING";
	case ClawState::LANDING_SUPPORT: return "LANDING_SUPPORT";
	default:                         return "UNKNOWN";
	}
}

int ClawController::print_status()
{
	PX4_INFO("State      : %s", state_str());
	PX4_INFO("Landed     : %s", _landed ? "yes" : "no");
	PX4_INFO("RC input   : AUX1 (set via RC_MAP_AUX1)");
	PX4_INFO("RC thr     : %.2f", (double)_param_rc_thr.get());
	PX4_INFO("Grab pos   : %.2f", (double)_param_grab_pos.get());
	PX4_INFO("Release pos: %.2f", (double)_param_rel_pos.get());
	PX4_INFO("Fly pos    : %.2f", (double)_param_fly_pos.get());
	PX4_INFO("Land pos   : %.2f", (double)_param_land_pos.get());
	PX4_INFO("Grab TO    : %.2f s", (double)_param_grab_timeout_s.get());
	PX4_INFO("Release TO : %.2f s", (double)_param_rel_timeout_s.get());
	return 0;
}

int ClawController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RC-driven claw (mechanical gripper) controller for a servo on main8.

A single RC switch channel controls grab/release. The claw transitions through
states with configurable servo positions and timeouts. When the vehicle is
detected as landed, the claw is automatically held at the landing-support angle.

### Configuration
Set PWM_MAIN_FUNC8 = 430 (Gripper) to route this module's output to main8.

Parameters:
  CLAW_RC_CHAN  – RC channel (1-based) for the grab switch
  CLAW_RC_THR  – RC threshold for "grab" position
  CLAW_GRAB_TO – Time [s] before locking into GRABBED state (grab hold time)
  CLAW_REL_TO  – Time [s] before returning to IDLE after release
  CLAW_GRAB_POS– Servo position [-1,1] when grabbed (holding object)
  CLAW_REL_POS – Servo position [-1,1] when released / idle
  CLAW_FLY_POS – Servo position [-1,1] during grab/release transit in flight
  CLAW_LAND_POS– Servo position [-1,1] used as landing support leg
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("claw_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int ClawController::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command");
}

int ClawController::task_spawn(int argc, char *argv[])
{
	ClawController *instance = new ClawController();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int claw_controller_main(int argc, char *argv[])
{
	return ModuleBase::main(ClawController::desc, argc, argv);
}
