/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ExtPolicy.hpp"

#include <commander/px4_custom_mode.h>
#include <lib/mathlib/mathlib.h>

#include <inttypes.h>
#include <string.h>

using matrix::Vector2f;
using matrix::Vector3f;

static constexpr float ACC_XY_MAX = 6.f;   // m/s^2
static constexpr float ACC_Z_MAX = 6.f;    // m/s^2
static constexpr float JERK_MAX = 30.f;    // m/s^3
static constexpr hrt_abstime MODE_REQUEST_DELAY = 200_ms;  // stream setpoints this long before requesting Offboard
static constexpr hrt_abstime MODE_SWITCH_TIMEOUT = 2_s;

static const char *state_str(int s)
{
	static const char *names[] = {"idle", "engaging", "engaged", "disengaging"};
	return names[s];
}

ExtPolicy::ExtPolicy() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

ExtPolicy::~ExtPolicy()
{
	for (auto &slot : _slots) {
		ext_policy::unload_policy(slot);
	}

	perf_free(_step_perf);
}

bool ExtPolicy::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int ExtPolicy::load(const char *path)
{
	if (_pending_slot.load() >= 0) {
		PX4_ERR("previous policy not picked up yet, retry");
		return PX4_ERROR;
	}

	// load into the slot Run() is not using
	const int target = (_active_slot.load() == 0) ? 1 : 0;
	ext_policy::unload_policy(_slots[target]);

	char err[96];

	if (ext_policy::load_policy(path, _slots[target], err, sizeof(err)) != 0) {
		PX4_ERR("rejected %s: %s", path, err);
		return PX4_ERROR;
	}

	PX4_INFO("loaded '%s' from %s: %u bytes code, crc 0x%08" PRIx32 ", slot %d",
		 _slots[target].name, path, (unsigned)_slots[target].code_size, _slots[target].crc, target);
	_pending_slot.store(target);
	return PX4_OK;
}

void ExtPolicy::send_mode_command(uint8_t main_mode, uint8_t sub_mode)
{
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.f;  // custom mode
	cmd.param2 = main_mode;
	cmd.param3 = sub_mode;
	cmd.source_system = _system_id;
	cmd.target_system = _system_id;
	cmd.source_component = _component_id;
	cmd.target_component = _component_id;
	cmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(cmd);
}

void ExtPolicy::publish_setpoints(hrt_abstime now)
{
	offboard_control_mode_s ocm{};
	ocm.acceleration = true;
	ocm.timestamp = now;
	_offboard_mode_pub.publish(ocm);

	trajectory_setpoint_s sp{};
	sp.position[0] = sp.position[1] = sp.position[2] = NAN;
	sp.velocity[0] = sp.velocity[1] = sp.velocity[2] = NAN;
	_acc.copyTo(sp.acceleration);
	_jerk.copyTo(sp.jerk);
	sp.yaw = _yaw_hold;
	sp.yawspeed = NAN;
	sp.timestamp = now;
	_trajectory_setpoint_pub.publish(sp);
}

void ExtPolicy::stop(const char *reason)
{
	if (_state == State::Engaging || _state == State::Engaged) {
		send_mode_command(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
	}

	PX4_WARN("policy stopped: %s", reason);
	_state = State::Idle;
}

void ExtPolicy::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// swap in a newly loaded policy; the old slot is only released by the next load
	const int pending = _pending_slot.load();

	if (pending >= 0) {
		_active_slot.store(pending);
		_pending_slot.store(-1);
		PX4_INFO("policy '%s' is now active%s", _slots[pending].name, _state == State::Engaged ? " (hot swap)" : "");
	}

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
		_system_id = status.system_id;
		_component_id = status.component_id;
	}

	vehicle_local_position_s lpos;

	if (!_local_pos_sub.update(&lpos)) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((lpos.timestamp_sample - _last_sample) * 1e-6f, 0.001f, 0.05f);
	_last_sample = lpos.timestamp_sample;
	const bool in_offboard = (_nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	if (_disengage_request.load()) {
		_disengage_request.store(false);

		if (_state == State::Engaging || _state == State::Engaged) {
			send_mode_command(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
			_state = State::Disengaging;
			_state_since = now;
		}
	}

	if (_engage_request.load()) {
		_engage_request.store(false);

		if (_state != State::Idle) {
			PX4_WARN("already %s", state_str((int)_state));

		} else if (_active_slot.load() < 0) {
			PX4_ERR("no policy loaded");

		} else if (!(lpos.xy_valid && lpos.z_valid && lpos.v_xy_valid && lpos.v_z_valid)) {
			PX4_ERR("local position not valid");

		} else {
			// start the integrator from the current state so the hand-over is continuous
			_acc = Vector3f(lpos.ax, lpos.ay, lpos.az);
			_jerk.zero();
			_yaw_hold = lpos.heading;
			_state = State::Engaging;
			_state_since = now;
			PX4_INFO("engaging '%s'", _slots[_active_slot.load()].name);
		}
	}

	if (_state == State::Idle) {
		return;
	}

	// run the policy
	const ext_policy::PolicyImage &policy = _slots[_active_slot.load()];
	const float x[XPOL_NX] = {
		lpos.x, lpos.y, lpos.z, lpos.vx, lpos.vy, lpos.vz,
		_acc(0), _acc(1), _acc(2), _jerk(0), _jerk(1), _jerk(2)
	};
	float u[XPOL_NU] {};

	perf_begin(_step_perf);
	policy.step(policy.state, ext_policy::math_table(), x, u);
	perf_end(_step_perf);

	_snap = Vector3f(u[0], u[1], u[2]);

	if (!_snap.isAllFinite()) {
		stop("non-finite policy output");
		return;
	}

	// integrate snap -> jerk -> acceleration
	_jerk = matrix::constrain(_jerk + _snap * dt, -JERK_MAX, JERK_MAX);
	_acc += _jerk * dt;
	const float acc_xy = Vector2f(_acc(0), _acc(1)).norm();

	if (acc_xy > ACC_XY_MAX) {
		_acc(0) *= ACC_XY_MAX / acc_xy;
		_acc(1) *= ACC_XY_MAX / acc_xy;
	}

	_acc(2) = math::constrain(_acc(2), -ACC_Z_MAX, ACC_Z_MAX);

	publish_setpoints(now);

	switch (_state) {
	case State::Engaging:
		if (in_offboard) {
			_state = State::Engaged;
			PX4_INFO("engaged");

		} else if (now - _state_since > MODE_SWITCH_TIMEOUT) {
			stop("Offboard mode not accepted");

		} else if (now - _state_since > MODE_REQUEST_DELAY) {
			send_mode_command(PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0);
		}

		break;

	case State::Engaged:
		if (!in_offboard) {
			_state = State::Idle;
			PX4_WARN("left Offboard (nav_state %u), policy idle", _nav_state);
		}

		break;

	case State::Disengaging:
		if (!in_offboard) {
			_state = State::Idle;
			PX4_INFO("disengaged, PX4 in control");

		} else if (now - _state_since > MODE_SWITCH_TIMEOUT) {
			_state = State::Idle;
			PX4_WARN("disengage timed out, stopped streaming");
		}

		break;

	case State::Idle:
		break;
	}
}

int ExtPolicy::task_spawn(int argc, char *argv[])
{
	ExtPolicy *instance = new ExtPolicy();

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

int ExtPolicy::print_status()
{
	PX4_INFO("state: %s, nav_state: %u", state_str((int)_state), _nav_state);

	for (int i = 0; i < NUM_SLOTS; i++) {
		if (_slots[i].loaded()) {
			PX4_INFO("slot %d%s: '%s' %u bytes, crc 0x%08" PRIx32, i, i == _active_slot.load() ? " (active)" : "",
				 _slots[i].name, (unsigned)_slots[i].code_size, _slots[i].crc);
		}
	}

	PX4_INFO("acc [%.2f %.2f %.2f] jerk [%.2f %.2f %.2f] snap [%.2f %.2f %.2f]",
		 (double)_acc(0), (double)_acc(1), (double)_acc(2), (double)_jerk(0), (double)_jerk(1), (double)_jerk(2),
		 (double)_snap(0), (double)_snap(1), (double)_snap(2));
	perf_print_counter(_step_perf);
	return 0;
}

int ExtPolicy::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (argc == 2 && !strcmp(argv[0], "load")) {
		return get_instance()->load(argv[1]);
	}

	if (argc == 1 && !strcmp(argv[0], "on")) {
		get_instance()->request_engage();
		return PX4_OK;
	}

	if (argc == 1 && !strcmp(argv[0], "off")) {
		get_instance()->request_disengage();
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int ExtPolicy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Runs an uploaded control policy u = f(x) on the flat state x = [p, v, a, j] (NED), u = snap.
The policy image is verified (header, CRC-32) and hot-swappable in flight.
`on` switches to Offboard driven by the policy, `off` switches back to Hold (PX4 controllers).

### Example
$ ext_policy load policy/hold.xpol
$ ext_policy on
$ ext_policy load policy/circle.xpol
$ ext_policy off
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ext_policy", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("load", "Load and verify a policy image; becomes active on the next cycle");
	PRINT_MODULE_USAGE_ARG("<file>", "Policy image (.xpol)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("on", "Hand control to the active policy (Offboard)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("off", "Hand control back to PX4 (Hold)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ext_policy_main(int argc, char *argv[])
{
	return ExtPolicy::main(argc, argv);
}
