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

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "PolicyImage.hpp"

using namespace time_literals;

/**
 * Runs an uploaded policy u = f(x) on the flat state and switches between it and PX4's own controllers.
 *
 * The policy outputs snap; this module integrates jerk and acceleration internally and sends the
 * acceleration as an offboard setpoint, so mc_pos_control maps it to attitude and thrust.
 * Engaging switches the vehicle to Offboard, disengaging switches it to Hold (PX4 position control).
 */
class ExtPolicy : public ModuleBase<ExtPolicy>, public px4::ScheduledWorkItem
{
public:
	ExtPolicy();
	~ExtPolicy() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	int load(const char *path);
	void request_engage() { _engage_request.store(true); }
	void request_disengage() { _disengage_request.store(true); }

	void send_mode_command(uint8_t main_mode, uint8_t sub_mode);
	void publish_setpoints(hrt_abstime now);
	void stop(const char *reason);

	enum class State { Idle, Engaging, Engaged, Disengaging };

	static constexpr int NUM_SLOTS = 2;
	ext_policy::PolicyImage _slots[NUM_SLOTS] {};
	px4::atomic<int> _active_slot{-1};   // slot used by Run()
	px4::atomic<int> _pending_slot{-1};  // loaded and verified, picked up at the start of the next Run()
	px4::atomic<bool> _engage_request{false};
	px4::atomic<bool> _disengage_request{false};

	State _state{State::Idle};
	hrt_abstime _state_since{0};
	hrt_abstime _last_sample{0};

	matrix::Vector3f _acc{};   // integrated flat state
	matrix::Vector3f _jerk{};
	matrix::Vector3f _snap{};
	float _yaw_hold{0.f};
	uint8_t _nav_state{0};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<offboard_control_mode_s> _offboard_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	uint8_t _system_id{1};
	uint8_t _component_id{1};

	perf_counter_t _step_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": policy step")};
};
