/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "mavlink_attitude_bootstrap.h"
#include "mavlink_main.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <cmath>

using attitude_bootstrap::Packet;
using Status = attitude_bootstrap_status_s;

int32_t MavlinkAttitudeBootstrap::parameter(const char *name, int32_t fallback) const
{
	const param_t handle = param_find(name);

	if (handle != PARAM_INVALID) {
		param_get(handle, &fallback);
	}

	return fallback;
}

void MavlinkAttitudeBootstrap::send(const Packet &packet, uint8_t target_system, uint8_t target_component)
{
	mavlink_tunnel_t tunnel{};
	tunnel.target_system = target_system;
	tunnel.target_component = target_component;
	tunnel.payload_type = attitude_bootstrap::PAYLOAD_TYPE;
	tunnel.payload_length = attitude_bootstrap::LENGTH;
	attitude_bootstrap::encode(packet, tunnel.payload);
	_mavlink.lock_send();
	mavlink_msg_tunnel_send_struct(_mavlink.get_channel(), &tunnel);
	_mavlink.unlock_send();
}

void MavlinkAttitudeBootstrap::publish_status(const Packet &packet, uint8_t source_system, uint8_t source_component,
		uint8_t target_system, uint8_t target_component, uint8_t result, bool follower_result)
{
	Status status{};
	status.timestamp = hrt_absolute_time();
	status.timestamp_sample = packet.timestamp_sample;
	status.request_id = packet.request_id;
	status.source_timestamp = packet.source_timestamp;
	status.timestamp_uncertainty_us = packet.timestamp_uncertainty_us;
	memcpy(status.q, packet.q, sizeof(status.q));
	memcpy(status.variance, packet.variance, sizeof(status.variance));

	if (follower_result) {
		// Only the master logging an actual follower result may copy application
		// fields from the wire. A master cannot assert local EKF application.
		status.timestamp_applied = packet.timestamp_applied;
		memcpy(status.q_applied, packet.q_applied, sizeof(status.q_applied));
		memcpy(status.variance_applied, packet.variance_applied, sizeof(status.variance_applied));
	}

	status.frame = packet.frame;
	status.child_frame = packet.child_frame;
	status.source_system = source_system;
	status.source_component = source_component;
	status.target_system = target_system;
	status.target_component = target_component;
	status.estimator_instance = packet.estimator_instance;
	status.result = result;
	_status_pub.publish(status);
}

void MavlinkAttitudeBootstrap::serve(Packet &packet, const mavlink_message_t &msg, uint64_t now)
{
	Served *served = nullptr;

	for (Served &previous : _served) {
		if (previous.system == msg.sysid && previous.component == msg.compid) {
			served = &previous;
			break;
		}
	}

	if (served && now - served->time < 20000) {
		packet.kind = attitude_bootstrap::RESULT;
		packet.result = Status::BUSY;
		publish_status(packet, _mavlink.get_system_id(), _mavlink.get_component_id(), msg.sysid, msg.compid, packet.result);
		send(packet, msg.sysid, msg.compid);
		return;
	}

	if (!served) {
		served = &_served[_served_next++ % 16];
	}

	const uint64_t request_id = packet.request_id;
	const uint64_t t1 = packet.t1;
	packet = Packet{};
	packet.request_id = request_id;
	packet.t1 = t1;
	packet.t2 = now;
	packet.kind = attitude_bootstrap::RESULT;
	packet.result = Status::DISABLED;

	if (parameter("MAV_AB_EN", 0) == 1) {
		estimator_selector_status_s selector{};

		if (_selector_sub.copy(&selector)) {
			_estimator_status_sub.ChangeInstance(selector.primary_instance);

			if (_odometry_sub.orb_id() != ORB_ID::estimator_odometry) {
				_odometry_sub = uORB::Subscription{ORB_ID(estimator_odometry), selector.primary_instance};
				_local_position_sub = uORB::Subscription{ORB_ID(estimator_local_position), selector.primary_instance};

			} else {
				_odometry_sub.ChangeInstance(selector.primary_instance);
				_local_position_sub.ChangeInstance(selector.primary_instance);
			}

			packet.estimator_instance = selector.primary_instance;
		}

		vehicle_odometry_s odometry{};
		vehicle_local_position_s local_position{};
		estimator_status_s estimator{};
		packet.result = Status::NO_ATTITUDE;

		if (_odometry_sub.copy(&odometry) && _local_position_sub.copy(&local_position)
		    && _estimator_status_sub.copy(&estimator)) {
			const uint64_t required_flags = (uint64_t(1) << estimator_status_s::CS_TILT_ALIGN)
							| (uint64_t(1) << estimator_status_s::CS_YAW_ALIGN);
			const bool fresh = odometry.timestamp_sample > 0 && odometry.timestamp_sample <= now
					   && now - odometry.timestamp_sample < 100000
					   && local_position.timestamp <= now && now - local_position.timestamp < 100000
					   && estimator.timestamp <= now && now - estimator.timestamp < 100000
					   && std::abs(int64_t(local_position.timestamp) - int64_t(odometry.timestamp)) < 20000;

			packet.source_timestamp = odometry.timestamp_sample;
			memcpy(packet.q, odometry.q, sizeof(packet.q));
			// Each horizontal NED variance is bounded by the horizontal sum.
			// The factor three bounds unknown cross-axis correlations: 3*diag(P)
			// dominates any positive semidefinite 3x3 P with those diagonal entries.
			packet.variance[0] = 3.f * local_position.tilt_var;
			packet.variance[1] = 3.f * local_position.tilt_var;
			packet.variance[2] = 3.f * local_position.heading_var;
			bool finite = true;
			float norm_squared = 0.f;

			for (float v : packet.q) {
				finite = finite && PX4_ISFINITE(v);
				norm_squared += v * v;
			}

			for (float v : packet.variance) {
				finite = finite && PX4_ISFINITE(v) && v > 0.f;
			}

			if (fresh && finite && fabsf(norm_squared - 1.f) < 0.02f
			    && odometry.pose_frame == vehicle_odometry_s::POSE_FRAME_NED
			    && (estimator.control_mode_flags & required_flags) == required_flags
			    && estimator.filter_fault_flags == 0
			    && !estimator.pre_flt_fail_innov_heading && !estimator.pre_flt_fail_mag_field_disturbed
			    && (!PX4_ISFINITE(estimator.hdg_test_ratio) || estimator.hdg_test_ratio <= 1.f)) {
				packet.kind = attitude_bootstrap::SAMPLE;
				packet.result = Status::WAITING;
			}
		}
	}

	packet.t3 = hrt_absolute_time();
	*served = {packet.request_id, now, msg.sysid, msg.compid, false, 255};
	// Source logs exactly what was sent. WAITING is never reported as success.
	publish_status(packet, _mavlink.get_system_id(), _mavlink.get_component_id(), msg.sysid, msg.compid, packet.result);
	send(packet, msg.sysid, msg.compid);
}

void MavlinkAttitudeBootstrap::handle(const mavlink_message_t &msg, const mavlink_tunnel_t &tunnel)
{
	const uint64_t now = hrt_absolute_time();
	Packet packet;
	const bool valid = attitude_bootstrap::decode(tunnel.payload, tunnel.payload_length, packet);

	// This private service requires exact addressing; never accept a broadcast seed.
	if (tunnel.target_system != _mavlink.get_system_id() || tunnel.target_component != _mavlink.get_component_id()) {
		publish_status(packet, msg.sysid, msg.compid, tunnel.target_system, tunnel.target_component, Status::WRONG_TARGET);
		return;
	}

	if (!valid || !packet.request_id || !packet.t1) {
		publish_status(packet, msg.sysid, msg.compid, tunnel.target_system, tunnel.target_component, Status::PROTOCOL_ERROR);

		// A malformed result must not provoke a result-of-result exchange.
		if (tunnel.payload_length >= 6 && tunnel.payload[5] < attitude_bootstrap::RESULT) {
			packet.kind = attitude_bootstrap::RESULT;
			packet.result = Status::PROTOCOL_ERROR;
			send(packet, msg.sysid, msg.compid);
		}

		return;
	}

	if (packet.kind == attitude_bootstrap::REQUEST) {
		serve(packet, msg, now);
		return;
	}

	if (packet.kind == attitude_bootstrap::RESULT && parameter("MAV_AB_EN", 0) == 1) {
		for (Served &served : _served) {
			if (served.request_id == packet.request_id && served.system == msg.sysid && served.component == msg.compid) {
				if (packet.result > Status::WRONG_TARGET) {
					return;
				}

				if (!served.final_seen || served.last_result != packet.result) {
					publish_status(packet, _mavlink.get_system_id(), _mavlink.get_component_id(), msg.sysid, msg.compid, packet.result, true);
					served.final_seen = packet.result != Status::WAITING;
					served.last_result = packet.result;
				}

				packet.kind = attitude_bootstrap::RESULT_ACK;
				send(packet, msg.sysid, msg.compid);
				return;
			}
		}
	}

	if (packet.kind == attitude_bootstrap::RESULT_ACK) {
		if (_awaiting_ack && packet.request_id == _final_result.request_id
		    && msg.sysid == _result_target_system && msg.compid == _result_target_component) {
			_awaiting_ack = false;
		}

		return;
	}

	uint8_t reject = Status::WAITING;

	if (msg.sysid != parameter("EKF2_AB_SYS", 1) || msg.compid != parameter("EKF2_AB_COMP", 1)) {
		reject = Status::WRONG_SOURCE;

	} else if (_done) {
		reject = Status::ALREADY_INITIALIZED;

	} else if (parameter("EKF2_AB_MODE", 0) != 1) {
		reject = Status::DISABLED;

	} else if (!_pending || packet.request_id != _request_id || packet.t1 != _request_time) {
		reject = Status::STALE;

	} else if (packet.kind == attitude_bootstrap::RESULT) {
		_pending = false;
		// The configured source explicitly refused the request (e.g. no aligned EKF).
		const uint8_t result = attitude_bootstrap::valid_source_refusal(packet.result) ? packet.result : Status::PROTOCOL_ERROR;
		publish_status(packet, msg.sysid, msg.compid, tunnel.target_system, tunnel.target_component, result);
		return;

	} else if (!attitude_bootstrap::map_time(packet, now)) {
		reject = Status::TIME_UNCERTAIN;

	} else {
		_pending = false; // consume the challenge before handing it to EKF2
		attitude_bootstrap_s seed{};
		seed.timestamp = now;
		seed.timestamp_sample = packet.timestamp_sample;
		seed.request_id = packet.request_id;
		seed.source_timestamp = packet.source_timestamp;
		seed.timestamp_uncertainty_us = packet.timestamp_uncertainty_us;
		memcpy(seed.q, packet.q, sizeof(seed.q));
		memcpy(seed.variance, packet.variance, sizeof(seed.variance));
		seed.frame = packet.frame;
		seed.child_frame = packet.child_frame;
		seed.source_system = msg.sysid;
		seed.source_component = msg.compid;
		seed.target_system = tunnel.target_system;
		seed.target_component = tunnel.target_component;
		_seed_pub.publish(seed);
		return;
	}

	const bool refused_sample = packet.kind == attitude_bootstrap::SAMPLE;
	packet.kind = attitude_bootstrap::RESULT;
	packet.result = reject;

	if (packet.request_id == _request_id && msg.sysid == parameter("EKF2_AB_SYS", 1)
	    && msg.compid == parameter("EKF2_AB_COMP", 1)) {
		_pending = false;
	}

	publish_status(packet, msg.sysid, msg.compid, tunnel.target_system, tunnel.target_component, reject);

	if (refused_sample && packet.request_id != _request_id) {
		// There is no locally originated matching challenge for update() to route.
		// Still explicitly refuse an unsolicited/replayed packet on its input link.
		send(packet, msg.sysid, msg.compid);
	}

	// update() sends results for locally originated challenges on their input link.
}

void MavlinkAttitudeBootstrap::update(uint64_t now)
{
	Status status{};

	while (_status_sub.update(&status)) {
		if (status.target_system != _mavlink.get_system_id() || status.target_component != _mavlink.get_component_id()) {
			continue;
		}

		if (status.result == Status::ACCEPTED && status.timestamp_applied > 0
		    && status.source_system == parameter("EKF2_AB_SYS", 1)
		    && status.source_component == parameter("EKF2_AB_COMP", 1)) {
			_done = true;
			_pending = false;
		}

		// Only the link which originated the challenge emits the result.
		if (status.request_id == 0 || status.request_id != _request_id || status.source_system == _mavlink.get_system_id()) {
			continue;
		}

		Packet packet;
		packet.kind = attitude_bootstrap::RESULT;
		packet.result = status.result;
		packet.request_id = status.request_id;
		packet.t1 = _request_time;
		packet.source_timestamp = status.source_timestamp;
		packet.timestamp_sample = status.timestamp_sample;
		packet.timestamp_applied = status.timestamp_applied;
		packet.timestamp_uncertainty_us = status.timestamp_uncertainty_us;
		packet.frame = status.frame;
		packet.child_frame = status.child_frame;
		packet.estimator_instance = status.estimator_instance;
		memcpy(packet.q, status.q, sizeof(packet.q));
		memcpy(packet.q_applied, status.q_applied, sizeof(packet.q_applied));
		memcpy(packet.variance, status.variance, sizeof(packet.variance));
		memcpy(packet.variance_applied, status.variance_applied, sizeof(packet.variance_applied));
		send(packet, status.source_system, status.source_component);

		if (status.result == Status::ACCEPTED && status.timestamp_applied > 0) {
			_final_result = packet;
			_result_target_system = status.source_system;
			_result_target_component = status.source_component;
			_last_result_send = now;
			_result_deadline = now + 5000000;
			_awaiting_ack = true;
		}
	}

	// Bounded result retransmission never requests or re-applies an attitude.
	if (_awaiting_ack && now < _result_deadline && now - _last_result_send >= 250000) {
		send(_final_result, _result_target_system, _result_target_component);
		_last_result_send = now;
	}

	if (_pending && now - _request_time > 100000) {
		_pending = false;
		Packet timeout;
		timeout.request_id = _request_id;
		timeout.t1 = _request_time;
		publish_status(timeout, parameter("EKF2_AB_SYS", 1), parameter("EKF2_AB_COMP", 1),
			       _mavlink.get_system_id(), _mavlink.get_component_id(), Status::TIME_UNCERTAIN);
		PX4_WARN("attitude bootstrap: source reply timed out (100 ms)");
	}

	actuator_armed_s armed{};
	_armed_sub.copy(&armed);

	if (_done || armed.armed || parameter("EKF2_AB_MODE", 0) != 1
	    || parameter("MAV_AB_LINK", 0) != _mavlink.get_instance_id()
	    || now - _last_request < 1000000) {
		return;
	}

	Packet request;
	request.t1 = request.request_id = now;
	_request_id = request.request_id;
	_request_time = now;
	_last_request = now;
	_pending = true;
	send(request, parameter("EKF2_AB_SYS", 1), parameter("EKF2_AB_COMP", 1));
}
