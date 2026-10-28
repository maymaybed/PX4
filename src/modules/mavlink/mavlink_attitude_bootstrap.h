/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "mavlink_bridge_header.h"
#include "mavlink_attitude_bootstrap_protocol.h"
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/attitude_bootstrap.h>
#include <uORB/topics/attitude_bootstrap_status.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_armed.h>

class Mavlink;

// Owned and called exclusively by one MAVLink receiver thread.
class MavlinkAttitudeBootstrap
{
public:
	explicit MavlinkAttitudeBootstrap(Mavlink &mavlink) : _mavlink(mavlink) {}

	void handle(const mavlink_message_t &msg, const mavlink_tunnel_t &tunnel);
	void update(uint64_t now);

private:
	int32_t parameter(const char *name, int32_t fallback) const;
	void send(const attitude_bootstrap::Packet &packet, uint8_t target_system, uint8_t target_component);
	void publish_status(const attitude_bootstrap::Packet &packet, uint8_t source_system, uint8_t source_component,
			    uint8_t target_system, uint8_t target_component, uint8_t result, bool follower_result = false);
	void serve(attitude_bootstrap::Packet &packet, const mavlink_message_t &msg, uint64_t now);

	Mavlink &_mavlink;
	uORB::Publication<attitude_bootstrap_s> _seed_pub{ORB_ID(attitude_bootstrap)};
	uORB::Publication<attitude_bootstrap_status_s> _status_pub{ORB_ID(attitude_bootstrap_status)};
	uORB::Subscription _status_sub{ORB_ID(attitude_bootstrap_status)};
	uORB::Subscription _selector_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uint64_t _request_id{0};
	uint64_t _request_time{0};
	uint64_t _last_request{0};
	bool _pending{false};
	bool _done{false};
	bool _awaiting_ack{false};
	attitude_bootstrap::Packet _final_result{};
	uint64_t _last_result_send{0};
	uint64_t _result_deadline{0};
	uint8_t _result_target_system{0};
	uint8_t _result_target_component{0};
	struct Served {
		uint64_t request_id{0};
		uint64_t time{0};
		uint8_t system{0};
		uint8_t component{0};
		bool final_seen{false};
		uint8_t last_result{255};
	};

	Served _served[16] {};
	unsigned _served_next{0};
};
