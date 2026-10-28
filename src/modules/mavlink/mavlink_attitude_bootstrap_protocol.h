/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <stdint.h>
#include <string.h>

// Private, versioned payload inside standard MAVLink TUNNEL. Never serialize a
// native struct: padding, alignment and endianness differ between flight boards.
namespace attitude_bootstrap
{
static constexpr uint16_t PAYLOAD_TYPE = 32768;
static constexpr uint8_t LENGTH = 120;
static constexpr uint32_t MAGIC = 0x31424150; // "PAB1"
enum Kind : uint8_t { REQUEST = 0, SAMPLE = 1, RESULT = 2, RESULT_ACK = 3 };

// Source service refusals are transport results, never an assertion that the
// follower has applied an attitude. Values match AttitudeBootstrapStatus.msg.
inline bool valid_source_refusal(uint8_t result)
{
	return result == 1 || result == 11 || result == 13 || result == 16 || result == 17;
}

inline uint64_t read_uint(const uint8_t *p, unsigned n)
{
	uint64_t value = 0;

	for (unsigned i = 0; i < n; ++i) {
		value |= uint64_t(p[i]) << (8 * i);
	}

	return value;
}

inline void write_uint(uint8_t *p, uint64_t value, unsigned n)
{
	for (unsigned i = 0; i < n; ++i) {
		p[i] = value >> (8 * i);
	}
}

inline float read_float(const uint8_t *p)
{
	const uint32_t bits = read_uint(p, 4);
	float value;
	memcpy(&value, &bits, sizeof(value));
	return value;
}

inline void write_float(uint8_t *p, float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	write_uint(p, bits, 4);
}

struct Packet {
	uint8_t kind{REQUEST};
	uint8_t result{0};
	uint8_t frame{1};
	uint8_t child_frame{12};
	uint64_t request_id{0};
	uint64_t t1{0};
	uint64_t t2{0};
	uint64_t t3{0};
	uint64_t source_timestamp{0};
	float q[4] {};
	float variance[3] {};
	float variance_applied[3] {};
	uint64_t timestamp_sample{0};
	uint64_t timestamp_applied{0};
	float q_applied[4] {};
	uint32_t timestamp_uncertainty_us{0};
	uint8_t estimator_instance{0};
};

inline bool decode(const uint8_t *p, unsigned length, Packet &packet)
{
	if (length != LENGTH || read_uint(p, 4) != MAGIC || p[4] != 1 || p[5] > RESULT_ACK
	    || p[9] || p[10] || p[11] || p[117] || p[118] || p[119]) {
		return false;
	}

	packet.kind = p[5];
	packet.result = p[6];
	packet.frame = p[7];
	packet.child_frame = p[8];
	packet.request_id = read_uint(p + 12, 8);
	packet.t1 = read_uint(p + 20, 8);

	if (packet.kind >= RESULT) {
		for (unsigned i = 0; i < 3; ++i) {
			packet.variance_applied[i] = read_float(p + 28 + 4 * i);
		}

	} else {
		packet.t2 = read_uint(p + 28, 8);
		packet.t3 = read_uint(p + 36, 8);
	}

	packet.source_timestamp = read_uint(p + 44, 8);

	for (unsigned i = 0; i < 4; ++i) {
		packet.q[i] = read_float(p + 52 + 4 * i);
		packet.q_applied[i] = read_float(p + 96 + 4 * i);
	}

	for (unsigned i = 0; i < 3; ++i) {
		packet.variance[i] = read_float(p + 68 + 4 * i);
	}

	packet.timestamp_sample = read_uint(p + 80, 8);
	packet.timestamp_applied = read_uint(p + 88, 8);
	packet.timestamp_uncertainty_us = read_uint(p + 112, 4);
	packet.estimator_instance = p[116];
	return true;
}

inline void encode(const Packet &packet, uint8_t *p)
{
	memset(p, 0, LENGTH);
	write_uint(p, MAGIC, 4);
	p[4] = 1;
	p[5] = packet.kind;
	p[6] = packet.result;
	p[7] = packet.frame;
	p[8] = packet.child_frame;
	write_uint(p + 12, packet.request_id, 8);
	write_uint(p + 20, packet.t1, 8);

	if (packet.kind >= RESULT) {
		for (unsigned i = 0; i < 3; ++i) {
			write_float(p + 28 + 4 * i, packet.variance_applied[i]);
		}

	} else {
		write_uint(p + 28, packet.t2, 8);
		write_uint(p + 36, packet.t3, 8);
	}

	write_uint(p + 44, packet.source_timestamp, 8);

	for (unsigned i = 0; i < 4; ++i) {
		write_float(p + 52 + 4 * i, packet.q[i]);
		write_float(p + 96 + 4 * i, packet.q_applied[i]);
	}

	for (unsigned i = 0; i < 3; ++i) {
		write_float(p + 68 + 4 * i, packet.variance[i]);
	}

	write_uint(p + 80, packet.timestamp_sample, 8);
	write_uint(p + 88, packet.timestamp_applied, 8);
	write_uint(p + 112, packet.timestamp_uncertainty_us, 4);
	p[116] = packet.estimator_instance;
}

// t1/t4 belong to the follower; t2/t3/source_timestamp belong to the master.
// With nonnegative network delays, the sample's follower time lies in
// [t1 + ts-t2, t4 + ts-t3]. No clock synchronization or symmetric link is assumed.
inline bool map_time(Packet &packet, uint64_t t4, uint64_t max_round_trip_us = 100000)
{
	if (packet.t1 == 0 || packet.t1 > t4 || t4 - packet.t1 > max_round_trip_us
	    || packet.t2 == 0 || packet.t3 < packet.t2 || packet.t3 - packet.t2 > t4 - packet.t1
	    || packet.source_timestamp == 0 || packet.source_timestamp > packet.t3
	    || packet.t3 - packet.source_timestamp > max_round_trip_us) {
		return false;
	}

	const uint64_t server_duration = packet.t3 - packet.t2;
	const uint64_t sample_age = packet.t3 - packet.source_timestamp;

	// server_duration <= t4-t1 above also proves t1+server_duration cannot overflow.
	if (packet.t1 + server_duration <= sample_age) {
		return false;
	}

	const uint64_t lower = packet.t1 + server_duration - sample_age;
	const uint64_t upper = t4 - sample_age;

	packet.timestamp_sample = lower + (upper - lower) / 2;
	packet.timestamp_uncertainty_us = (upper - lower + 1) / 2;
	return true;
}
} // namespace attitude_bootstrap
