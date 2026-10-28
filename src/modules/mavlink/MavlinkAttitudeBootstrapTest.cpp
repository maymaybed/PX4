/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>
#include "mavlink_attitude_bootstrap_protocol.h"
#include <limits>

using namespace attitude_bootstrap;

TEST(MavlinkAttitudeBootstrap, SourceCannotAssertFollowerApplication)
{
	EXPECT_FALSE(valid_source_refusal(0)); // ACCEPTED belongs exclusively to follower EKF
	EXPECT_FALSE(valid_source_refusal(12)); // WAITING is not a source refusal
	EXPECT_FALSE(valid_source_refusal(255));
	EXPECT_TRUE(valid_source_refusal(1));
	EXPECT_TRUE(valid_source_refusal(11));
	EXPECT_TRUE(valid_source_refusal(13));
	EXPECT_TRUE(valid_source_refusal(16));
	EXPECT_TRUE(valid_source_refusal(17));
}

TEST(MavlinkAttitudeBootstrap, WireRoundtripAndMalformed)
{
	Packet sample;
	sample.kind = SAMPLE;
	sample.request_id = 0x123456789abcdef0;
	sample.t1 = 3000000;
	sample.t2 = 1000000;
	sample.t3 = 1000001;
	sample.source_timestamp = 999000;
	sample.q[0] = 0.70710678f;
	sample.q[3] = -0.70710678f;
	sample.variance[0] = 0.001f;
	sample.variance[1] = 0.002f;
	sample.variance[2] = 0.003f;
	sample.timestamp_sample = 2999000;
	sample.timestamp_applied = 2999500;
	sample.q_applied[0] = 1.f;
	sample.timestamp_uncertainty_us = 1234;
	sample.estimator_instance = 2;
	uint8_t bytes[LENGTH];
	encode(sample, bytes);
	EXPECT_EQ(bytes[12], 0xf0);
	EXPECT_EQ(bytes[19], 0x12);
	Packet decoded;
	ASSERT_TRUE(decode(bytes, LENGTH, decoded));
	EXPECT_EQ(decoded.request_id, sample.request_id);
	EXPECT_EQ(decoded.timestamp_sample, sample.timestamp_sample);
	EXPECT_EQ(decoded.timestamp_applied, sample.timestamp_applied);
	EXPECT_EQ(decoded.timestamp_uncertainty_us, sample.timestamp_uncertainty_us);
	EXPECT_FLOAT_EQ(decoded.q[3], sample.q[3]);
	EXPECT_FLOAT_EQ(decoded.variance[2], sample.variance[2]);
	EXPECT_FALSE(decode(bytes, LENGTH - 1, decoded));
	EXPECT_FALSE(decode(bytes, LENGTH + 1, decoded));
	bytes[4] = 2;
	EXPECT_FALSE(decode(bytes, LENGTH, decoded));
	encode(sample, bytes);
	bytes[117] = 1;
	EXPECT_FALSE(decode(bytes, LENGTH, decoded));
	sample.kind = RESULT;
	sample.variance_applied[0] = 0.01f;
	sample.variance_applied[1] = 0.02f;
	sample.variance_applied[2] = 0.03f;
	encode(sample, bytes);
	ASSERT_TRUE(decode(bytes, LENGTH, decoded));
	EXPECT_FLOAT_EQ(decoded.variance_applied[0], sample.variance_applied[0]);
	EXPECT_FLOAT_EQ(decoded.variance_applied[2], sample.variance_applied[2]);
}

TEST(MavlinkAttitudeBootstrap, IndependentClocksAndAsymmetricLink)
{
	Packet sample;
	// The master's boot clock is 10 hours ahead; link delays are 1ms/9ms.
	sample.t1 = 1000000;
	sample.t2 = 36001001000ULL;
	sample.t3 = sample.t2 + 200;
	sample.source_timestamp = sample.t2 - 500;
	ASSERT_TRUE(map_time(sample, 1010200));
	EXPECT_EQ(sample.timestamp_sample, 1004500);
	EXPECT_EQ(sample.timestamp_uncertainty_us, 5000);
	// The actual follower-local sampling time (1000500) is inside the interval.
	EXPECT_LE(sample.timestamp_sample - sample.timestamp_uncertainty_us, 1000500);
	EXPECT_GE(sample.timestamp_sample + sample.timestamp_uncertainty_us, 1000500);
}

TEST(MavlinkAttitudeBootstrap, InvalidTimesAndOverflow)
{
	Packet sample;
	sample.t1 = 1000000;
	sample.t2 = std::numeric_limits<uint64_t>::max() - 200;
	sample.t3 = sample.t2 + 100;
	sample.source_timestamp = sample.t2 - 1000;
	ASSERT_TRUE(map_time(sample, 1002100));
	EXPECT_EQ(sample.timestamp_sample, 1000000);
	EXPECT_EQ(sample.timestamp_uncertainty_us, 1000);
	sample.source_timestamp = sample.t3 + 1;
	EXPECT_FALSE(map_time(sample, 1002100));
	sample.source_timestamp = sample.t3 - 1000;
	EXPECT_FALSE(map_time(sample, 1100001));
	sample.t3 = sample.t2 - 1;
	EXPECT_FALSE(map_time(sample, 1002100));
	sample.t3 = sample.t2 + 10000;
	EXPECT_FALSE(map_time(sample, 1002100));
	sample.t1 = 1;
	sample.t2 = 1000000;
	sample.t3 = 1000001;
	sample.source_timestamp = 999000;
	EXPECT_FALSE(map_time(sample, 100));
}
