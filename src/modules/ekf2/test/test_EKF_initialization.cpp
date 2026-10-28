/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <math.h>
#include <memory>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfInitializationTest : public ::testing::Test
{
public:
	EkfInitializationTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	const float _init_tilt_period = 0.7f; // seconds

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// first init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void initializedOrienationIsMatchingGroundTruth(Quatf true_quaternion)
	{
		const Quatf quat_est = _ekf->getQuaternion();
		EXPECT_TRUE(matrix::isEqual(quat_est, true_quaternion))
				<< "quat est = " << quat_est(0) << ", " << quat_est(1) << ", "
				<< quat_est(2) << ", " << quat_est(3)
				<< "\nquat true = " << true_quaternion(0) << ", " << true_quaternion(1) << ", "
				<< true_quaternion(2) << ", " << true_quaternion(3);
	}

	void quaternionVarianceBigEnoughAfterOrientationInitialization(float quat_variance_limit = 0.00001f)
	{
		const matrix::Vector3f quat_variance = _ekf->getRotVarBody();
		EXPECT_TRUE(quat_variance(0) > quat_variance_limit) << "quat_variance(3): " << quat_variance(0);
		EXPECT_TRUE(quat_variance(1) > quat_variance_limit) << "quat_variance(1): " << quat_variance(1);
		EXPECT_TRUE(quat_variance(2) > quat_variance_limit) << "quat_variance(2): " << quat_variance(2);
	}

	void yawVarianceBigEnoughAfterHeadingReset()
	{
		// The yaw variance is smaller than its reset value as we do not probe instantly after the reset
		EXPECT_GT(sqrtf(_ekf->getYawVar()), _ekf_wrapper.getMagHeadingNoise() / 5.f);
	}

	void velocityAndPositionCloseToZero()
	{
		const Vector3f pos = _ekf->getPosition();
		const Vector3f vel = _ekf->getVelocity();

		EXPECT_TRUE(matrix::isEqual(pos, Vector3f{}, 0.002f))
				<< "pos = " << pos(0) << ", " << pos(1) << ", " << pos(2);
		EXPECT_TRUE(matrix::isEqual(vel, Vector3f{}, 0.003f))
				<< "vel = " << vel(0) << ", " << vel(1) << ", " << vel(2);
	}

	void velocityVarianceBigEnoughAfterOrientationInitialization(float vel_variance_limit)
	{
		const Vector3f vel_var = _ekf->getVelocityVariance();

		EXPECT_TRUE(vel_var(0) > vel_variance_limit) << "vel_var(0): " << vel_var(0);
		EXPECT_TRUE(vel_var(1) > vel_variance_limit) << "vel_var(1): " << vel_var(1);
		EXPECT_TRUE(vel_var(2) > vel_variance_limit) << "vel_var(2): " << vel_var(2);
	}

	void positionVarianceBigEnoughAfterOrientationInitialization(float pos_variance_limit)
	{
		const Vector3f pos_var = _ekf->getPositionVariance();

		EXPECT_TRUE(pos_var(0) > pos_variance_limit) << "pos_var(0): " << pos_var(0);
		EXPECT_TRUE(pos_var(1) > pos_variance_limit) << "pos_var(1): " << pos_var(1);
		EXPECT_TRUE(pos_var(2) > pos_variance_limit) << "pos_var(2): " << pos_var(2);
	}

	void learningCorrectAccelBias()
	{
		const Dcmf R_to_earth = Dcmf(_ekf->getQuaternion());
		const Vector3f accel_bias_var = _ekf->getAccelBiasVariance();
		const Vector3f accel_bias = _ekf->getAccelBias();

		for (int i = 0; i < 3; i++) {
			if (fabsf(R_to_earth(2, i)) > 0.8f) {
				// Highly observable, the variance decreases
				EXPECT_LT(accel_bias_var(i), 4.0e-2f) << "axis " << i;
			}

			EXPECT_LT(accel_bias(i), 4.0e-6f) << "axis " << i;
		}
	}
};

TEST_F(EkfInitializationTest, initializeWithZeroTilt)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(0.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);
	EXPECT_FALSE(_ekf->control_status_flags().in_air);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithZeroTiltNotAtRest)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(0.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f); // Fake position fusion obs var when at rest sq(0.5f)
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithTiltNoGyroBiasEstimate)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(-20.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf_wrapper.disableGyroBiasEstimation();
	_sensor_simulator.simulateOrientation(quat_sim);

	_sensor_simulator.runSeconds(_init_tilt_period);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f); // Fake position fusion obs var when at rest sq(0.5f)
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, gyroBias)
{
	// GIVEN: a healthy filter
	_sensor_simulator.runSeconds(20);

	// WHEN: there is a yaw gyro bias after initial convergence of the filter
	_sensor_simulator._imu.setGyroData(Vector3f(0.f, 0.f, 0.1f));

	// THEN: the vertical accel bias should not be affected
	Vector3f accel_bias;

	for (int ii = 0; ii < 100; ii++) {
		_sensor_simulator.runSeconds(0.5);
		accel_bias = _ekf->getAccelBias();

		if (fabsf(accel_bias(2)) > 0.3f) {

			// Print state covariance and correlation matrices for debugging
			const auto P = _ekf->covariances();

			printf("State covariance:\n");

			for (int i = 0; i <= State::size; i++) {
				for (int j = 0; j <= State::size; j++) {
					printf("%.3fe-9  ", ((double)P(i, j)) * 1e9);
				}

				printf("\n");
			}

			printf("State correlation:\n");
			printf("\t0\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\n");

			for (uint8_t i = 0; i <= State::size; i++) {
				printf("%d|  ", i);

				for (uint8_t j = 0; j <= State::size; j++) {
					float corr = sqrtf(fabsf(P(i, i) * P(j, j)));

					if (corr > 0.0f) {
						corr = fabsf(P(i, j)) / corr;
					}

					printf("%.3f\t", (double)corr);
				}

				printf("\n");
			}

			printf("Accel bias = (%f, %f, %f)\n", (double)accel_bias(0), (double)accel_bias(1), (double)accel_bias(2));

			Vector3f gyro_bias = _ekf->getGyroBias();
			printf("Gyro bias = (%f, %f, %f)\n", (double)gyro_bias(0), (double)gyro_bias(1), (double)gyro_bias(2));

			EXPECT_TRUE(false);
			break;
		}
	}
}

TEST_F(EkfInitializationTest, initializeHeadingWithZeroTilt)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(0.0f);
	const float yaw = math::radians(90.0f);
	const Eulerf euler_angles_sim(roll, pitch, yaw);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);
	EXPECT_FALSE(_ekf->control_status_flags().in_air);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	yawVarianceBigEnoughAfterHeadingReset();

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithTilt)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(60.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	yawVarianceBigEnoughAfterHeadingReset();

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithTiltNotAtRest)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(60.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	positionVarianceBigEnoughAfterOrientationInitialization(0.01f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithPitch90)
{
	const float pitch = math::radians(90.0f);
	const float roll = math::radians(0.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(false);
	_ekf->set_vehicle_at_rest(true);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	// TODO: Quaternion Variance is smaller and vel x is larger
	// in this case than in the other cases

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithRoll90)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(90.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(false);
	_ekf->set_vehicle_at_rest(true);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, bootstrapRequiredWaitsWithoutAccelerometerFallback)
{
	_ekf->requireAttitudeBootstrap(true);
	_sensor_simulator.runSeconds(2.f);
	EXPECT_FALSE(_ekf->attitude_valid());
	EXPECT_FALSE(_ekf->control_status_flags().tilt_align);
	EXPECT_FALSE(_ekf->attitudeBootstrapApplied());
}

TEST_F(EkfInitializationTest, bootstrapInitializesFullAttitudeUnderAcceleration)
{
	_ekf->requireAttitudeBootstrap(true);
	_ekf_wrapper.setMagFuseTypeNone();
	_sensor_simulator._imu.setAccelData(Vector3f{5.f, -3.f, -CONSTANTS_ONE_G});
	_sensor_simulator.runSeconds(0.7f);
	ASSERT_FALSE(_ekf->attitude_valid());
	const Quatf seed{Eulerf{0.2f, -0.1f, 0.7f}};
	const Vector3f variance{sq(0.01f), sq(0.015f), sq(0.03f)};
	const uint64_t sample_time = _sensor_simulator.getTime() - 10000;
	ASSERT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, variance), Ekf::AttitudeBootstrapResult::Waiting);
	_sensor_simulator.runSeconds(0.3f);
	ASSERT_TRUE(_ekf->attitudeBootstrapApplied());
	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);
	EXPECT_TRUE(matrix::isEqual(_ekf->attitudeBootstrapQuaternion(), seed, 1e-4f));
	EXPECT_GE(_ekf->attitudeBootstrapTime(), sample_time);
	EXPECT_GT(_ekf->getRotVarNed()(0), 0.f);
	EXPECT_GT(_ekf->getRotVarNed()(1), 0.f);
	EXPECT_GT(_ekf->getRotVarNed()(2), 0.f);

	// Once initialized, a different source attitude cannot reset the local estimate.
	const Quatf unrelated_seed{Eulerf{-0.3f, 0.4f, -2.f}};
	EXPECT_EQ(_ekf->setAttitudeBootstrap(_sensor_simulator.getTime() - 10000, unrelated_seed, variance),
		  Ekf::AttitudeBootstrapResult::AlreadyInitialized);
	EXPECT_TRUE(matrix::isEqual(_ekf->attitudeBootstrapQuaternion(), seed, 1e-4f));
}

TEST_F(EkfInitializationTest, bootstrapAlignsDelayedHorizonAndContinuesLocalGyroPropagation)
{
	_ekf->requireAttitudeBootstrap(true);
	_ekf_wrapper.setMagFuseTypeNone();
	const float yaw_rate = 0.7f;
	_sensor_simulator._imu.setData(Vector3f{4.f, 1.f, -CONSTANTS_ONE_G}, Vector3f{0.f, 0.f, yaw_rate});
	_sensor_simulator.runSeconds(0.7f);
	const Quatf seed{Eulerf{0.f, 0.f, 0.8f}};
	const uint64_t sample_time = _sensor_simulator.getTime() - 13000; // intentionally inside an IMU interval
	ASSERT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, Vector3f{0.0001f, 0.0001f, 0.0001f}),
		  Ekf::AttitudeBootstrapResult::Waiting);
	_sensor_simulator.runSeconds(0.3f);
	ASSERT_TRUE(_ekf->attitudeBootstrapApplied());
	const float dt = (_ekf->attitudeBootstrapTime() - sample_time) * 1e-6f;
	const Quatf expected = seed * Quatf{AxisAnglef{Vector3f{0.f, 0.f, yaw_rate * dt}}};
	EXPECT_TRUE(matrix::isEqual(_ekf->attitudeBootstrapQuaternion(), expected, 2e-4f));

	const float yaw_before = Eulerf{_ekf->getQuaternion()}.psi();
	const float realtime_dt = (_sensor_simulator.getTime() - sample_time) * 1e-6f;
	EXPECT_NEAR(matrix::wrap_pi(yaw_before - Eulerf{seed}.psi()), yaw_rate * realtime_dt, 0.02f);
	_sensor_simulator.runSeconds(0.3f);
	const float yaw_after = Eulerf{_ekf->getQuaternion()}.psi();
	EXPECT_NEAR(matrix::wrap_pi(yaw_after - yaw_before), yaw_rate * 0.3f, 0.03f);
}

TEST_F(EkfInitializationTest, bootstrapRejectsInvalidLateFutureAndDuplicateData)
{
	const Quatf seed{};
	const Vector3f variance{0.0001f, 0.0001f, 0.0001f};
	EXPECT_EQ(_ekf->setAttitudeBootstrap(1000, seed, variance), Ekf::AttitudeBootstrapResult::Disabled);
	_ekf->requireAttitudeBootstrap(true);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(1000, seed, variance), Ekf::AttitudeBootstrapResult::NoImu);
	_sensor_simulator.runSeconds(0.7f);
	const uint64_t sample_time = _sensor_simulator.getTime() - 10000;
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, Quatf{NAN, 0.f, 0.f, 0.f}, variance),
		  Ekf::AttitudeBootstrapResult::InvalidQuaternion);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, Quatf{2.f, 0.f, 0.f, 0.f}, variance),
		  Ekf::AttitudeBootstrapResult::InvalidQuaternion);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, Vector3f{0.f, 0.001f, 0.001f}),
		  Ekf::AttitudeBootstrapResult::InvalidVariance);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, Vector3f{NAN, 0.001f, 0.001f}),
		  Ekf::AttitudeBootstrapResult::InvalidVariance);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, Vector3f{-0.001f, 0.001f, 0.001f}),
		  Ekf::AttitudeBootstrapResult::InvalidVariance);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, Vector3f{INFINITY, 0.001f, 0.001f}),
		  Ekf::AttitudeBootstrapResult::InvalidVariance);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(1, seed, variance), Ekf::AttitudeBootstrapResult::Stale);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(_sensor_simulator.getTime() + 100000, seed, variance),
		  Ekf::AttitudeBootstrapResult::Future);
	ASSERT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, variance), Ekf::AttitudeBootstrapResult::Waiting);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, variance), Ekf::AttitudeBootstrapResult::Busy);
	_sensor_simulator.runSeconds(0.3f);
	EXPECT_EQ(_ekf->setAttitudeBootstrap(sample_time, seed, variance), Ekf::AttitudeBootstrapResult::AlreadyInitialized);
}

TEST_F(EkfInitializationTest, bootstrapRejectsMissingImuCoverage)
{
	_ekf->requireAttitudeBootstrap(true);
	_sensor_simulator.runSeconds(0.7f);
	_sensor_simulator._imu.stop();
	_sensor_simulator.runSeconds(0.5f);
	_sensor_simulator._imu.start();
	_sensor_simulator.runMicroseconds(1000);

	// This otherwise fresh sample falls in the IMU outage, rather than in a
	// buffered integration interval. It must not be extrapolated across the gap.
	const uint64_t sample_time = _sensor_simulator.getTime() - 100000;
	ASSERT_EQ(_ekf->setAttitudeBootstrap(sample_time, Quatf{}, Vector3f{0.0001f, 0.0001f, 0.0001f}),
		  Ekf::AttitudeBootstrapResult::Waiting);
	_sensor_simulator.runSeconds(0.4f);
	EXPECT_FALSE(_ekf->attitudeBootstrapApplied());
	EXPECT_FALSE(_ekf->attitude_valid());
	EXPECT_EQ(_ekf->attitudeBootstrapResult(), Ekf::AttitudeBootstrapResult::Stale);
}

TEST_F(EkfInitializationTest, bootstrapYawSurvivesFirstLocalMagnetometerSample)
{
	_ekf->requireAttitudeBootstrap(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator._mag.stop();
	const Quatf seed{Eulerf{0.1f, -0.05f, 0.6f}};
	_sensor_simulator.simulateOrientation(seed);
	_sensor_simulator.runSeconds(0.7f);
	ASSERT_EQ(_ekf->setAttitudeBootstrap(_sensor_simulator.getTime() - 10000, seed,
					   Vector3f{0.0001f, 0.0001f, 0.0001f}), Ekf::AttitudeBootstrapResult::Waiting);
	_sensor_simulator.runSeconds(0.3f);
	ASSERT_TRUE(_ekf->control_status_flags().yaw_align);
	const uint8_t quat_resets = _ekf->get_quat_reset_count();
	_sensor_simulator._mag.start();
	_sensor_simulator.runSeconds(0.5f);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);
	EXPECT_TRUE(_ekf->control_status_flags().mag);
	EXPECT_EQ(_ekf->get_quat_reset_count(), quat_resets);
	EXPECT_NEAR(matrix::wrap_pi(Eulerf{_ekf->getQuaternion()}.psi() - Eulerf{seed}.psi()), 0.f, 0.01f);
}

TEST_F(EkfInitializationTest, bootstrapInitializesUnknownVelocityFromFirstQualifiedOwnGnssOnly)
{
	_ekf->requireAttitudeBootstrap(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->getParamHandle()->ekf2_imu_ctrl |= static_cast<int32_t>(ImuCtrl::GravityVector);
	_ekf->set_min_required_gps_health_time(500000);
	_ekf_wrapper.enableGpsFusion();
	const Quatf seed{Eulerf{0.f, 0.f, 0.5f}};
	_sensor_simulator.simulateOrientation(seed);
	_sensor_simulator._gps.setVelocity(Vector3f{3.f, 0.f, 0.f});
	_sensor_simulator._gps.setPositionRateNED(Vector3f{3.f, 0.f, 0.f});
	_sensor_simulator._gps.setNumberOfSatellites(3); // Keep the ordinary GNSS quality checks failing initially.
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(0.7f);
	ASSERT_EQ(_ekf->setAttitudeBootstrap(_sensor_simulator.getTime() - 10000, seed,
					   Vector3f{0.0001f, 0.0001f, 0.0004f}), Ekf::AttitudeBootstrapResult::Waiting);
	_sensor_simulator.runSeconds(1.f);
	ASSERT_TRUE(_ekf->attitudeBootstrapApplied());
	EXPECT_FALSE(_ekf->control_status_flags().gnss_vel);
	EXPECT_FALSE(_ekf->information_event_flags().reset_vel_to_gps);

	_sensor_simulator._gps.setNumberOfSatellites(16);
	_sensor_simulator.runSeconds(4.f);
	ASSERT_TRUE(_ekf->control_status_flags().gnss_vel);
	EXPECT_TRUE(_ekf->information_event_flags().reset_vel_to_gps);
	EXPECT_NEAR(_ekf->getVelocity()(0), 3.f, 0.15f);
	EXPECT_NEAR(_ekf->getVelocity()(1), 0.f, 0.15f);
	EXPECT_LT(_ekf->getGyroBias().norm(), 0.01f);
	EXPECT_NEAR(matrix::wrap_pi(Eulerf{_ekf->getQuaternion()}.psi() - Eulerf{seed}.psi()), 0.f, 0.1f);

	// After normal velocity initialization, a later outage/reacquisition uses
	// the normal fusion policy, rather than repeating the bootstrap reset.
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(9.f);
	ASSERT_FALSE(_ekf->control_status_flags().gnss_vel);
	const uint8_t velocity_resets = _ekf->get_velNE_reset_count();
	_ekf->clear_information_events();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(3.f);
	ASSERT_TRUE(_ekf->control_status_flags().gnss_vel);
	EXPECT_EQ(_ekf->get_velNE_reset_count(), velocity_resets);
	EXPECT_FALSE(_ekf->information_event_flags().reset_vel_to_gps);
	EXPECT_NEAR(_ekf->getVelocity()(0), 3.f, 0.2f);
	EXPECT_LT(_ekf->getGyroBias().norm(), 0.01f);
}
