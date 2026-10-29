/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#ifndef MOVING_BASE_REFERENCE_HPP
#define MOVING_BASE_REFERENCE_HPP

#include <px4_platform_common/atomic.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/moving_base_reference.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <cmath>
#include <cstring>

class MavlinkStreamMovingBaseReference : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		// 先检查角色与链路，关闭时连采集订阅对象都不创建。
		return link_enabled(mavlink) ? new MavlinkStreamMovingBaseReference(mavlink) : nullptr;
	}
	static constexpr const char *get_name_static() { return "MOVING_BASE_REFERENCE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MOVING_BASE_REFERENCE; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }
	unsigned get_size() override
	{
		return link_enabled() ? MAVLINK_MSG_ID_MOVING_BASE_REFERENCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

	~MavlinkStreamMovingBaseReference() override { release_owner(); }

private:
	explicit MavlinkStreamMovingBaseReference(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Publication<moving_base_reference_s> _reference_pub{ORB_ID(moving_base_reference)};
	uORB::Subscription _selector_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uint64_t _epoch{0};
	uint64_t _last_instance_change{0};
	uint32_t _instance_changed_count{0};
	uint8_t _instance{255};
	bool _multi_mode{false};
	bool _owns_publication{false};

	// 即便手动在多个实例请求该流，也只能有一个内部主题发布者。
	static px4::atomic<Mavlink *> &owner()
	{
		static px4::atomic<Mavlink *> selected{nullptr};
		return selected;
	}

	void release_owner()
	{
		if (_owns_publication) {
			Mavlink *expected = _mavlink;
			owner().compare_exchange(&expected, nullptr);
			_owns_publication = false;
		}
	}

	bool link_enabled() const
	{
		return link_enabled(_mavlink);
	}

	static bool link_enabled(Mavlink *mavlink)
	{
		int32_t role = 0;
		int32_t configured_instance = -1;
		return param_get(param_find("MB_ROLE"), &role) == 0 && role == 1
		       && param_get(param_find("MB_MAV_INST"), &configured_instance) == 0
		       && configured_instance >= 0 && configured_instance <= 2
		       && mavlink->get_moving_base_config_instance() == configured_instance
		       && mavlink->get_protocol() == Protocol::SERIAL
		       && mavlink->get_mode() == Mavlink::MAVLINK_MODE_CUSTOM
		       && strncmp(mavlink->_device_name, "/dev/ttyS", 9) == 0;
	}

	static bool fresh(uint64_t now, uint64_t stamp, uint64_t maximum_age)
	{
		return stamp != 0 && stamp <= now && now - stamp <= maximum_age;
	}

	static bool finite_vector(const float v[3])
	{
		return PX4_ISFINITE(v[0]) && PX4_ISFINITE(v[1]) && PX4_ISFINITE(v[2]);
	}

	static bool valid_quaternion(const float q[4])
	{
		float norm = 0.f;

		for (unsigned i = 0; i < 4; ++i) {
			if (!PX4_ISFINITE(q[i])) { return false; }

			norm += q[i] * q[i];
		}

		return PX4_ISFINITE(norm) && fabsf(norm - 1.f) < 0.05f;
	}

	bool send() override
	{
		if (!link_enabled()) {
			release_owner();
			_epoch = 0;
			return false;
		}

		if (!_owns_publication) {
			Mavlink *expected = nullptr;

			if (!owner().compare_exchange(&expected, _mavlink)) { return false; }

			_owns_publication = true;
			_epoch = 0;
		}

		// 私有消息编号超过255，专用车载链路必须使用MAVLink 2；不修改USB/QGC实例。
		if (_mavlink->getProtocolVersion() != 2) { _mavlink->setProtocolVersion(2); }

		const uint64_t now = hrt_absolute_time();
		constexpr uint64_t data_age = 100000;
		constexpr uint64_t quality_age = 2000000;
		int32_t imu_mode = -1;
		bool mode_known = param_get(param_find("SENS_IMU_MODE"), &imu_mode) == 0;
		bool multi = false;
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
		multi = imu_mode == 0;
#else
		mode_known = mode_known && imu_mode != 0;
#endif
		estimator_selector_status_s selector{};
		bool selection_ok = mode_known;
		uint8_t instance = 0;

		if (multi) {
			selection_ok = _selector_sub.copy(&selector) && fresh(now, selector.timestamp, quality_age)
				       && selector.primary_instance < sizeof(selector.healthy) / sizeof(selector.healthy[0]);
			instance = selection_ok ? selector.primary_instance : 255;
		}

		if (!selection_ok) {
			_epoch = 0;
			_instance = 255;

		} else if (_epoch == 0 || instance != _instance || multi != _multi_mode
			   || (multi && (selector.instance_changed_count != _instance_changed_count
					 || selector.last_instance_change != _last_instance_change))) {
			// 用检测时刻建立新边界。只接受之后采样的数据，不能固定跳过一帧。
			_epoch = hrt_absolute_time();
			_instance = instance;
			_multi_mode = multi;
			_instance_changed_count = selector.instance_changed_count;
			_last_instance_change = selector.last_instance_change;
		}

		// 新实例主题可能稍后才出现，必须重试切换订阅，不能继续读旧实例质量。
		selection_ok = selection_ok && _flags_sub.ChangeInstance(instance);

		vehicle_attitude_s attitude{};
		vehicle_local_position_s position{};
		vehicle_angular_velocity_s angular_velocity{};
		estimator_status_flags_s flags{};
		_attitude_sub.copy(&attitude);
		_position_sub.copy(&position);
		_angular_velocity_sub.copy(&angular_velocity);
		_flags_sub.copy(&flags);

		if (multi && selection_ok) {
			estimator_selector_status_s after{};
			// 采集期间再次切换时，本帧不使用任何融合量；下一帧重新建立边界。
			selection_ok = _selector_sub.copy(&after) && after.primary_instance == selector.primary_instance
				       && after.instance_changed_count == selector.instance_changed_count
				       && after.last_instance_change == selector.last_instance_change;

			if (!selection_ok) { _epoch = 0; }
		}

		moving_base_reference_s ref{};
		ref.timestamp = hrt_absolute_time();
		ref.timestamp_attitude_sample = attitude.timestamp_sample;
		ref.timestamp_velocity_sample = position.timestamp_sample;
		ref.timestamp_angular_velocity_sample = angular_velocity.timestamp_sample;
		memcpy(ref.q, attitude.q, sizeof(ref.q));
		ref.velocity[0] = position.vx;
		ref.velocity[1] = position.vy;
		ref.velocity[2] = position.vz;
		memcpy(ref.angular_velocity, angular_velocity.xyz, sizeof(ref.angular_velocity));
		ref.quat_reset_counter = attitude.quat_reset_counter;
		ref.vxy_reset_counter = position.vxy_reset_counter;
		ref.vz_reset_counter = position.vz_reset_counter;
		ref.estimator_instance = selection_ok ? _instance : 255;

		// 保守退化：各融合量和所选质量样本越过切换边界后才恢复对应有效位。
		// 不回用旧实例，不锁死选择器；角速度独立检查，允许部分有效或全无效帧继续发送。
		const bool coherent = selection_ok && (!multi || selector.healthy[instance]) && _epoch != 0
				      && flags.timestamp_sample > _epoch
				      && flags.timestamp_sample <= flags.timestamp
				      && fresh(now, flags.timestamp, quality_age)
				      && fresh(now, flags.timestamp_sample, quality_age);
		const bool attitude_ok = coherent && fresh(now, attitude.timestamp, data_age)
					 && attitude.timestamp_sample > _epoch
					 && attitude.timestamp_sample <= attitude.timestamp
					 && fresh(now, attitude.timestamp_sample, data_age) && valid_quaternion(ref.q);
		const bool position_ok = coherent && fresh(now, position.timestamp, data_age)
					 && position.timestamp_sample > _epoch
					 && position.timestamp_sample <= position.timestamp
					 && fresh(now, position.timestamp_sample, data_age);
		const bool tilt_ok = attitude_ok && flags.cs_tilt_align
				     && !flags.fs_bad_acc_vertical && !flags.fs_bad_acc_clipping;

		if (tilt_ok) { ref.validity_flags |= moving_base_reference_s::VALID_TILT; }

		if (tilt_ok && position_ok && flags.cs_yaw_align && position.heading_good_for_control && !flags.fs_bad_hdg) {
			ref.validity_flags |= moving_base_reference_s::VALID_HEADING;
		}

		const bool velocity_ok = position_ok && !position.dead_reckoning
					 && !flags.cs_inertial_dead_reckoning && !flags.cs_wind_dead_reckoning
					 && !flags.cs_fake_pos && !flags.cs_valid_fake_pos
					 && !flags.fs_bad_acc_vertical && !flags.fs_bad_acc_clipping;

		if (velocity_ok && position.v_xy_valid && PX4_ISFINITE(position.vx) && PX4_ISFINITE(position.vy)) {
			ref.validity_flags |= moving_base_reference_s::VALID_HORIZONTAL_VELOCITY;
		}

		if (velocity_ok && position.v_z_valid && PX4_ISFINITE(position.vz)) {
			ref.validity_flags |= moving_base_reference_s::VALID_VERTICAL_VELOCITY;
		}

		if (fresh(now, angular_velocity.timestamp, data_age)
		    && angular_velocity.timestamp_sample <= angular_velocity.timestamp
		    && fresh(now, angular_velocity.timestamp_sample, data_age) && finite_vector(ref.angular_velocity)) {
			ref.validity_flags |= moving_base_reference_s::VALID_ANGULAR_VELOCITY;
		}

		_reference_pub.publish(ref);
		mavlink_moving_base_reference_t msg{};
		msg.time_boot_us = ref.timestamp;
		msg.attitude_sample_us = ref.timestamp_attitude_sample;
		msg.velocity_sample_us = ref.timestamp_velocity_sample;
		msg.angular_velocity_sample_us = ref.timestamp_angular_velocity_sample;
		memcpy(msg.q, ref.q, sizeof(msg.q));
		memcpy(msg.velocity, ref.velocity, sizeof(msg.velocity));
		memcpy(msg.angular_velocity, ref.angular_velocity, sizeof(msg.angular_velocity));
		msg.protocol_version = 1;
		msg.validity_flags = ref.validity_flags;
		msg.quat_reset_counter = ref.quat_reset_counter;
		msg.vxy_reset_counter = ref.vxy_reset_counter;
		msg.vz_reset_counter = ref.vz_reset_counter;
		msg.estimator_instance = ref.estimator_instance;
		mavlink_msg_moving_base_reference_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MOVING_BASE_REFERENCE_HPP
