/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

// 补充车载地面磁航向入口；调用者仍须检查姿态、实例和样本时效。
// 不改变 EKF 的飞行中磁对齐状态，也不作为机载解锁或起飞许可。
template<typename EstimatorFlags>
bool moving_base_heading_ready(const EstimatorFlags &flags, bool heading_good_for_control)
{
	if (!flags.cs_yaw_align || flags.fs_bad_hdg) {
		return false;
	}

	// 保留已有完整航向质量路径，例如合格的双天线 GNSS 航向。
	if (heading_good_for_control) {
		return true;
	}

	// 车载端不要求起飞才能完成磁航向质量判定。只接受正在融合、
	// 与滤波器一致且无磁场扰动、磁传感器或磁融合故障的磁航向。
	return (flags.cs_mag_hdg || flags.cs_mag_3d)
	       && flags.cs_mag_heading_consistent
	       && !flags.cs_mag_field_disturbed && !flags.cs_mag_fault
	       && !flags.fs_bad_mag_x && !flags.fs_bad_mag_y && !flags.fs_bad_mag_z
	       && !flags.fs_bad_mag_decl;
}
