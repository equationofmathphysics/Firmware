/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <AttitudeControl.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf(), Quatf(), 0.f);
	EXPECT_EQ(rate_setpoint, Vector3f());
}

TEST(AttitudeControlTest, Convergence)
{
	AttitudeControl attitude_control;
	attitude_control.setProportionalGain(Vector3f(1,1,1));
	attitude_control.setRateLimit(Vector3f(10000,10000,10000));

	Quatf quat_goal;
	Quatf quat_state(0.996,0.087,0,0);
	// Quatf quat_state(0,0,0,1);

	float error = 10.0f;

	int i;
	for (i = 0; i < 100 || fabsf(error) < 1e-4f; i++) {
		Vector3f rate_setpoint = attitude_control.update(quat_state, quat_goal, 0.f);
		rate_setpoint.print();
		quat_state = quat_state * Quatf(AxisAnglef(rate_setpoint * 0.01f));
		quat_state.normalize();
		quat_state.print();
		const float new_error = Vector<float, 4>(quat_state - quat_goal).norm();
		EXPECT_LT(new_error, error);
		error = new_error;
	}

	EXPECT_LT(i, 100);
}
