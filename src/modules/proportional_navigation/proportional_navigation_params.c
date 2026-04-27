/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * Navigation gain
 *
 * @min 2
 * @max 6
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PNAV_GAIN, 3);

/**
 * Maximum acceleration
 *
 * @unit m/s^2
 * @min 5.0
 * @max 50.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_MAX_ACCEL, 40.f);

/**
 * Maximum velocity
 *
 * @unit m/s
 * @min 5.0
 * @max 100.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_MAX_VEL, 50.f);

/**
 * Vertical impact angle
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_IMPACT_ANG_V, -45.f);

/**
 * Horizontal impact angle
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_IMPACT_ANG_H, 0.f);

/**
 * Miss distance threshold
 *
 * @unit m
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_MISS_DIST, 2.f);

/**
 * Constraint mode
 *
 * @value 0 No constraint
 * @value 1 Vertical constraint
 * @value 2 Horizontal constraint
 * @group Proportional Navigation
 */
PARAM_DEFINE_INT32(PN_CONSTR_MODE, 1);

/**
 * Enable pullup maneuver
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group Proportional Navigation
 */
PARAM_DEFINE_INT32(PN_ENABLE_PULLUP, 0);

/**
 * Pullup trigger distance
 *
 * @unit m
 * @min 2.0
 * @max 20.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_PULLUP_DIST, 5.f);

/**
 * Pullup target altitude
 *
 * @unit m
 * @min -100.0
 * @max -10.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_PULLUP_ALT, -30.f);

/**
 * Pullup acceleration
 *
 * @unit m/s^2
 * @min 5.0
 * @max 20.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_PULLUP_ACCEL, 10.f);

/**
 * Disable velocity limit during engagement
 *
 * @value 0 Enforce velocity limit
 * @value 1 No velocity limit (maximum speed)
 * @group Proportional Navigation
 */
PARAM_DEFINE_INT32(PN_NO_VEL_LIMIT, 0);

/**
 * Strike phase thrust command
 *
 * Normalized thrust command used during direct-attitude ground strike.
 * Higher values push harder for terminal impact speed.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_STRIKE_THRUST, 1.0f);

/**
 * Strike minimum dive pitch
 *
 * Minimum nose-down pitch magnitude commanded during direct-attitude ground strike.
 *
 * @unit deg
 * @min 10.0
 * @max 85.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_STRIKE_PITCH, 70.0f);

/**
 * Strike maximum dive pitch
 *
 * Upper pitch magnitude limit for direct-attitude ground strike.
 *
 * @unit deg
 * @min 20.0
 * @max 89.0
 * @decimal 1
 * @group Proportional Navigation
 */
PARAM_DEFINE_FLOAT(PN_STRIKE_MAX_P, 82.0f);
