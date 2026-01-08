/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file swarm_node_params.c
 * Swarm Function parameters.
 *
 * @author yr<17729322112@163.com>
 */

/**
 * Set as leader Vehicle
 *
 * Set this Vehicle as the fleet lead Vehicle
 * Note:Effective only when the swarm function is enabled.
 *
 * @boolean
 * @group Swarm
 */
PARAM_DEFINE_INT32(SWARM_SET_LEADER, 0);

/**
 * Which swarm group this Vehicle belongs
 *
 * This parameter defines which swarm group this Vehicle belongs to when
 * the swarm function is enabled
 *
 * @min 0
 * @max 255
 * @group Swarm
 */
PARAM_DEFINE_INT32(SWARM_GROUP_ID, 1);

/**
 * Leader Vehicle ID for follower
 *
 * This parameter defines which vehicle ID the follower should follow.
 * Set by QGroundControl when selecting a leader vehicle.
 * 0 = Auto-detect (use first available leader), >0 = Specific vehicle ID
 *
 * @min 0
 * @max 255
 * @group Swarm
 */
PARAM_DEFINE_INT32(SWARM_LEADER_ID, 0);

/**
 * X Offset relative to the leader vehicle(NED)
 *
 * North offset position in leader NED earth-fixed frame.
 * Note: This parameter is only valid when leader vehicle is set.
 *
 * @unit m
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_X_OFFSET, 5.00f);

/**
 * Y Offset relative to the leader vehicle(NED)
 *
 * East position offset in NED earth-fixed frame
 * Note: This parameter is only valid when leader vehicle is set.
 *
 * @unit m
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_Y_OFFSET, 0.00f);

/**
 * Z Offset relative to the leader vehicle(NED)
 *
 * Down position (negative altitude) offset in NED earth-fixed frame
 * Note: This parameter is only valid when leader vehicle is set.
 *
 * @unit m
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_Z_OFFSET, 0.00f);


/**
 * Absolute altitude for swarm aircraft
 *
 * Set the absolute altitude for this aircraft in the swarm.
 * When set to a positive value, the aircraft will fly at this altitude
 * regardless of whether it's a leader or follower.
 * Value of 0 means use default behavior (follow leader + offset).
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_ABS_ALT, 0.0f);

/**
 * Enable APF collision avoidance
 *
 * Enable or disable the Artificial Potential Field collision avoidance system.
 * 0 = Disable, 1 = Enable
 *
 * @min 0
 * @max 1
 * @increment 1
 * @group Swarm
 */
PARAM_DEFINE_INT32(SWARM_APF_ENABLE, 1);

/**
 * APF safety radius for collision avoidance
 *
 * Safety radius within which the aircraft will actively avoid collision.
 * Aircraft will start avoiding maneuvers when other aircraft come within this distance.
 * Note: Set to 2.5m to work with 7m formation spacing.
 *
 * @unit m
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_APF_SAFE_R, 2.5f);

/**
 * APF danger radius for emergency avoidance
 *
 * Dangerous radius for emergency collision avoidance within which maximum
 * repulsive force will be applied.
 * Note: Set to 1.2m for 7m formation spacing.
 *
 * @unit m
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_APF_DANGER, 2.0f);

/**
 * APF maximum avoidance distance
 *
 * Maximum distance at which the aircraft starts to apply avoidance force.
 * Beyond this distance, no avoidance force will be applied.
 * Note: Set to 7.0m as hard limit - no avoidance beyond 7m regardless of speed.
 *
 * @unit m
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_APF_MAXD, 7.0f);

/**
 * APF repulsive force gain
 *
 * Gain for the repulsive force that pushes aircraft away from each other.
 * Higher values create stronger avoidance forces.
 * Note: Increased to 2.5 for better avoidance with 7m formation.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_APF_REP_G, 2.5f);

/**
 * APF tangential force gain for clockwise avoidance
 *
 * Gain for the tangential (lateral) force that creates clockwise sliding motion.
 * This force helps aircraft pass each other smoothly.
 * Note: Increased to 2.0 for better lateral avoidance.
 *
 * @min 0.0
 * @max 15.0
 * @decimal 2
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_APF_TAN_G, 5.0f);

/**
 * APF maximum avoidance force magnitude
 *
 * Maximum magnitude of the total avoidance force vector to prevent excessive maneuvers.
 * This limits the maximum speed change applied for collision avoidance.
 *
 * @unit m/s
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_APF_MAX_F, 3.00f);

/**
 * Enable APF avoidance with leader aircraft
 *
 * Whether to apply collision avoidance with the leader aircraft.
 * 0 = Only avoid other followers, 1 = Avoid all aircraft including leader
 *
 * @min 0
 * @max 1
 * @increment 1
 * @group Swarm
 */
PARAM_DEFINE_INT32(SWARM_APF_LEADER, 1);


/**
 * Swarm UAV takeoff altitude
 *
 *Set the takeoff altitude for the cluster. The aircraft first takes off
 *from its current position to a specific altitude, then follows the
 * formation set by the ground station.
 *
 * @unit m
 * @min 1.0
 * @max 50.0
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_TOFF_ALT, 5.0f);

// ============== 编队控制参数 ==============

/**
 * Formation switch detection threshold
 *
 * Distance threshold for detecting formation switching (target position jump).
 * When target position changes more than this value, formation switching is triggered.
 *
 * @unit m
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_FRM_SWTHR, 3.0f);

/**
 * Need avoidance distance threshold
 *
 * Distance to target below which collision avoidance is disabled.
 * When closer than this to target, avoidance is not needed.
 *
 * @unit m
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_AVOID_DST, 3.0f);

/**
 * At target distance threshold
 *
 * Distance threshold for determining if aircraft has reached target position.
 *
 * @unit m
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_AT_TGT_D, 2.0f);

/**
 * At target speed threshold
 *
 * Speed threshold for determining if aircraft has reached target position.
 *
 * @unit m/s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_AT_TGT_V, 0.5f);

/**
 * Maximum formation speed
 *
 * Maximum speed during formation switching maneuvers.
 *
 * @unit m/s
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_MAX_SPD, 6.0f);

/**
 * High speed mode threshold
 *
 * Speed above which high-speed avoidance mode is activated.
 *
 * @unit m/s
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_HI_SPD, 2.0f);

/**
 * Low speed mode threshold
 *
 * Speed below which low-speed avoidance mode is activated.
 *
 * @unit m/s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_LO_SPD, 1.0f);

/**
 * Formation following filter alpha
 *
 * Low-pass filter coefficient for smooth formation following.
 * Lower values = smoother but slower response.
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_FLT_FRM, 0.1f);

/**
 * Formation switching filter alpha
 *
 * Low-pass filter coefficient during formation switching.
 * Higher than normal for faster response during transitions.
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_FLT_SWT, 0.15f);

// ============== 从机碰撞检测阈值 ==============

/**
 * Follower collision detection distance
 *
 * Distance at which follower starts collision detection with other aircraft.
 *
 * @unit m
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_F_COL_D, 4.0f);

/**
 * Follower close distance threshold
 *
 * Close proximity threshold for follower collision avoidance.
 *
 * @unit m
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_F_CLS_D, 2.5f);

/**
 * Follower critical distance threshold
 *
 * Critical/emergency distance threshold for follower.
 *
 * @unit m
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_F_CRT_D, 2.5f);

/**
 * Follower warning distance threshold
 *
 * Warning distance threshold for follower collision detection.
 *
 * @unit m
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_F_WRN_D, 4.0f);

// ============== 主机碰撞检测阈值 ==============

/**
 * Leader collision detection distance
 *
 * Distance at which collision detection with leader aircraft starts.
 * Larger than follower for extra safety margin.
 *
 * @unit m
 * @min 2.0
 * @max 30.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_L_COL_D, 7.0f);

/**
 * Leader close distance threshold
 *
 * Close proximity threshold when near leader aircraft.
 *
 * @unit m
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_L_CLS_D, 5.0f);

/**
 * Leader critical distance threshold
 *
 * Critical/emergency distance threshold when near leader.
 *
 * @unit m
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_L_CRT_D, 4.5f);

/**
 * Leader warning distance threshold
 *
 * Warning distance threshold for leader collision detection.
 *
 * @unit m
 * @min 2.0
 * @max 20.0
 * @decimal 1
 * @group Swarm
 */
PARAM_DEFINE_FLOAT(SWARM_L_WRN_D, 6.0f);
