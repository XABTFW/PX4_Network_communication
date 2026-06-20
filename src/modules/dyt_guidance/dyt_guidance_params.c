/**
 * Terminal guidance activation AUX channel
 *
 * Enables DYT visual guidance to take over aircraft motion after seeker lock.
 * Midcourse geographic pointing follows the Cooperative Rendezvous activation
 * switch (CRDZ_ACT_AUX / CRDZ_ACT_BTN) when DYTG_COOP_EN is enabled.
 *
 * @value 0 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ACT_AUX, 1);

/**
 * Terminal guidance activation joystick button
 *
 * Enables DYT visual guidance to take over aircraft motion after seeker lock
 * using the MANUAL_CONTROL buttons bitmask. Button numbers match the zero-based
 * numbering shown by QGroundControl. Midcourse geographic pointing follows the
 * Cooperative Rendezvous activation button (CRDZ_ACT_BTN) when DYTG_COOP_EN is
 * enabled.
 *
 * @value -1 Disabled
 * @min -1
 * @max 15
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ACT_BTN, -1);

/**
 * Intercept AUX channel
 *
 * @value 0 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_INT_AUX, 2);

/**
 * Cooperative rendezvous handoff enable
 *
 * When enabled, the DYT seeker only commands aircraft motion (trajectory /
 * offboard setpoints) while the camera is locked and tracking. While searching
 * or after losing the lock, the seeker controls the gimbal only and leaves the
 * aircraft motion to the cooperative_rendezvous position-sharing follower, so
 * the two controllers never publish setpoints at the same time. The
 * Cooperative Rendezvous activation switch also enables midcourse geographic
 * pointing, so the payload can look at the shared target before terminal
 * guidance is authorized.
 *
 * Disable for standalone seeker operation (the seeker then holds position while
 * searching, as before).
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_COOP_EN, 0);

/**
 * Midcourse geographic tracking enable
 *
 * When enabled, midcourse pointing uses the DYT payload geographic tracking
 * protocol with ownship state and target latitude/longitude/altitude packets.
 * Disable to compute a frame-angle command in PX4 from the shared target
 * position instead. Disabling avoids sending near-vertical ownship Euler
 * attitudes for upward-looking payload installations.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_GEO_EN, 1);

/**
 * Midcourse target MAV_SYS_ID
 *
 * Target aircraft ID used to point the seeker before visual lock. Set to 0 to
 * use the newest valid remote follower_info sample that is not this vehicle.
 *
 * @min 0
 * @max 255
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_TGT_ID, 1);

/**
 * Midcourse target position timeout
 *
 * Maximum age of position-sharing target data used to point the seeker before
 * visual lock.
 *
 * @unit s
 * @min 0.1
 * @max 30.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_TGT_TO, 2.0f);

/**
 * Midcourse target altitude offset
 *
 * Offset added to follower_info.alt before sending the midcourse geographic
 * target to the DYT seeker. Keep at 0 for normal flight. Use this to compensate
 * known inter-aircraft absolute altitude bias during ground tests.
 *
 * @unit m
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_TGT_ALTOFF, 0.0f);

/**
 * Midcourse gimbal prediction time
 *
 * Prediction time used when PX4 computes midcourse gimbal frame-angle commands
 * from ownship and target positions. This compensates seeker/gimbal response
 * delay by pointing at the predicted line of sight. Set to 0 to disable.
 *
 * @unit s
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_PRED, 0.0f);

/**
 * Manual takeover stick threshold
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_STK_TK, 0.30f);

/**
 * Enable automatic guidance activation from detected target hints
 *
 * When enabled, the activation AUX/button/payload switch gates automatic
 * tracking: switch on, then guidance arms its tracking workflow after a target
 * lock candidate is visible for DYTG_AUTO_N consecutive frames.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_AUTO_EN, 0);

/**
 * Consecutive candidate frames for automatic guidance activation
 *
 * @min 1
 * @max 30
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_AUTO_N, 5);

/**
 * Consecutive lock frames to enter tracking
 *
 * @min 1
 * @max 20
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LOCK_N, 4);

/**
 * Consecutive lock frames to recover from lost hold
 *
 * @min 1
 * @max 20
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_RELOCKN, 3);

/**
 * Search wait timeout
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_WAITMS, 1500);

/**
 * Lost hold/search timeout
 *
 * @unit ms
 * @min 100
 * @max 600000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LOSTMS, 240000);

/**
 * Search yaw speed
 *
 * Gimbal yaw rate used while scanning after target loss.
 *
 * @unit deg/s
 * @min 1
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_YSPD, 3.f);

/**
 * Search pitch speed
 *
 * Gimbal pitch rate used while changing scan rows after target loss.
 *
 * @unit deg/s
 * @min 1
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_PSPD, 3.f);

/**
 * Search pitch step
 *
 * @unit deg
 * @min 1
 * @max 30
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_STEP, 20.f);

/**
 * Search edge pause
 *
 * @unit s
 * @min 0
 * @max 2
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_PAUSE, 0.05f);

/**
 * Search yaw step
 *
 * @unit deg
 * @min 5
 * @max 60
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_YSTP, 10.f);

/**
 * Search dwell time
 *
 * @unit s
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_DWEL, 0.2f);

/**
 * Gimbal center settle time before relock retry
 *
 * @unit ms
 * @min 0
 * @max 5000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_CTRMS, 400);

/**
 * Lost target relock retry interval
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_RTRYMS, 1000);

/**
 * Fixed pipeline delay
 *
 * @unit ms
 * @min 0
 * @max 1000
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_DLY_MS, 120.f);

/**
 * Maximum target age
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXAGE, 0.25f);

/**
 * Maximum allowed frame gap
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXJIT, 0.12f);

/**
 * Terminal handoff blend time
 *
 * Time used to blend from the previous midcourse velocity setpoint to the
 * visual guidance velocity setpoint after seeker lock. Set to 0 to disable the
 * blend.
 *
 * @unit s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_HOFF_T, 0.80f);

/**
 * Maximum delay allowed for intercept
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_INTDLY, 0.18f);

/**
 * Follow navigation gain
 *
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_N_FOL, 2.0f);

/**
 * Follow commanded speed
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_V_FOL, 2.0f);

/**
 * Follow LOS acceleration gain
 *
 * @unit m/s^2
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KA_FOL, 1.2f);

/**
 * Follow damping gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KV_FOL, 1.0f);

/**
 * Intercept navigation gain
 *
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_N_INT, 3.5f);

/**
 * Intercept commanded speed
 *
 * @unit m/s
 * @min 0.1
 * @max 25.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_V_INT, 6.0f);

/**
 * Intercept LOS acceleration gain
 *
 * @unit m/s^2
 * @min 0.0
 * @max 25.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KA_INT, 2.0f);

/**
 * Intercept damping gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KV_INT, 1.2f);

/**
 * Minimum closing speed proxy
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_VMIN, 1.0f);

/**
 * Maximum horizontal speed setpoint
 *
 * @unit m/s
 * @min 0.5
 * @max 30.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXV, 8.0f);

/**
 * Maximum acceleration feedforward
 *
 * @unit m/s^2
 * @min 0.5
 * @max 30.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXACC, 4.0f);

/**
 * Maximum yaw rate
 *
 * @unit deg/s
 * @min 5
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXYAWR, 30.f);

/**
 * Maximum yaw lag
 *
 * @unit deg
 * @min 1
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YAWLIM, 25.f);

/**
 * Maximum vertical speed magnitude
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXDZ, 1.0f);

/**
 * Vertical tracking scale
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ZSCALE, 0.25f);

/**
 * Enable vertical-priority XY scaling
 *
 * When enabled, horizontal tracking is reduced while the target is far above
 * or below the vehicle LOS. This helps avoid flying past the target projection
 * before vertical error has reduced.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ZXY_EN, 0);

/**
 * Minimum XY scale during vertical-priority tracking
 *
 * Horizontal velocity and horizontal feedforward acceleration are never scaled
 * below this fraction when vertical-priority XY scaling is enabled.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ZXY_MIN, 0.25f);

/**
 * Vertical LOS value for maximum XY reduction
 *
 * When vertical-priority XY scaling is enabled, XY reduction reaches
 * DYTG_ZXY_MIN once abs(los_ned[2]) reaches this value.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ZXY_FULL, 0.70f);

/**
 * Enable XY overshoot guard
 *
 * When enabled, horizontal tracking is reduced if the vehicle is moving away
 * from the target horizontal LOS while the target is still far above or below.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_XYOVR_EN, 0);

/**
 * Vertical LOS threshold for XY overshoot guard
 *
 * XY overshoot protection starts once abs(los_ned[2]) is above this value.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYOVR_Z, 0.75f);

/**
 * Reverse horizontal closing speed for full XY overshoot guard
 *
 * XY overshoot protection reaches DYTG_XYOVR_MIN once the vehicle is moving
 * away from the target horizontal LOS by this speed.
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYOVR_V, 2.0f);

/**
 * Minimum XY scale during XY overshoot guard
 *
 * Horizontal velocity and horizontal feedforward acceleration are never scaled
 * below this fraction when XY overshoot protection is active.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYOVR_MIN, 0.15f);

/**
 * Enable XY LOS turn-rate guard
 *
 * When enabled, horizontal tracking is reduced before overshoot if horizontal
 * LOS direction is rotating quickly while the target is still far above or
 * below the vehicle.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_XYROT_EN, 0);

/**
 * Horizontal LOS turn rate for full XY guard
 *
 * The turn-rate guard reaches DYTG_XYROT_MIN when the horizontal LOS direction
 * rotates at this rate.
 *
 * @unit rad/s
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYROT_W, 1.0f);

/**
 * Minimum XY scale during LOS turn-rate guard
 *
 * Horizontal velocity and horizontal feedforward acceleration are never scaled
 * below this fraction when the turn-rate guard is active.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYROT_MIN, 0.20f);

/**
 * Horizontal LOS deadband near vertical target
 *
 * @min 0.0
 * @max 0.8
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYDB, 0.20f);

/**
 * Horizontal LOS value for full XY tracking
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYFULL, 0.60f);

/**
 * Minimum horizontal LOS for yaw tracking
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YAWLOS, 0.20f);

/**
 * Horizontal velocity setpoint slew rate
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYSLEW, 3.0f);

/**
 * Front cone half-angle for intercept
 *
 * @unit deg
 * @min 5
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_FCONE, 35.f);

/**
 * LOS low-pass alpha
 *
 * @min 0.0
 * @max 0.99
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_LPF_A, 0.65f);

/**
 * Maximum LOS prediction horizon
 *
 * @unit s
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_PREDMAX, 0.20f);

/**
 * LOS X sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LXSIGN, 1);

/**
 * LOS Y sign
 *
 * The DYT V2.11 telemetry reports vertical miss angle as down-positive and up-negative,
 * which already matches PX4 NED Z sign.
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LYSIGN, 1);

/**
 * Gimbal roll sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_RSIGN, 1);

/**
 * Gimbal pitch sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_PSIGN, 1);

/**
 * Gimbal yaw sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_YSIGN, 1);

/**
 * Gimbal roll offset
 *
 * @unit deg
 * @min -180
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ROFF, 0.f);

/**
 * Gimbal pitch offset
 *
 * @unit deg
 * @min -180
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_POFF, 0.f);

/**
 * Gimbal yaw offset
 *
 * @unit deg
 * @min -180
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YOFF, 0.f);

/**
 * Midcourse mount rotation enable
 *
 * Enables a full Euler rotation from gimbal installation frame to aircraft body
 * frame before computing midcourse frame-angle commands from target position.
 * Keep disabled to use the legacy aircraft-body angle calculation.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_MNT_EN, 0);

/**
 * Midcourse mount roll
 *
 * Roll angle of the gimbal installation frame relative to the aircraft body
 * frame. Applied when DYTG_MNT_EN is set.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_R, 0.f);

/**
 * Midcourse mount pitch
 *
 * Pitch angle of the gimbal installation frame relative to the aircraft body
 * frame. For an installation where gimbal +X points along aircraft -Z, start
 * with +90 or -90 degrees and verify the direction on the ground.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_P, 0.f);

/**
 * Midcourse mount yaw
 *
 * Yaw angle of the gimbal installation frame relative to the aircraft body
 * frame. Applied when DYTG_MNT_EN is set.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_Y, 0.f);
