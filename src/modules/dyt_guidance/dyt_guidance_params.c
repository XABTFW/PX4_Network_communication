/**
 * Activation AUX channel
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
 * Manual takeover stick threshold
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_STK_TK, 0.30f);

/**
 * Consecutive lock frames to enter tracking
 *
 * @min 1
 * @max 20
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LOCK_N, 1);

/**
 * Consecutive lock frames to recover from lost hold
 *
 * @min 1
 * @max 20
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_RELOCKN, 1);

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
PARAM_DEFINE_FLOAT(DYTG_SC_PSPD, 8.f);

/**
 * Search pitch step
 *
 * @unit deg
 * @min 1
 * @max 30
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_STEP, 15.f);

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
PARAM_DEFINE_FLOAT(DYTG_V_FOL, 3.0f);

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
PARAM_DEFINE_FLOAT(DYTG_MAXYAWR, 60.f);

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
PARAM_DEFINE_FLOAT(DYTG_MAXDZ, 1.5f);

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
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LYSIGN, -1);

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
