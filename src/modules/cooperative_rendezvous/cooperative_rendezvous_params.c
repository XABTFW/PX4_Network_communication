/**
 * Activation AUX channel
 *
 * Controls whether the rendezvous aircraft may publish Offboard setpoints.
 * Position broadcast remains enabled while the module is running.
 *
 * @value 0 Always enabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_ACT_AUX, 3);

/**
 * Activation joystick button
 *
 * Controls whether the rendezvous aircraft may publish Offboard setpoints using
 * the MANUAL_CONTROL buttons bitmask. Button numbers match the zero-based
 * numbering shown by QGroundControl.
 *
 * @value -1 Disabled
 * @min -1
 * @max 15
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_ACT_BTN, -1);

/**
 * Horizontal distance from target
 *
 * Desired horizontal distance for the rendezvous aircraft relative to the
 * target aircraft. Set to 0 to track the target position without a horizontal
 * offset. Negative values keep the startup offset from -d/-x/-y.
 *
 * @unit m
 * @min -1
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_DIST, 0.f);

/**
 * Enable explicit horizontal offsets
 *
 * When enabled, CRDZ_X_OFF and CRDZ_Y_OFF directly define the target-relative
 * NED horizontal offset. CRDZ_DIST is ignored for horizontal offset generation.
 * Keep disabled to use the legacy CRDZ_DIST scaling of the startup -x/-y
 * offset direction.
 *
 * @boolean
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_XY_OFF_EN, 0);

/**
 * Target NED X offset
 *
 * Exact NED X offset from the target aircraft used when CRDZ_XY_OFF_EN is set.
 * Positive is North in the local NED frame.
 *
 * @unit m
 * @min -100
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_X_OFF, -5.f);

/**
 * Target NED Y offset
 *
 * Exact NED Y offset from the target aircraft used when CRDZ_XY_OFF_EN is set.
 * Positive is East in the local NED frame.
 *
 * @unit m
 * @min -100
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_Y_OFF, 0.f);

/**
 * Approach speed toward target position
 *
 * Horizontal closing speed added while the rendezvous aircraft has not reached
 * the desired target-relative position. Keep this above the target aircraft
 * speed when the follower needs to catch up.
 *
 * Set to 0 to use the startup -v argument.
 *
 * @unit m/s
 * @min 0
 * @max 80
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_APP_SPD, 40.f);

/**
 * Slowdown radius near target position
 *
 * The approach speed stays at CRDZ_APP_SPD while horizontal position error is
 * larger than this radius, then scales down linearly to avoid overshoot near
 * the desired target-relative position.
 *
 * @unit m
 * @min 0.5
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_SLOW_RAD, 5.f);

/**
 * Horizontal velocity slew limit
 *
 * Limits how quickly the rendezvous horizontal velocity setpoint may change.
 * Set to 0 to disable the limit and keep the legacy direct target-velocity
 * feed-forward behavior.
 *
 * @unit m/s^2
 * @min 0
 * @max 20
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_VSLEW, 0.f);

/**
 * Target position low-pass time constant
 *
 * Applies a first-order low-pass filter to the target aircraft local NED
 * position before generating the rendezvous position and velocity setpoints.
 * This helps reject telemetry jitter that would otherwise make the follower
 * accelerate and decelerate repeatedly.
 *
 * Set to 0 to disable position filtering.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TPOS_TC, 0.f);

/**
 * Target velocity low-pass time constant
 *
 * Applies a first-order low-pass filter to the target aircraft NED velocity
 * feed-forward before it is added to the rendezvous approach velocity.
 *
 * Set to 0 to disable velocity filtering.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TVEL_TC, 0.f);

/**
 * Target position jump limit
 *
 * Limits one accepted target-position measurement jump before the low-pass
 * filter. This is useful when the target telemetry occasionally reports a
 * bad GPS/global position sample.
 *
 * Set to 0 to disable jump limiting.
 *
 * @unit m
 * @min 0
 * @max 200
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TPOS_JMP, 0.f);

/**
 * Altitude difference from target
 *
 * Desired altitude difference for the rendezvous aircraft relative to the
 * target aircraft. A positive value keeps the rendezvous aircraft above the
 * target aircraft.
 *
 * @unit m
 * @min -50
 * @max 50
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_ALT_DIFF, 0.f);
