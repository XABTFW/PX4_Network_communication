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
