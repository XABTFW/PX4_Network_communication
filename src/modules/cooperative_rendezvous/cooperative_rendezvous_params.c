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
 * target aircraft. Negative values keep the startup offset from -d/-x/-y.
 *
 * @unit m
 * @min -1
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_DIST, -1.f);

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
