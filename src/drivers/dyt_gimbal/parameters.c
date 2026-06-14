/**
 * DYT serial baudrate
 *
 * @min 9600
 * @max 921600
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_BAUD, 115200);

/**
 * DYT telemetry timeout
 *
 * @unit ms
 * @min 50
 * @max 5000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_TO_MS, 300);

/**
 * DYT port reopen retry interval
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_RTRY_MS, 1000);

/**
 * DYT debug log period
 *
 * Set to 0 to disable periodic decoded-frame logs. When greater than 0,
 * the driver prints one parsed target sample to the system log every N ms,
 * which can be read from the NSH `dmesg` command.
 *
 * @unit ms
 * @min 0
 * @max 5000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_LOG_MS, 0);

/**
 * DYT raw frame logging
 *
 * Set to 1 to print the original telemetry/reply frame bytes in hex.
 * Telemetry hex logs follow the same throttle as DYT_LOG_MS, while command
 * replies are printed whenever they are received.
 *
 * @boolean
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_RAWLOG, 0);

/**
 * DYT startup home command enable
 *
 * Sends frame-angle commands after the serial link opens so the gimbal moves
 * from its power-on centered position to the configured startup view.
 *
 * @boolean
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_EN, 1);

/**
 * DYT startup home command delay
 *
 * Delay after opening the serial link before sending the startup frame-angle
 * command. This gives the gimbal time to finish its own power-on centering.
 *
 * @unit ms
 * @min 0
 * @max 30000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_DLY, 3000);

/**
 * DYT startup home command duration
 *
 * Duration over which the startup frame-angle command is resent. Repeating the
 * command helps if the gimbal is still completing its own power-on sequence.
 *
 * @unit ms
 * @min 0
 * @max 30000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_DUR, 10000);

/**
 * DYT startup home command interval
 *
 * Interval between repeated startup frame-angle commands.
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_INT, 500);

/**
 * DYT startup home yaw angle
 *
 * Frame yaw angle sent by the startup home command.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_HOME_YAW, 0.f);

/**
 * DYT startup home pitch angle
 *
 * Frame pitch angle sent by the startup home command. Positive pitch is up for
 * the DYT frame-angle command.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_HOME_PIT, 90.f);
