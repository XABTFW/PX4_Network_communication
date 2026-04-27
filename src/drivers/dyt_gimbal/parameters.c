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
