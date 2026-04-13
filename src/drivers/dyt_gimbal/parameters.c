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
