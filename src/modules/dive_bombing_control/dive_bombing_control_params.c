/**
 * @file dive_bombing_control_params.c
 * 俯冲轰炸控制参数定义
 */

/**
 * Initial hover altitude
 *
 * @unit m
 * @min 50.0
 * @max 200.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_INIT_ALT, 100.f);

/**
 * Release altitude
 *
 * @unit m
 * @min 10.0
 * @max 50.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_REL_ALT, 20.f);

/**
 * Dive angle
 *
 * @unit deg
 * @min 30.0
 * @max 70.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_DIVE_ANG, 60.f);

/**
 * Maximum velocity
 *
 * @unit m/s
 * @min 5.0
 * @max 20.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_MAX_VEL, 15.f);

/**
 * Pullup acceleration
 *
 * @unit m/s^2
 * @min 4.0
 * @max 12.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_PULLUP_ACC, 8.f);

/**
 * Drag coefficient
 *
 * @min 0.3
 * @max 1.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_DRAG_COEF, 0.6f);

/**
 * Vehicle mass
 *
 * @unit kg
 * @min 0.5
 * @max 5.0
 * @group Dive Bombing
 */
PARAM_DEFINE_FLOAT(DBC_MASS, 1.5f);
