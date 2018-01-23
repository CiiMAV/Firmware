/**
 * Whether to scale throttle by battery power level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The fixed wing
 * should constantly behave as if it was fully charged with reduced max thrust
 * at lower battery percentages. i.e. if cruise speed is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group TBBV
 */
PARAM_DEFINE_INT32(TBBV_BAT_SCA_EN, 0);

PARAM_DEFINE_INT32(TBBV_SERVO_RE_1, 0);
PARAM_DEFINE_INT32(TBBV_SERVO_RE_2, 0);
PARAM_DEFINE_INT32(TBBV_SERVO_RE_3, 0);
PARAM_DEFINE_INT32(TBBV_SERVO_RE_4, 0);

PARAM_DEFINE_FLOAT(TBBV_KMX, 0.0f);
PARAM_DEFINE_FLOAT(TBBV_KMY, 0.0f);
PARAM_DEFINE_FLOAT(TBBV_KMZ, 0.0f);

PARAM_DEFINE_FLOAT(TBBV_ROLL_P, 0.0f);
PARAM_DEFINE_FLOAT(TBBV_PITCH_P, 0.0f);
PARAM_DEFINE_FLOAT(TBBV_YAW_P, 0.0f);

PARAM_DEFINE_INT32(TBBV_FEED_FX, 0);
PARAM_DEFINE_INT32(TBBV_FEED_FY, 0);

PARAM_DEFINE_FLOAT(TBBV_ATT_ROLL, 1.0f);
PARAM_DEFINE_FLOAT(TBBV_ATT_PITCH, 1.0f);
PARAM_DEFINE_FLOAT(TBBV_ATT_YAW, 1.0f);