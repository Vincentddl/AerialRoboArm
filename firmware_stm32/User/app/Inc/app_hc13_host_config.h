#ifndef APP_HC13_HOST_CONFIG_H
#define APP_HC13_HOST_CONFIG_H
/* Existing arm convention: home/normal = encoder 0, forward = negative.
 * Fold geometry is NOT inferred from photographs or an arbitrary endpoint. */
#define HOST_VERTICAL_MDEG 0L
#define HOST_HOME_MDEG HOST_VERTICAL_MDEG
#define HOST_NORMAL_MDEG HOST_VERTICAL_MDEG
#define HOST_PC_WORKING_MDEG 90000L
#define HOST_PC_FOLDED_MDEG 160000L
#define HOST_EXTENSION_SIGN (-1)
#define HOST_MAX_EXTENSION_MDEG 90000UL
#define HOST_FOLD_CALIBRATED 0
#define HOST_FOLD_MDEG 0L
/* V2 PC contract; command presets are not measured position feedback. */
#define HOST_GRIP_OPEN_MDEG 125000L
#define HOST_GRIP_CLOSE_MDEG 90000L
#define HOST_GRIP_CONFIGURED 1
#define HOST_VISION_MIN_CONFIDENCE 60U
/* -1 means unmeasured: publish NA and reject dependent automated actions.
 * Do not invent a delay or confuse angular lead with servo travel time. */
#define HOST_OPEN_WAIT_MS (-1)
#define HOST_CLOSE_WAIT_MS (-1)
#define HOST_DEFAULT_LEAD_MDEG 4000U
/* PICK may settle slightly short of its target under load. Completion still
 * requires fresh, quiet HX8 feedback for at least 200 ms and a closed claw. */
#define HOST_PICK_SETTLE_TOLERANCE_MDEG 2500L
_Static_assert(HOST_OPEN_WAIT_MS>=-1 && HOST_OPEN_WAIT_MS<=60000,"Invalid open wait calibration");
_Static_assert(HOST_CLOSE_WAIT_MS>=-1 && HOST_CLOSE_WAIT_MS<=60000,"Invalid close wait calibration");
_Static_assert(HOST_DEFAULT_LEAD_MDEG>=3000 && HOST_DEFAULT_LEAD_MDEG<=5000 && HOST_DEFAULT_LEAD_MDEG%100==0,"Invalid gripper lead");
_Static_assert(HOST_PICK_SETTLE_TOLERANCE_MDEG>0 && HOST_PICK_SETTLE_TOLERANCE_MDEG<=3000,"Invalid PICK settle tolerance");
#endif
