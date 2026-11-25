#ifndef MOD_VOFA_MANUAL_H
#define MOD_VOFA_MANUAL_H

#include "task_arbiter.h"

/* Explicitly selected console authority, never a fabricated physical RC link. */
static inline bool ModVofaManual_Permitted(const RcControlData_t *physical, bool stopped)
{
    return !stopped && !(physical->is_link_up && physical->estop_state == ESTOP_ACTIVE);
}

static inline RcControlData_t ModVofaManual_Intent(const RcControlData_t *physical,
                                                  bool stopped, bool reset,
                                                  uint8_t grip)
{
    RcControlData_t out = {0};
    /* Only the arbiter's local input uses this permit; DataHub keeps real rc=0. */
    out.is_link_up = true;
    out.req_mode = ARA_MODE_MANUAL;
    out.estop_state = ModVofaManual_Permitted(physical, stopped) ? ESTOP_RELEASED : ESTOP_ACTIVE;
    out.sys_reset_pulse = reset;
    out.incremental_angle_deg = ARBITER_TARGET_ANGLE_HOLD;
    out.gripper_angle = grip;
    out.roll_degree = 90;
    return out;
}
#endif
