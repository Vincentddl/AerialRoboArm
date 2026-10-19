#include "mod_startup_angle.h"
#include <math.h>
#include <string.h>

void ModStartupAngle_Reset(StartupAngleGate_t *gate)
{
    if (gate != NULL) memset(gate, 0, sizeof(*gate));
}

bool ModStartupAngle_Update(StartupAngleGate_t *gate, const FsusFeedback_t *fb,
                            bool valid, bool fresh, uint32_t now_ms)
{
    if (gate == NULL) return false;
    if (!valid || fb == NULL || !isfinite(fb->angle_deg) ||
        fb->angle_deg < ARA_MAIN_ARM_ANGLE_MIN_DEG || fb->angle_deg > ARA_MAIN_ARM_ANGLE_MAX_DEG ||
        fb->voltage_mv < 9000 || fb->voltage_mv > 12600 || (fb->status & 0xFEU)) {
        ModStartupAngle_Reset(gate);
        return false;
    }
    if (!fresh || (gate->active && fb->timestamp_ms == gate->last_sample_ms)) return false;
    float low = gate->active && gate->min_deg < fb->angle_deg ? gate->min_deg : fb->angle_deg;
    float high = gate->active && gate->max_deg > fb->angle_deg ? gate->max_deg : fb->angle_deg;
    if (!gate->active || high - low > STARTUP_ANGLE_SPREAD_DEG ||
        (uint32_t)(now_ms - gate->last_sample_ms) > 140U) {
        gate->active = true;
        gate->since_ms = now_ms;
        gate->samples = 0U;
        low = high = fb->angle_deg;
    }
    gate->min_deg = low;
    gate->max_deg = high;
    gate->last_sample_ms = fb->timestamp_ms;
    if (gate->samples < 65535U) gate->samples++;
    return gate->samples >= 3U && (uint32_t)(now_ms - gate->since_ms) >= STARTUP_ANGLE_SETTLE_MS;
}
