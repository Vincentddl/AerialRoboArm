#include "mod_home_zero.h"
#include <math.h>
#include <string.h>
void ModHomeZero_Reset(HomeZero_t *home) { memset(home, 0, sizeof(*home)); }
bool ModHomeZero_Update(HomeZero_t *home, const FsusFeedback_t *fb,
                        bool valid, bool fresh, uint32_t age_ms,
                        bool permitted, uint32_t now_ms)
{
    if (home == NULL || home->stage == HOME_ZERO_FAILED) return false;
    bool healthy = valid && fb != NULL && age_ms < 140U && isfinite(fb->angle_deg) &&
                   fb->angle_deg >= ARA_MAIN_ARM_ANGLE_MIN_DEG && fb->angle_deg <= ARA_MAIN_ARM_ANGLE_MAX_DEG &&
                   fb->voltage_mv >= 9000 && fb->voltage_mv <= 12600 && !(fb->status & 0xFEU);
    if (!healthy || !permitted) {
        if (home->stage == HOME_ZERO_MOVING || home->stage == HOME_ZERO_DONE) home->stage = HOME_ZERO_FAILED;
        return false;
    }
    if (home->stage == HOME_ZERO_DONE) return true;
    if (home->stage == HOME_ZERO_WAITING) {
        home->stage = HOME_ZERO_MOVING;
        home->started_ms = now_ms;
    }
    if ((uint32_t)(now_ms - home->started_ms) >= HOME_ZERO_TIMEOUT_MS) {
        home->stage = HOME_ZERO_FAILED;
        return false;
    }
    if (fresh && (!home->have_sample || fb->timestamp_ms != home->sample_ms)) {
        bool quiet = !home->have_sample || fabsf(fb->angle_deg - home->previous_deg) <= 0.25f;
        if (fabsf(fb->angle_deg - ARA_MAIN_ARM_DEFAULT_ANGLE_DEG) <= HOME_ZERO_TOLERANCE_DEG && quiet) {
            if (home->arrived_samples < 3U) home->arrived_samples++;
        } else home->arrived_samples = 0U;
        home->have_sample = true;
        home->sample_ms = fb->timestamp_ms;
        home->previous_deg = fb->angle_deg;
        if (home->arrived_samples >= 3U) home->stage = HOME_ZERO_DONE;
    }
    return true;
}
