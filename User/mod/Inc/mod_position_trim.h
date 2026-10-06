#ifndef MOD_POSITION_TRIM_H
#define MOD_POSITION_TRIM_H

#include <stdbool.h>
#include <stdint.h>

#ifndef POSITION_TRIM_SETTLE_MS
#define POSITION_TRIM_SETTLE_MS       (300U)
#endif
#ifndef POSITION_TRIM_INTERVAL_MS
#define POSITION_TRIM_INTERVAL_MS     (100U)
#endif
#ifndef POSITION_TRIM_QUIET_SAMPLES
#define POSITION_TRIM_QUIET_SAMPLES   (2U)
#endif
#define POSITION_TRIM_FEEDBACK_MAX_MS (140U)
#define POSITION_TRIM_DEADBAND_DEG    (0.30f)
#define POSITION_TRIM_MIN_STEP_DEG    (0.10f)
#ifndef POSITION_TRIM_MAX_STEP_DEG
#define POSITION_TRIM_MAX_STEP_DEG    (0.20f)
#endif
#define POSITION_TRIM_ERROR_GAIN      (0.35f)
#define POSITION_TRIM_MAX_DEG         (1.50f)
#define POSITION_TRIM_CAPTURE_DEG     (3.00f)

typedef struct {
    bool initialized;
    bool have_sample;
    bool limited;
    bool blocked;
    uint8_t quiet_samples;
    uint32_t goal_changed_ms;
    uint32_t sample_ms;
    uint32_t adjusted_ms;
    float goal_deg;
    float trim_deg;
    float previous_position_deg;
    float previous_error_deg;
} PositionTrim_t;

void ModPositionTrim_Reset(PositionTrim_t *state);
float ModPositionTrim_Update(PositionTrim_t *state, float requested_deg,
                             float position_deg, uint32_t feedback_ms,
                             bool feedback_healthy, bool torque_on,
                             uint32_t now_ms, float minimum_deg, float maximum_deg);

#endif
