#include "mod_position_trim.h"
#include <string.h>

static float absolute(float value) { return value < 0.0f ? -value : value; }
static float clamp(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

void ModPositionTrim_Reset(PositionTrim_t *state)
{
    memset(state, 0, sizeof(*state));
}

float ModPositionTrim_Update(PositionTrim_t *state, float requested_deg,
                             float position_deg, uint32_t feedback_ms,
                             bool feedback_healthy, bool torque_on,
                             uint32_t now_ms, float minimum_deg, float maximum_deg)
{
    const float goal = clamp(requested_deg, minimum_deg, maximum_deg);
    if (!torque_on || !feedback_healthy ||
        (uint32_t)(now_ms - feedback_ms) >= POSITION_TRIM_FEEDBACK_MAX_MS) {
        ModPositionTrim_Reset(state);
        return goal;
    }
    if (!state->initialized || absolute(goal - state->goal_deg) > 0.05f) {
        ModPositionTrim_Reset(state);
        state->initialized = true;
        state->goal_deg = goal;
        state->goal_changed_ms = now_ms;
        state->adjusted_ms = now_ms;
    }
    /* Cached feedback is never counted as another observation or integral step. */
    if (state->have_sample && feedback_ms == state->sample_ms) {
        return clamp(goal + state->trim_deg, minimum_deg, maximum_deg);
    }
    /* Once fine correction has begun, allow the small motion caused by our
     * own bounded step. This does not delay the normal target-following path. */
    const float quiet_delta = absolute(state->trim_deg) > 0.01f
                                  ? POSITION_TRIM_MAX_STEP_DEG + 0.05f : 0.15f;
    if (state->have_sample && absolute(position_deg - state->previous_position_deg) <= quiet_delta) {
        if (state->quiet_samples < POSITION_TRIM_QUIET_SAMPLES) state->quiet_samples++;
    } else {
        state->quiet_samples = 0U;
    }
    state->previous_position_deg = position_deg;
    state->sample_ms = feedback_ms;
    state->have_sample = true;
    const float error = goal - position_deg;
    const float error_abs = absolute(error);
    /* Stop this goal's correction if a previously applied trim makes error grow.
     * A new goal or a torque/link transition explicitly resets this block. */
    if (absolute(state->trim_deg) > 0.01f && error_abs > state->previous_error_deg + 0.50f) {
        state->blocked = true;
        state->trim_deg = 0.0f;
    }
    if (state->blocked || state->quiet_samples < POSITION_TRIM_QUIET_SAMPLES ||
        (uint32_t)(now_ms - state->goal_changed_ms) < POSITION_TRIM_SETTLE_MS ||
        (uint32_t)(now_ms - state->adjusted_ms) < POSITION_TRIM_INTERVAL_MS ||
        error_abs <= POSITION_TRIM_DEADBAND_DEG || error_abs > POSITION_TRIM_CAPTURE_DEG) {
        return clamp(goal + state->trim_deg, minimum_deg, maximum_deg);
    }
    const float step = clamp(POSITION_TRIM_ERROR_GAIN * error_abs,
                             POSITION_TRIM_MIN_STEP_DEG, POSITION_TRIM_MAX_STEP_DEG);
    const float proposed = state->trim_deg + (error > 0.0f ? step : -step);
    const float bounded = clamp(proposed, -POSITION_TRIM_MAX_DEG, POSITION_TRIM_MAX_DEG);
    const float sent = clamp(goal + bounded, minimum_deg, maximum_deg);
    /* Anti-windup: retain only correction that can actually fit the hard limits. */
    state->trim_deg = sent - goal;
    state->limited = absolute(proposed - state->trim_deg) > 0.01f;
    state->previous_error_deg = error_abs;
    state->adjusted_ms = now_ms;
    return sent;
}
