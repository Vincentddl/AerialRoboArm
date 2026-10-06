#include "mod_position_trim.h"
#include "drv_fsus.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_bias(float bias, uint32_t start)
{
    PositionTrim_t state;
    ModPositionTrim_Reset(&state);
    float sent = -50.0f;
    for (uint32_t dt = 0; dt < 10000; dt += 100) {
        uint32_t now = start + dt;
        sent = ModPositionTrim_Update(&state, -50.0f, sent + bias, now,
                                      true, true, now, -90.0f, 85.0f);
        assert(sent >= -90.0f && sent <= 85.0f);
        assert(fabsf(state.trim_deg) <= POSITION_TRIM_MAX_DEG + 0.001f);
    }
    assert(fabsf(sent + bias + 50.0f) <= 0.31f);
    printf("bias=%+.1f: final residual %.3f deg, trim %.3f deg\n", bias, sent + bias + 50.0f, state.trim_deg);
}

static PositionTrim_t trimmed_state(void)
{
    PositionTrim_t state;
    ModPositionTrim_Reset(&state);
    for (uint32_t now = 100; now <= 1000; now += 100)
        ModPositionTrim_Update(&state, -50, -49, now, true, true, now, -90, 85);
    assert(state.trim_deg < 0);
    return state;
}

static void test_safety_resets(void)
{
    PositionTrim_t state = trimmed_state();
    const float trim = state.trim_deg;
    /* One physical observation cannot become several correction steps. */
    for (uint32_t now = 1020; now <= 1120; now += 20)
        ModPositionTrim_Update(&state, -50, -49, 1000, true, true, now, -90, 85);
    assert(fabsf(state.trim_deg - trim) < 0.001f);
    ModPositionTrim_Update(&state, -50, -49, 1000, true, true, 1140, -90, 85);
    assert(state.trim_deg == 0);
    state = trimmed_state();
    ModPositionTrim_Update(&state, -45, -49, 1100, true, true, 1100, -90, 85);
    assert(state.trim_deg == 0);
    state = trimmed_state();
    ModPositionTrim_Update(&state, -50, -49, 1100, false, true, 1100, -90, 85);
    assert(state.trim_deg == 0);
    state = trimmed_state();
    ModPositionTrim_Update(&state, -50, -49, 1100, true, false, 1100, -90, 85);
    assert(state.trim_deg == 0);
    state = trimmed_state();
    ModPositionTrim_Update(&state, -50, -48, 1100, true, true, 1100, -90, 85);
    assert(state.blocked && state.trim_deg == 0);
}

static void test_limits_and_motion(void)
{
    PositionTrim_t s;
    ModPositionTrim_Reset(&s);
    for (uint32_t now = 0; now < 10000; now += 100) {
        float sent = ModPositionTrim_Update(&s, -50, -47.5f, now, true, true, now, -90, 85);
        assert(sent >= -51.501f);
    }
    assert(s.limited && fabsf(s.trim_deg + 1.5f) < 0.001f);
    ModPositionTrim_Reset(&s);
    for (uint32_t now = 0; now < 3000; now += 100)
        ModPositionTrim_Update(&s, -50, -46.8f, now, true, true, now, -90, 85);
    assert(s.trim_deg == 0); /* Too large to treat as a settled fine error. */
    for (int sign = -1; sign <= 1; sign += 2) {
        float goal = sign < 0 ? -90.f : 85.f;
        ModPositionTrim_Reset(&s);
        for (uint32_t now = 0; now < 3000; now += 100) {
            float sent = ModPositionTrim_Update(&s, goal, goal - sign, now, true, true, now, -90, 85);
            assert(sent == goal && s.trim_deg == 0);
        }
        assert(s.limited);
    }
    ModPositionTrim_Reset(&s);
    for (uint32_t now = 0; now < 3000; now += 100) {
        float goal = -50.f + (float)now / 1000.f;
        ModPositionTrim_Update(&s, goal, goal + 1.f, now, true, true, now, -90, 85);
        assert(s.trim_deg == 0); /* Moving target must not integrate. */
    }
}

static void test_wire_rounding(void)
{
    uint8_t packet[32];
    float angles[] = {-70.36f, -70.34f, 70.36f, 70.34f, -0.16f, 0.16f};
    int16_t expected[] = {-704, -703, 704, 703, -2, 2};
    for (unsigned i = 0; i < sizeof(angles) / sizeof(angles[0]); i++) {
        assert(DrvFsus_EncodeSetAngleByVelocity(packet, 0, angles[i], 80, 100, 100, 0) > 0);
        int16_t raw = (int16_t)((uint16_t)packet[5] | (uint16_t)packet[6] << 8);
        assert(raw == expected[i]);
    }
}

static void test_delayed_servo(void)
{
    PositionTrim_t state;
    ModPositionTrim_Reset(&state);
    float position = -48.8f;
    float sent = -50.f;
    for (uint32_t now = 0; now < 20000; now += 100) {
        /* Slower first-order internal position loop and +/-0.05 deg noise. */
        position += 0.15f * (sent + 1.2f - position);
        float measured = position + (((now / 100) % 2) ? .05f : -.05f);
        sent = ModPositionTrim_Update(&state, -50, measured, now,
                                      true, true, now, -90, 85);
        assert(!state.blocked && fabsf(state.trim_deg) <= 1.501f);
    }
    assert(fabsf(position + 50.f) <= .4f);
    printf("delayed/noisy servo: residual %.3f deg\n", position + 50.f);
}

int main(void)
{
    test_bias(1.2f, 0); test_bias(-1.2f, 0);
    test_bias(0.5f, UINT32_MAX - 800U);
    test_safety_resets(); test_limits_and_motion(); test_wire_rounding(); test_delayed_servo();
    puts("PASS: convergence, cached/stale feedback, faults, motion, limits, wraparound and protocol rounding");
    return 0;
}
