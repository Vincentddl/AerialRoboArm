#include "mod_startup_angle.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static StartupAngleGate_t gate;
static FsusFeedback_t fb;
static bool sample(uint32_t ms, float angle)
{
    fb.angle_deg = angle; fb.timestamp_ms = ms;
    return ModStartupAngle_Update(&gate, &fb, true, true, ms);
}
int main(void)
{
    fb.voltage_mv = 12000;
    assert(!sample(0, -86)); // a transient first reading never enables control
    assert(!sample(100, 1));
    assert(!sample(200, 1.1f));
    assert(!sample(300, 1));
    assert(sample(400, 1.1f));
    assert(!sample(400, 1.1f)); // duplicate sample cannot complete a window
    assert(!ModStartupAngle_Update(&gate, &fb, true, false, 420));
    assert(!sample(500, 20)); // moving arm starts a new window
    assert(!sample(600, 20.1f));
    assert(!sample(700, 20));
    assert(sample(800, 20.1f));
    fb.status = 9; assert(!sample(900, 20)); assert(!gate.active);
    fb.status = 0; fb.voltage_mv = 12620; assert(!sample(1000, 20));
    fb.voltage_mv = 12000; assert(!sample(1100, -91)); assert(!gate.active);
    assert(!sample(1200, 86)); assert(!sample(1300, NAN));
    assert(!sample(1400, 0)); assert(!sample(1600, 0)); // interrupted feedback
    assert(gate.since_ms == 1600);
    assert(!ModStartupAngle_Update(&gate, &fb, false, true, 1620));
    assert(!gate.active);
    assert(!sample(0xFFFFFF00U, 0));
    assert(!sample(0xFFFFFF64U, .1f));
    assert(!sample(0xFFFFFFC8U, .1f));
    assert(sample(44U, 0));
    puts("Startup angle tests passed: transient first frame, stable window, stale/fault/range rejection, wrap.");
    return 0;
}
