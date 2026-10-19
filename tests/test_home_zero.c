#include "mod_home_zero.h"
#include <assert.h>
#include <stdio.h>
static HomeZero_t home;
static FsusFeedback_t fb;
static bool sample(uint32_t ms, float pos, bool permitted)
{
    fb.angle_deg = pos; fb.timestamp_ms = ms;
    return ModHomeZero_Update(&home, &fb, true, true, 0, permitted, ms);
}
int main(void)
{
    fb.voltage_mv = 12000; ModHomeZero_Reset(&home);
    assert(!sample(0,-85,false) && home.stage==HOME_ZERO_WAITING);
    assert(sample(100,-85,true) && home.stage==HOME_ZERO_MOVING);
    assert(sample(200,-70,true)); assert(sample(300,-20,true));
    assert(sample(400,.3f,true) && home.arrived_samples==0); // still travelling
    assert(sample(500,.2f,true) && home.arrived_samples==1);
    assert(ModHomeZero_Update(&home,&fb,true,false,20,true,520));
    assert(home.arrived_samples==1); // cache never confirms arrival
    assert(sample(600,.1f,true) && home.stage==HOME_ZERO_MOVING);
    assert(sample(700,.1f,true) && home.stage==HOME_ZERO_DONE);
    ModHomeZero_Reset(&home); assert(sample(1000,-60,true));
    assert(!sample(1100,-50,false) && home.stage==HOME_ZERO_FAILED);
    assert(!sample(1200,0,true)); // fault cannot restart automatically
    ModHomeZero_Reset(&home); assert(sample(2000,-80,true)); fb.status=9;
    assert(!sample(2100,-70,true) && home.stage==HOME_ZERO_FAILED); fb.status=0;
    ModHomeZero_Reset(&home); assert(sample(3000,-80,true));
    assert(!ModHomeZero_Update(&home,&fb,true,false,140,true,3140));
    assert(home.stage==HOME_ZERO_FAILED);
    ModHomeZero_Reset(&home); assert(sample(4000,-80,true));
    assert(!sample(9000,-80,true) && home.stage==HOME_ZERO_FAILED);
    ModHomeZero_Reset(&home); assert(sample(0xFFFFFF00U,-80,true));
    assert(sample(44U,-70,true));
    puts("Explicit home-zero tests passed: motion, stable arrival, cache, faults, timeout, wrap.");
    return 0;
}
