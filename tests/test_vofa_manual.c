#include "mod_vofa_manual.h"
#include "mod_home_zero.h"
#include "mod_startup_angle.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    RcControlData_t physical = {0};
    physical.req_mode = ARA_MODE_AUTO;
    physical.estop_state = ESTOP_ACTIVE; /* Invalid offline default must not block console. */
    RcControlData_t console = ModVofaManual_Intent(&physical, false, false, 120);
    VisionIntent_t vision = {.target_present=true, .confidence=100, .target_angle_deg=-70};
    ArbiterInput_t input = {.rc=&console, .vision=&vision, .servo_online=true};
    ArbiterOutput_t out;
    TaskArbiter_Decide(&input, &out);
    assert(!physical.is_link_up && physical.req_mode == ARA_MODE_AUTO);
    assert(out.mode == ARA_MODE_MANUAL && out.torque_request);
    assert(out.target_angle_deg == ARBITER_TARGET_ANGLE_HOLD && out.gripper_angle == 120);
    assert(ModVofaManual_Permitted(&physical, false));
    HomeZero_t home; ModHomeZero_Reset(&home);
    FsusFeedback_t fb = {.voltage_mv=12000, .angle_deg=0};
    for (unsigned i=1; i<=3; i++) {
        fb.timestamp_ms=i*100;
        assert(ModHomeZero_Update(&home,&fb,true,true,0,ModVofaManual_Permitted(&physical,false),i*100));
    }
    assert(home.stage == HOME_ZERO_DONE);
    console = ModVofaManual_Intent(&physical, true, false, 120);
    TaskArbiter_Decide(&input,&out);
    assert(out.estop && !out.torque_request && out.fault_latched_next);
    assert(!ModVofaManual_Permitted(&physical,true));
    input.fault_latched=true;
    console = ModVofaManual_Intent(&physical,false,false,120);
    TaskArbiter_Decide(&input,&out); assert(!out.torque_request);
    console = ModVofaManual_Intent(&physical,false,true,120);
    TaskArbiter_Decide(&input,&out); assert(out.fault_reset_consumed && !out.torque_request);
    input.fault_latched=false; input.servo_stall_active=true;
    TaskArbiter_Decide(&input,&out); assert(!out.torque_request && out.reason_code==ARB_REASON_SERVO_STALL);
    input.servo_stall_active=false; input.servo_online=false;
    TaskArbiter_Decide(&input,&out); assert(!out.torque_request);
    physical.is_link_up=true; physical.estop_state=ESTOP_ACTIVE;
    assert(!ModVofaManual_Permitted(&physical,false));
    console=ModVofaManual_Intent(&physical,false,true,120); input.servo_online=true;
    TaskArbiter_Decide(&input,&out); assert(out.estop && !out.torque_request);
    StartupAngleGate_t gate; ModStartupAngle_Reset(&gate);
    fb.status=0x08;
    assert(!ModStartupAngle_Update(&gate,&fb,true,true,1000));
    ModHomeZero_Reset(&home);
    assert(!ModHomeZero_Update(&home,&fb,true,true,0,true,1000));
    fb.status=0; fb.voltage_mv=8500;
    assert(!ModStartupAngle_Update(&gate,&fb,true,true,1100));
    puts("VOFA offline manual/home, vision isolation, stop/reset, real E-stop and HX8 protections passed.");
    return 0;
}
