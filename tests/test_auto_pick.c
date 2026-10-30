#include "task_manipulator.h"
#include "drv_hc13.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t grip;
static bool fail_gripper;
int8_t ModActuator_Init(ModActuator_Context_t *ctx) { (void)ctx; return 0; }
int8_t ModActuator_SetRoll(ModActuator_Context_t *ctx, uint8_t value)
{ (void)ctx; (void)value; return 0; }
int8_t ModActuator_SetGripperAngle(ModActuator_Context_t *ctx, uint8_t value)
{ (void)ctx; if (fail_gripper) return -1; grip = value; return 0; }

static ArbiterOutput_t arb;
static MotionState_t mot;
static MotionCmd_t cmd;
static void tick(uint32_t ms) { TaskManipulator_Update(&arb, &mot, ms, &cmd); }
static void setup(void)
{
    memset(&arb, 0, sizeof arb); memset(&mot, 0, sizeof mot);
    memset(&cmd, 0, sizeof cmd); fail_gripper = false;
    arb.mode = ARA_MODE_AUTO; arb.torque_request = true;
    arb.reason_code = ARB_REASON_AUTO_VISION_FRESH;
    arb.target_angle_deg = -60; arb.roll_degree = 90;
    mot.servo_online = mot.feedback_valid = mot.feedback_fresh = true;
    mot.feedback.voltage_mv = 12000; mot.feedback.angle_deg = -20;
    mot.feedback.status = 1; mot.is_moving = true;
    TaskManipulator_Init(); TaskManipulator_SeedCurrentAngle(-20);
}
int main(void)
{
    /* Exercise the external speed=0 contract and its bounds in both schemes. */
    setup(); arb.target_speed=0; tick(1000);
    assert(cmd.velocity_deg_per_s == 500.0f);
    assert(cmd.t_acc_ms == 100U && cmd.t_dec_ms == 100U);
    arb.target_speed=5; tick(1020);
    assert(cmd.velocity_deg_per_s == 20.0f);
    arb.target_speed=350; tick(1030);
    assert(cmd.velocity_deg_per_s == 350.0f);
    arb.target_speed=65000; tick(1040);
    assert(cmd.velocity_deg_per_s == 500.0f);
    arb.estop=true; tick(1060); assert(!cmd.torque_on);
#if !TASK_MANIP_AUTO_GRIP_THREE_FRAME
    RcControlData_t rc = {0}; VisionIntent_t vision = {0};
    rc.is_link_up = true; rc.estop_state = ESTOP_RELEASED;
    rc.req_mode = ARA_MODE_AUTO; rc.gripper_angle = 90;
    vision.target_present = true; vision.confidence = 90;
    vision.target_angle_deg = -60; vision.last_update_tick_ms = 1000;
    ArbiterInput_t input = {.rc=&rc, .vision=&vision, .servo_online=true, .tick_ms=1000};
    setup(); TaskArbiter_Decide(&input, &arb); tick(1000);
    assert(grip == 130 && cmd.target_angle_deg == -60);
    for (uint32_t i=1; i<=6; i++) {
        arb.vision_frame_id=i; arb.vision_stable_frames=3;
        arb.target_angle_deg=-40; tick(1000+i*20);
        assert(grip == 130 && cmd.target_angle_deg == -40);
        assert(TaskManipulator_GetGripStableFrames() == 0);
    }
    rc.gripper_angle=135; TaskArbiter_Decide(&input, &arb); tick(1140);
    assert(grip == 135);
    rc.gripper_angle=90; TaskArbiter_Decide(&input, &arb); tick(1160);
    assert(grip == 90);
    vision.target_present=false; rc.gripper_angle=135;
    TaskArbiter_Decide(&input, &arb); tick(1180);
    assert(grip == 135 && cmd.target_angle_deg == -60);
    arb.gripper_override_enabled=true; arb.gripper_angle=120; tick(1200);
    assert(grip == 120);
    arb.mode=ARA_MODE_MANUAL; tick(1210);
    arb.mode=ARA_MODE_AUTO; arb.gripper_override_enabled=false; arb.gripper_angle=90;
    tick(1215); assert(grip == 130);
    arb.estop=true; tick(1220); assert(!cmd.torque_on);
    puts("AUTO follow with manual CH4 grip passed: entry, stable3 ignored, stale vision, override, estop.");
#else
    setup(); tick(1000);
    assert(grip == 130 && cmd.target_angle_deg == -60); // first update, no opening delay
    arb.target_angle_deg = -39; tick(1020);
    assert(cmd.target_angle_deg == -39 && grip == 130);
    mot.feedback.angle_deg = -39; tick(1040);
    assert(TaskManipulator_GetState() == MANIP_STATE_AUTO_HOLD);
    arb.target_angle_deg = -70; tick(1060);
    assert(cmd.target_angle_deg == -70 && grip == 130); // updates after arrival
    arb.reason_code = ARB_REASON_AUTO_VISION_STALE;
    arb.target_angle_deg = ARBITER_TARGET_ANGLE_HOLD; tick(1080);
    assert(cmd.target_angle_deg == -70 && grip == 130);
    arb.reason_code = ARB_REASON_AUTO_VISION_FRESH; arb.target_angle_deg = -50;
    tick(1100); assert(cmd.target_angle_deg == -50);
    mot.feedback_age_ms = 140; arb.target_angle_deg = -40;
    tick(1120); assert(cmd.target_angle_deg == -50);
    mot.feedback_age_ms = 0; mot.feedback.status = 9;
    tick(1140); assert(cmd.target_angle_deg == -50);
    mot.feedback.status = 1; mot.feedback.voltage_mv = 8000;
    tick(1160); assert(cmd.target_angle_deg == -50);
    mot.feedback.voltage_mv = 12000; mot.encoder_jump_suspected = true;
    tick(1180); assert(cmd.target_angle_deg == -50);
    mot.encoder_jump_suspected = false; tick(1200);
    assert(cmd.target_angle_deg == -40);
    arb.gripper_override_enabled = true; arb.gripper_angle = 90;
    arb.target_angle_deg = -55; tick(1220);
    assert(grip == 90 && cmd.target_angle_deg == -55); // PTK override independent of tracking
    arb.gripper_override_enabled = false; tick(1240); assert(grip == 130);
    arb.target_angle_deg = -100; tick(1260); assert(cmd.target_angle_deg == -90);
    arb.target_angle_deg = 100; tick(1280); assert(cmd.target_angle_deg == 85);
    arb.target_angle_deg = NAN; tick(1300); assert(cmd.target_angle_deg == 85);
    arb.mode = ARA_MODE_ERROR; arb.estop = true; tick(1320);
    assert(!cmd.torque_on && TaskManipulator_GetAutoStage() == 0);
    arb.mode = ARA_MODE_AUTO; arb.estop = false; arb.target_angle_deg = -60;
    tick(1340); assert(cmd.target_angle_deg == -60 && grip == 130);
    arb.mode = ARA_MODE_MANUAL; arb.gripper_angle = 80; arb.target_angle_deg = -30;
    tick(1360); assert(grip == 80 && cmd.target_angle_deg == -30);
    arb.mode = ARA_MODE_AUTO; arb.reason_code = ARB_REASON_AUTO_VISION_STALE;
    mot.feedback.angle_deg = -31; tick(1380);
    assert(grip == 130 && cmd.target_angle_deg == -31);
    TaskManipulator_CancelAutoCycle();
    assert(TaskManipulator_GetAutoStage() == 0);
    setup(); arb.reason_code = ARB_REASON_AUTO_VISION_STALE;
    arb.target_angle_deg = ARBITER_TARGET_ANGLE_HOLD;
    mot.feedback.angle_deg = -87;
    TaskManipulator_SeedDefaultPosition(); tick(1700);
    assert(cmd.target_angle_deg == 0 && grip == 130); // default is a command, not encoder reseeding
    tick(1720); assert(cmd.target_angle_deg == 0);
    arb.reason_code = ARB_REASON_AUTO_VISION_FRESH; arb.target_angle_deg = -50;
    tick(1740); assert(cmd.target_angle_deg == -50); // vision still overrides default immediately

    setup(); arb.gripper_override_enabled = true; arb.gripper_angle = 90;
    tick(1800); assert(grip == 130); // AUTO entry has priority over an old closed override
    arb.gripper_override_enabled = false;
    arb.mode = ARA_MODE_MANUAL; tick(1820);
    arb.mode = ARA_MODE_AUTO; tick(1840); assert(grip == 130);

    setup(); arb.vision_frame_id = 100; arb.vision_stable_frames = 3;
    tick(2000); assert(grip == 130); // open on entry, even if PC was already stable
    tick(2020); tick(2040); assert(grip == 130); // cached frame cannot count
    arb.vision_frame_id = 101; arb.vision_stable_frames = 1; tick(2060);
    assert(grip == 130 && TaskManipulator_GetGripStableFrames() == 1);
    arb.vision_frame_id = 102; arb.vision_stable_frames = 2; tick(2080);
    assert(grip == 130 && TaskManipulator_GetGripStableFrames() == 2);
    arb.vision_frame_id = 103; arb.vision_stable_frames = 3; tick(2100);
    assert(grip == 90 && TaskManipulator_GetAutoStage() == 3);
    arb.target_angle_deg = -39; arb.vision_frame_id = 104; tick(2120);
    assert(cmd.target_angle_deg == -39 && grip == 90); // grip does not latch arm target
    arb.mode = ARA_MODE_MANUAL; arb.gripper_angle = 80; tick(2140);
    arb.mode = ARA_MODE_AUTO; tick(2160); assert(grip == 130);
    arb.vision_frame_id = 105; tick(2180); assert(grip == 130);
    arb.vision_frame_id = 106; tick(2200); assert(grip == 130);
    arb.vision_frame_id = 107; tick(2220); assert(grip == 90);

    setup(); tick(3000);
    arb.vision_frame_id = 1; arb.vision_stable_frames = 1; tick(3020);
    arb.vision_frame_id = 2; arb.vision_stable_frames = 2; tick(3040);
    arb.vision_frame_id = 3; arb.vision_stable_frames = 0; tick(3060);
    assert(TaskManipulator_GetGripStableFrames() == 0 && grip == 130);
    arb.vision_frame_id = 4; arb.vision_stable_frames = 1; tick(3080);
    arb.vision_frame_id = 5; arb.vision_stable_frames = 2; tick(3100);
    mot.feedback.status = 9; arb.vision_frame_id = 6; arb.vision_stable_frames = 3;
    tick(3120); assert(grip == 130 && TaskManipulator_GetGripStableFrames() == 0);
    mot.feedback.status = 1;
    for (uint32_t i = 7; i < 12; i++) {
        arb.vision_frame_id = i; arb.vision_stable_frames = 0; tick(3120 + i * 20);
        assert(grip == 130); // legacy packets cannot close the gripper
    }
    DrvHC13_Init(); HC13VisionSample_t sample;
    const char *packet = "V,-60,0,90,3\n";
    for (const char *p = packet; *p; p++) DrvHC13_PushRxByte((uint8_t)*p);
    assert(DrvHC13_PollVision(1000, &sample) && sample.stable_frames == 3);
    packet = "V,-50,0,90\n";
    for (const char *p = packet; *p; p++) DrvHC13_PushRxByte((uint8_t)*p);
    assert(DrvHC13_PollVision(1020, &sample) && sample.stable_frames == 0);
    packet = "V,-50,0,90,9\n";
    for (const char *p = packet; *p; p++) DrvHC13_PushRxByte((uint8_t)*p);
    assert(DrvHC13_PollVision(1040, &sample) && sample.stable_frames == 0);
    puts("AUTO tracking + three-frame grip tests passed: immediate target, continuous updates after arrival, stale/fault hold, manual override, limits.");
#endif
    return 0;
}
