#include "mod_hc13_host.h"
#include "app_hc13_host_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static Hc13Host_t ctx;static HostSnapshot_t world;static uint32_t now;
static unsigned moves,follow_moves,stops,servos,records;static int32_t last_servo,last_follow_target;
static uint16_t last_follow_speed;
static bool transport_ready,emit_stdout;static HostError_t move_error,stop_error,servo_error;
static char output[64000];static size_t output_used;
static void snapshot(HostSnapshot_t *s,uint32_t t) { (void)t;*s=world; }
static HostError_t move(int32_t target,uint16_t speed,HostVerb_t kind,uint32_t t)
{
    assert(speed>=1 && speed<=500);assert(kind!=HOST_VISION);moves++;
    if (move_error) return move_error;
    world.target_valid=true;world.target_mdeg=target;world.motion_start_ms=t;world.phase=1;return HOST_OK;
}
static uint16_t last_ramp;
static HostError_t follow_move(int32_t target,uint16_t speed,uint32_t t)
{
    (void)t;assert(speed>=1 && speed<=500);
    if (move_error) return move_error;
    follow_moves++;last_follow_target=target;last_follow_speed=speed;
    world.target_valid=true;world.target_mdeg=target;return HOST_OK;
}
static HostError_t experiment_move(int32_t target,uint16_t speed,uint16_t ramp_ms,uint32_t t)
{ last_ramp=ramp_ms;return move(target,speed,HOST_MOVE,t); }
static uint32_t captured_id;
static unsigned capture_starts, capture_reads;
static void capture_begin(uint32_t id)
{ captured_id=id;capture_starts++; }
static bool capture_read(uint32_t id,uint16_t page,HostCapturePage_t *data)
{
    if (id!=captured_id || page) return false;
    capture_reads++;
    *data=(HostCapturePage_t){.start_ms=1000,.end_ms=1500,.acc_count=1,.length=8};
    return true;
}
static HostError_t stop(uint32_t t)
{
    stops++;
    if (world.phase) { world.phase=0;ModHc13Host_MotionFinished(&ctx,HOST_STOPPED,t,world.pos_mdeg); }
    return stop_error;
}
static HostError_t servo(int32_t v) { assert(v>=0 && v<=180000);servos++;last_servo=v;return servo_error; }
static HostError_t record(bool on) { records++;world.record_on=on;return HOST_OK; }
static bool send(const char *s,size_t n)
{
    if (!transport_ready) return false;
    assert(n<=HOST_LINE_MAX+1 && s[n-1]=='\n');assert(output_used+n+1<sizeof(output));
    memcpy(output+output_used,s,n);output_used+=n;output[output_used]=0;
    if (emit_stdout) { fwrite(s,1,n,stdout);fflush(stdout); }
    return true;
}
static void reset(bool calibrated)
{
    memset(&world,0,sizeof(world));world.hx_online=world.hx_healthy=world.ready=world.recovery_ready=true;
    world.imu_online=world.imu_ready=true;world.voltage_mv=11980;world.imu_hz=200;world.feedback_seq=1;
    world.acc_count=1;world.acc_ms=1000;world.acc_mg[0]=85;world.acc_mg[1]=-20;world.acc_mg[2]=998;
    now=1000;moves=follow_moves=stops=servos=records=0;last_servo=-1;last_follow_target=0;last_follow_speed=0;output_used=0;output[0]=0;
    move_error=stop_error=servo_error=HOST_OK;transport_ready=true;
    HostConfig_t cfg={.vertical_mdeg=HOST_VERTICAL_MDEG,.grip_open_mdeg=HOST_GRIP_OPEN_MDEG,
        .grip_close_mdeg=HOST_GRIP_CLOSE_MDEG,.grip_calibrated=true,.extension_sign=-1,
        .pc_working_mdeg=HOST_PC_WORKING_MDEG,.pc_folded_mdeg=HOST_PC_FOLDED_MDEG,
        .max_extension_mdeg=90000,.max_speed=500,.ramp_ms=100,.post_ms=1000,.min_confidence=60,
        .open_wait_calibrated=calibrated,.close_wait_calibrated=calibrated,
        .open_wait_ms=100,.close_wait_ms=150,.default_lead_mdeg=4000};
    HostHooks_t hooks={.snapshot=snapshot,.move=move,.follow_move=follow_move,.experiment_move=experiment_move,
        .stop=stop,.servo=servo,.record=record,.send=send};
    ModHc13Host_Init(&ctx,&cfg,&hooks,12345,now);
}
static void drain(void) { for (unsigned i=0;i<24;i++) ModHc13Host_Poll(&ctx,now); }
static void input(const char *s) { ModHc13Host_Receive(&ctx,(const uint8_t *)s,strlen(s),now);drain(); }
static void tick(uint32_t delta) { now+=delta;drain(); }
static void feedback(int32_t pos) { world.pos_mdeg=pos;world.feedback_seq++;drain(); }
static void arrive(int32_t pos)
{
    world.pos_mdeg=pos;world.feedback_seq++;world.phase=2;ModHc13Host_MotionArrived(&ctx,now,pos);drain();
}
static void done(int32_t pos)
{
    world.pos_mdeg=pos;world.feedback_seq++;world.phase=0;ModHc13Host_MotionFinished(&ctx,HOST_OK,now,pos);drain();
}
static unsigned occurrences(const char *s,const char *needle)
{ unsigned count=0;size_t n=strlen(needle);while ((s=strstr(s,needle))) { count++;s+=n; }return count; }
static void prepare(unsigned id,const char *verb)
{
    char text[64];
    if (!strcmp(verb,"SHAPE_NORMAL")) snprintf(text,sizeof(text),"SHAPE,%u,NORMAL\n",id);
    else snprintf(text,sizeof(text),"%s,%u\n",verb,id);
    input(text);
    assert(!ctx.prepared);tick(300);arrive(0);done(0);tick(100);
    assert(ctx.prepared && !ctx.active && last_servo==HOST_GRIP_OPEN_MDEG);
}
static void parser_tests(void)
{
    HostCommand_t c;
    const char *good[]={"MOVE,1,-30000,250","PICK,2,-30000,500,4000","PREP,3","TRIGGER,4",
        "HOME,5","STOP,6","MODE,7,VISION_PICK","MODE,19,VISION_FOLLOW","SHAPE,8,NORMAL","GRIP,9,OPEN","SERVO,10,1,120000",
        "RECORD,11,1","GET,12,STATE","GET,13,INFO","GET,14,CAL","GET,16,EXPERIMENT","GET,18,IMU",
        "PING,15","V,-60000,500,95","MOVE,4294967295,85000,500","XMOVE,17,20000,350,100"};
    for (unsigned i=0;i<sizeof(good)/sizeof(good[0]);i++) assert(ModHc13Host_Parse(good[i],strlen(good[i]),&c)==HOST_OK);
    const char *bad[]={"PICK,1,-30000,500,2999","PICK,1,-30000,500,5001","PICK,1,-30000,500,4050",
        "PICK,1,-30000,0,4000","MOVE,1,-30000","MOVE,1,-30000,250,EXTRA","MOVE,1,-90001,500",
        "MOVE,1,85001,500","MOVE,1,-30101,500","MOVE,4294967296,0,100","MOVE,0,0,100","MOVE,1,abc,100",
        "MODE,1,AUTO","GRIP,1,UNKNOWN","SERVO,1,2,90000","SERVO,1,1,90500","RECORD,1,2",
        "V,-60000,500,101","V,-60000,0,90","V,-60000,500,95,3","MOVE,1,,100","MOVE,1,0, 100",
        "XMOVE,2,20000,350,19","XMOVE,2,20000,350,2001","XMOVE,2,20000,350,100,0"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(ModHc13Host_Parse(bad[i],strlen(bad[i]),&c)!=HOST_OK);
    assert(ModHc13Host_Parse("MOVE,1,0\0,100",13,&c)!=HOST_OK);
}
static void experiment_move_tests(void)
{
    reset(true);
    world.imu_rx_bytes=1234;world.imu_good_frames=600;
    world.imu_checksum_errors=2;world.imu_uart_errors=1;
    world.imu_rearm_failures=0;world.imu_recovery_stage=1;world.imu_hz=0;
    input("GET,42,IMU\n");
    assert(strstr(output,"IMU,42,1234,600,2,1,0,1,0\n") && moves==0);
    input("GET,1,EXPERIMENT\n");
    assert(strstr(output,"EXPERIMENT,1,1,20,2000\n") && moves==0);
    input("XMOVE,2,20000,350,180\n");
    assert(moves==1 && last_ramp==180 && strstr(output,"ACK,2,XMOVE,ACCEPTED\n"));
    input("XMOVE,2,20000,350,200\n");
    assert(moves==1 && strstr(output,"ERR,2,REQUEST_CONFLICT\n"));
    arrive(20000);done(20000);
    assert(strstr(output,"DONE,2,XMOVE,20000\n"));
    input("MODE,3,VISION_DEMO\nXMOVE,4,20000,350,100\n");
    assert(strstr(output,"ERR,4,MODE_DENIED\n"));
}
static void buffered_experiment_tests(void)
{
    reset(true);
    ctx.hooks.capture_begin=capture_begin;
    ctx.hooks.capture_read=capture_read;
    captured_id=0;capture_starts=capture_reads=0;
    input("GET,9,CAPTURE\n");
    assert(strstr(output,"CAPTURE,9,1\n") && !moves);
    world.imu_ready=false;
    input("XMOVE,10,20000,350,100,1\n");
    assert(strstr(output,"ERR,10,IMU_RATE_LOW\n") && !moves && !capture_starts);
    world.imu_ready=true;
    input("XMOVE,12,20000,350,100,1\nXMOVE,12,20000,350,100,1\n");
    assert(moves==1 && capture_starts==1 && captured_id==12);
    input("XMOVE,12,20000,350,100\n");
    assert(strstr(output,"ERR,12,REQUEST_CONFLICT\n") && capture_starts==1);
    input("GET,14,CAPTURE,12,0\n");
    assert(!capture_reads && !strstr(output,"CAPDATA,14") && strstr(output,"ERR,14,BUSY\n"));
    arrive(20000);done(20000);
    input("GET,15,CAPTURE,12,0\n");
    assert(capture_reads==1 && moves==1 && ctx.highest_id==12);
    assert(strstr(output,"CAPDATA,15,12,0,1,0,1000,1500,0,AAAAAAAAAAA=,de7056ac\n"));
    input("GET,16,CAPTURE,99,0\n");
    assert(strstr(output,"ERR,16,NOT_READY\n"));
    input("XMOVE,17,80000,350,100\n");
    assert(moves==2 && capture_starts==1);
}
static void handshake_and_calibration(void)
{
    reset(false);input("GET,1,INFO\nGET,2,CAL\nGET,3,STATE\nPING,4\n");
    assert(!moves && !stops && !servos && !records);
    assert(strstr(output,"INFO,1,2,12345,0,115200,200,20,500,100,1000,60,500\n"));
    assert(strstr(output,"CAL,2,0,0,-1,90000,NA,125000,90000,0,180000,NA,NA,4000\n"));
    input("HOME,10\nPREP,11\nSHAPE,12,NORMAL\nSHAPE,13,FOLDED\n");
    assert(!moves && !servos);assert(occurrences(output,"NOT_CALIBRATED")==4);
    input("MODE,14,VISION_PICK\n");assert(ctx.mode==HOST_VISION_PICK && !servos && !moves);
    input("V,-30000,500,95\nTRIGGER,15\n");assert(!moves && !servos);
    input("MODE,16,MANUAL\nSERVO,17,1,120000\n");assert(servos==1 && !moves);
    assert(!strstr(output,"DONE,17"));
}
static void prep_aliases_and_delays(void)
{
    reset(true);input("HOME,100\n");assert(moves==1 && !servos);
    assert(strstr(output,"ACK,100,HOME,ACCEPTED\n"));assert(strstr(output,"PREPARING"));
    tick(300);arrive(0);assert(servos==1 && !ctx.prepared);assert(!strstr(output,"DONE,100"));
    done(0);tick(99);assert(!ctx.prepared);tick(1);assert(ctx.prepared && !ctx.active);
    assert(strstr(output,"READY") && strstr(output,"DONE,100,HOME,0\n"));
    input("HOME,100\n");assert(moves==1 && servos==1);
    input("HOME,99\n");assert(strstr(output,"STALE_REQUEST"));
    reset(true);prepare(1,"PREP");assert(strstr(output,"DONE,1,PREP,0\n"));
    reset(true);prepare(1,"SHAPE_NORMAL");assert(strstr(output,"DONE,1,SHAPE,0\n"));
}
static void pick_threshold_and_lock(void)
{
    reset(true);prepare(1,"HOME");
    input("PICK,2,30000,500,4000\nPICK,3,-3000,500,4000\n");assert(moves==1 && servos==1);
    input("PICK,4,-30000,500,4000\n");assert(moves==2 && !ctx.prepared && !ctx.closed_once);
    assert(ModHc13Host_FeedbackPeriod(&ctx)==5U);
    input("PICK,4,-30000,500,4000\n");assert(moves==2 && servos==1);
    input("PICK,4,-30000,500,3000\n");assert(strstr(output,"REQUEST_CONFLICT"));
    feedback(-25000);assert(servos==1);
    world.pos_mdeg=-28000;drain();assert(servos==1); /* Same cached feedback is not new evidence. */
    feedback(-28000);assert(servos==2 && ctx.closed_once && last_servo==90000);
    feedback(-29000);drain();assert(servos==2);
    input("V,-60000,300,95\n");assert(world.target_mdeg==-30000 && moves==2);
    arrive(-29700);done(-29700);tick(149);assert(ctx.active);tick(1);assert(!ctx.active);
    assert(strstr(output,"DONE,4,PICK,-29700\n"));
    assert(occurrences(output,",CLOSE_APPLIED,")==1);
    input("PICK,4,-30000,500,4000\n");assert(moves==2 && servos==2);
    input("PICK,5,-60000,500,4000\n");assert(moves==2);assert(strstr(output,"ERR,5,NOT_READY"));
    assert(ModHc13Host_FeedbackPeriod(&ctx)==50U);
}
static void candidates_and_trigger(void)
{
    reset(true);input("MODE,1,VISION_PICK\n");assert(!moves && !servos);
    prepare(2,"PREP");unsigned baseline=moves;
    input("V,-30000,250,95\nV,-60000,500,95\n");assert(moves==baseline && ctx.candidate_valid);
    input("TRIGGER,3\n");assert(moves==baseline+1 && world.target_mdeg==-60000 && !ctx.candidate_valid);
    input("V,-75000,500,95\nTRIGGER,3\n");assert(moves==baseline+1 && world.target_mdeg==-60000);
    input("TRIGGER,4\n");assert(moves==baseline+1 && strstr(output,"ERR,4,BUSY"));
    input("STOP,5\n");assert(!ctx.active && !ctx.prepared);unsigned n=servos;
    feedback(-58000);ModHc13Host_MotionArrived(&ctx,now,-60000);ModHc13Host_MotionFinished(&ctx,HOST_OK,now,-60000);tick(1000);
    assert(servos==n && !ctx.prepared && !strstr(output,"DONE,3,TRIGGER"));
    reset(true);input("MODE,1,VISION_PICK\n");prepare(2,"PREP");input("V,-30000,250,95\n");
    tick(501);baseline=moves;input("TRIGGER,3\n");assert(moves==baseline && strstr(output,"VISION_STALE"));
}
static void folding_stop_and_faults(void)
{
    reset(true);ctx.config.fold_calibrated=true;ctx.config.folded_mdeg=60000;
    input("SHAPE,1,FOLDED\n");assert(!moves && servos==1 && last_servo==90000);
    tick(149);assert(!moves);tick(1);assert(moves==1 && world.target_mdeg==60000);
    arrive(60000);done(60000);assert(strstr(output,"DONE,1,SHAPE,60000"));
    reset(true);ctx.config.fold_calibrated=true;ctx.config.folded_mdeg=60000;
    input("SHAPE,1,FOLDED\nSTOP,2\n");tick(1000);assert(!moves && servos==1 && !ctx.active);
    reset(true);input("PREP,1\n");input("STOP,2\n");unsigned n=servos;
    arrive(0);done(0);tick(1000);assert(servos==n && !ctx.prepared);
    reset(true);prepare(1,"HOME");input("PICK,2,-30000,500,4000\nMODE,3,VISION_DEMO\n");
    n=servos;feedback(-28000);tick(1000);assert(servos==n && ctx.mode==HOST_VISION_DEMO);
    reset(true);prepare(1,"HOME");input("PICK,2,-30000,500,4000\n");
    n=servos;world.hx_healthy=false;feedback(-28000);assert(servos==n && !ctx.active);
    reset(true);prepare(1,"HOME");input("PICK,2,-30000,500,4000\n");servo_error=HOST_TX_FAILED;
    feedback(-28000);assert(!ctx.active && strstr(output,"ERR,2,FAULT"));
    n=servos;feedback(-29000);tick(500);assert(servos==n);
}
static void ordinary_move_record_and_pressure(void)
{
    reset(true);input("MOVE,1,-30000,500\n");assert(moves==1 && servos==0);
    feedback(-28000);arrive(-29700);done(-29700);assert(!servos && strstr(output,"DONE,1,MOVE"));
    reset(true);world.imu_ready=false;input("RECORD,1,1\n");assert(!records && strstr(output,"IMU_RATE_LOW"));
    world.imu_ready=true;input("RECORD,2,1\n");assert(records==1 && world.record_on);
    input("MOVE,3,-30000,500\nRECORD,4,0\n");assert(records==1 && world.record_on && strstr(output,"ERR,4,BUSY"));
    input("STOP,5\n");assert(!ctx.active);
    reset(true);prepare(1,"HOME");input("PICK,2,-30000,500,4000\n");transport_ready=false;
    input("GET,3,INFO\nGET,4,CAL\nGET,5,STATE\nPING,6\nPING,7\nPING,8\n");
    feedback(-28000);assert(ctx.closed_once && servos==2); /* TX pressure cannot delay physical threshold action. */
    arrive(-29700);done(-29700);tick(150);assert(ctx.finish_pending);
    transport_ready=true;drain();assert(strstr(output,"DONE,2,PICK,-29700"));
    assert(occurrences(output,",CLOSE_APPLIED,")==1);
    reset(true);now+=4000;drain();assert(!moves && !stops && !servos);
    reset(true);prepare(1,"HOME");input("PICK,2,-30000,500,4000\n");unsigned n=servos;
    tick(3001);feedback(-29000);assert(stops==1 && servos==n && !ctx.active && strstr(output,"LINK_LOST"));
}
static void frame_recovery(void)
{
    reset(true);input("MOVE,1,-3");assert(!moves);input("0000,250\r\n");assert(moves==1);
    ModHc13Host_InputLost(&ctx,now);drain();assert(!ctx.active && stops==1);
    input("MOVE,2,-30000,250\n");assert(moves==1);
    input("MOVE,2,-30000,250\n");assert(moves==2);
    reset(true);char bad[230];memset(bad,'X',sizeof(bad));ModHc13Host_Receive(&ctx,(const uint8_t *)bad,sizeof(bad),now);
    input("\nGET,3,INFO\n");assert(!moves && strstr(output,"INFO,3,2"));
    reset(true);ctx.last_acc_send_ms=UINT32_MAX-20U;ctx.last_rx_ms=UINT32_MAX-20U;now=40;world.acc_ms=5;
    drain();assert(strstr(output,"ACC,0,5,85,-20,998"));
}
static void complete_prepare(void) { tick(300);arrive(0);done(0);tick(100); }
static void pc_preset_pick(void)
{
    /* No MCU PREP calibration is needed for the PC's explicitly opened
     * working/fold presets. Closure still uses fresh encoder feedback. */
    for (unsigned i=0;i<2;i++) {
        reset(false);
        world.pos_mdeg=i ? HOST_PC_FOLDED_MDEG : HOST_PC_WORKING_MDEG;
        input("SERVO,1,1,125000\n");
        input("PICK,2,-30000,500,4000\n");
        assert(moves==1 && servos==1 && ctx.active);
        feedback(-25000);assert(servos==1);
        feedback(-28000);assert(servos==2 && last_servo==90000);
        feedback(-29000);assert(servos==2);
        arrive(-30000);done(-30000);tick(150);
        assert(strstr(output,"DONE,2,PICK,-30000\n"));
    }
    /* An old open preset must not silently satisfy the new PICK gate. */
    reset(false);world.pos_mdeg=HOST_PC_WORKING_MDEG;
    input("SERVO,1,1,110000\nPICK,2,-30000,500,4000\n");
    assert(!moves && strstr(output,"NOT_READY"));
    reset(false);world.pos_mdeg=HOST_PC_WORKING_MDEG;
    input("PICK,1,-30000,500,4000\n");
    assert(!moves && strstr(output,"NOT_READY"));
    reset(false);world.pos_mdeg=HOST_PC_FOLDED_MDEG;
    input("SERVO,1,1,125000\nPICK,2,-30000,500,4000\nSTOP,3\n");
    feedback(-29000);assert(servos==1 && !ctx.active);
    HostCommand_t parsed;
    assert(ModHc13Host_Parse("MOVE,4,160000,200",17,&parsed)==HOST_OK);
    assert(ModHc13Host_Parse("MOVE,4,180000,200",17,&parsed)==HOST_OK);
    assert(ModHc13Host_Parse("MOVE,4,180100,200",17,&parsed)==HOST_OUT_OF_RANGE);

    /* A target between vertical 0 and the PC working position is still forward from
     * either PC preset; it must not be rejected by the legacy 0-degree rule. */
    for (unsigned i=0;i<2;i++) {
        reset(false);world.pos_mdeg=i ? HOST_PC_FOLDED_MDEG : HOST_PC_WORKING_MDEG;
        input("SERVO,1,1,125000\nPICK,2,5200,250,4000\n");
        assert(moves==1 && ctx.active && strstr(output,"ACK,2,PICK,ACCEPTED\n"));
        feedback(10000);assert(servos==1);
        feedback(7000);assert(servos==2 && last_servo==HOST_GRIP_CLOSE_MDEG);
        arrive(5200);done(5200);tick(150);
        assert(strstr(output,"DONE,2,PICK,5200\n"));
    }
    reset(false);world.pos_mdeg=HOST_PC_WORKING_MDEG;
    input("SERVO,1,1,125000\nPICK,2,91000,250,4000\n");
    assert(!moves && strstr(output,"OUT_OF_RANGE"));
}
static void high_speed_feedback_abort_diagnostic(void)
{
    reset(false);world.pos_mdeg=HOST_PC_FOLDED_MDEG;
    input("SERVO,1,1,125000\nPICK,2,-50000,425,4000\n");
    assert(ctx.active && moves==1 && servos==1);
    world.hx_healthy=false;world.hx_have_feedback=true;
    world.hx_feedback_age_ms=20;world.hx_last_angle_valid=true;
    world.hx_last_pos_mdeg=74200;world.hx_last_voltage_mv=8750;
    world.hx_last_current_ma=1200;world.hx_last_status=0;
    drain();
    assert(!ctx.active && stops==1 && servos==1);
    assert(occurrences(output,"HX8_DIAG,")==1);
    assert(strstr(output,"HX8_DIAG,1000,20,74200,8750,1200,0,VOLT_LOW\n"));
    assert(strstr(output,"ERR,2,FEEDBACK_STALE\n"));
    feedback(-46000);assert(servos==1);
}
static void pick_near_target_completion(void)
{
    /* The motor may hold 1.5 deg short. A fresh, quiet arrival from the
     * motion layer must let this PICK finish, so the PC can command return. */
    reset(false);world.pos_mdeg=HOST_PC_FOLDED_MDEG;
    input("SERVO,1,1,125000\nPICK,2,-2500,250,4000\n");
    assert(ctx.active && moves==1);
    feedback(-1000);assert(ctx.closed_once && last_servo==HOST_GRIP_CLOSE_MDEG);
    arrive(-1000);done(-1000);tick(150);
    assert(!ctx.active && strstr(output,"DONE,2,PICK,-1000\n"));

    /* A 2.6 deg miss remains outside the bounded completion tolerance. */
    reset(false);world.pos_mdeg=HOST_PC_FOLDED_MDEG;
    input("SERVO,1,1,125000\nPICK,2,-2500,250,4000\n");
    feedback(100);assert(ctx.closed_once);
    arrive(100);done(100);tick(150);
    assert(!ctx.active && strstr(output,"ERR,2,NOT_READY\n"));
    assert(!strstr(output,"DONE,2,PICK"));
}
static void vision_follow_streaming_tests(void)
{
    reset(false);world.pos_mdeg=HOST_PC_WORKING_MDEG;
    input("MODE,1,VISION_FOLLOW\n");
    assert(ctx.mode==HOST_VISION_FOLLOW && servos==0 && moves==0 && follow_moves==0);
    assert(strstr(output,"ACK,1,MODE,VISION_FOLLOW\n"));
    /* Gripper and discrete pick commands are strictly forbidden in VISION_FOLLOW. */
    input("GRIP,2,CLOSE\nSERVO,3,1,90000\nPICK,4,30000,300,4000\n");
    assert(servos==0 && moves==0 && occurrences(output,"MODE_DENIED")==3);
    /* First valid V frame dispatches immediately without waiting for discrete settle. */
    input("V,62000,300,90\n");
    assert(follow_moves==1 && last_follow_target==62000 && last_follow_speed==300 && ctx.following && servos==0);
    input("GET,5,STATE\n");
    assert(strstr(output,",VISION_FOLLOW,UNKNOWN,MOVING,1,1,90000,62000,"));
    /* Updates arriving faster than 40 ms coalesce and flush on the 40 ms boundary. */
    tick(20);input("V,55000,300,92\n");
    assert(follow_moves==1 && ctx.follow_pending);
    tick(10);input("V,48000,300,95\n");
    assert(follow_moves==1 && ctx.follow_pending);
    tick(10);
    assert(follow_moves==2 && last_follow_target==48000 && !ctx.follow_pending && servos==0);
    /* Sub-0.2 deg jitter (<200 mdeg) is filtered out. */
    tick(50);input("V,48100,300,95\n");
    assert(follow_moves==2 && !ctx.follow_pending);
    /* Idle >600 ms returns motion state to IDLE without stopping/faulting. */
    input("PING,6\n");tick(601);
    assert(!ctx.following && stops==0);
    /* Re-entering stream works immediately; link loss >3000 ms stops safely. */
    input("V,40000,300,95\n");
    assert(follow_moves==3 && ctx.following);
    tick(3001);
    assert(!ctx.following && stops==1 && ctx.fault==HOST_LINK_LOST && servos==0);
}
static void emit_fixture(void)
{
    reset(true);emit_stdout=true;input("GET,1,INFO\nGET,2,CAL\nGET,3,STATE\nPING,4\n");
    tick(50);input("HOME,50\n");complete_prepare();input("RECORD,51,1\nPICK,52,-30000,500,4000\n");
    feedback(-28000);tick(150);arrive(-29700);done(-29700);
    input("MODE,53,VISION_PICK\n");input("GET,54,STATE\nPREP,55\n");complete_prepare();
    input("V,-60000,250,95\nTRIGGER,56\n");feedback(-57000);input("STOP,57\nGET,58,STATE\n");
}
static void peer(bool calibrated)
{
    reset(calibrated);emit_stdout=true;char line[512];
    while (fgets(line,sizeof(line),stdin)) {
        now+=50;world.acc_count++;world.acc_ms=now;
        if (!strncmp(line,"@TICK,",6)) { unsigned long delta=0;assert(sscanf(line+6,"%lu",&delta)==1);now+=(uint32_t)delta;drain(); }
        else if (line[0]=='@') {
            char kind[16];long pos=0;assert(sscanf(line,"@%15[^,],%ld",kind,&pos)==2);
            if (!strcmp(kind,"DONE")) done((int32_t)pos);
            else if (!strcmp(kind,"ARRIVED")) arrive((int32_t)pos);
            else if (!strcmp(kind,"FB")) feedback((int32_t)pos);
        } else input(line);
    }
}
int main(int argc,char **argv)
{
    if (argc>1 && !strcmp(argv[1],"--emit-fixture")) { emit_fixture();return 0; }
    if (argc>1 && !strcmp(argv[1],"--peer")) { peer(true);return 0; }
    if (argc>1 && !strcmp(argv[1],"--peer-unconfigured")) { peer(false);return 0; }
    reset(false);
    world.pos_mdeg=100000;
    input("POSE,1,100000,181000\n");
    assert(ctx.config.pc_working_mdeg==HOST_PC_WORKING_MDEG);
    input("POSE,2,100000,170000\n");
    assert(strstr(output,"ACK,2,POSE,APPLIED\n") && !moves && !servos && !ctx.owner);
    assert(ctx.config.pc_working_mdeg==100000 && ctx.config.pc_folded_mdeg==170000);
    input("SERVO,3,1,125000\nPICK,4,70000,200,4000\n");
    assert(moves==1 && strstr(output,"ACK,4,PICK,ACCEPTED\n"));
    parser_tests();experiment_move_tests();buffered_experiment_tests();handshake_and_calibration();prep_aliases_and_delays();pick_threshold_and_lock();
    candidates_and_trigger();folding_stop_and_faults();ordinary_move_record_and_pressure();frame_recovery();pc_preset_pick();high_speed_feedback_abort_diagnostic();pick_near_target_completion();
    vision_follow_streaming_tests();
    puts("HC13 V2 tests passed: preparation, linked pick, one-shot threshold, no late closure, candidates, cancellation, recording and framing.");return 0;
}
