#include "mod_hc13_host.h"
#include "app_hc13_host_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static Hc13Host_t ctx;static HostSnapshot_t world;static uint32_t now;
static unsigned moves,stops,servos,records;static int32_t last_servo;
static bool transport_ready,emit_stdout;static HostError_t move_error,stop_error,servo_error;
static char output[64000];static size_t output_used;
static void snapshot(HostSnapshot_t *s,uint32_t t) { (void)t;*s=world; }
static HostError_t move(int32_t target,uint16_t speed,HostVerb_t kind,uint32_t t)
{
    assert(speed>=1 && speed<=500);assert(kind!=HOST_VISION);moves++;
    if (move_error) return move_error;
    world.target_valid=true;world.target_mdeg=target;world.motion_start_ms=t;world.phase=1;return HOST_OK;
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
    now=1000;moves=stops=servos=records=0;last_servo=-1;output_used=0;output[0]=0;
    move_error=stop_error=servo_error=HOST_OK;transport_ready=true;
    HostConfig_t cfg={.vertical_mdeg=HOST_VERTICAL_MDEG,.grip_open_mdeg=HOST_GRIP_OPEN_MDEG,
        .grip_close_mdeg=HOST_GRIP_CLOSE_MDEG,.grip_calibrated=true,.extension_sign=-1,
        .max_extension_mdeg=90000,.max_speed=500,.ramp_ms=100,.post_ms=1000,.min_confidence=60,
        .open_wait_calibrated=calibrated,.close_wait_calibrated=calibrated,
        .open_wait_ms=100,.close_wait_ms=150,.default_lead_mdeg=4000};
    HostHooks_t hooks={snapshot,move,stop,servo,record,send};
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
    assert(ctx.prepared && !ctx.active && last_servo==135000);
}
static void parser_tests(void)
{
    HostCommand_t c;
    const char *good[]={"MOVE,1,-30000,250","PICK,2,-30000,500,4000","PREP,3","TRIGGER,4",
        "HOME,5","STOP,6","MODE,7,VISION_PICK","SHAPE,8,NORMAL","GRIP,9,OPEN","SERVO,10,1,120000",
        "RECORD,11,1","GET,12,STATE","GET,13,INFO","GET,14,CAL","PING,15","V,-60000,500,95","MOVE,4294967295,85000,500"};
    for (unsigned i=0;i<sizeof(good)/sizeof(good[0]);i++) assert(ModHc13Host_Parse(good[i],strlen(good[i]),&c)==HOST_OK);
    const char *bad[]={"PICK,1,-30000,500,2999","PICK,1,-30000,500,5001","PICK,1,-30000,500,4050",
        "PICK,1,-30000,0,4000","MOVE,1,-30000","MOVE,1,-30000,250,EXTRA","MOVE,1,-90001,500",
        "MOVE,1,85001,500","MOVE,1,-30101,500","MOVE,4294967296,0,100","MOVE,0,0,100","MOVE,1,abc,100",
        "MODE,1,AUTO","GRIP,1,UNKNOWN","SERVO,1,2,90000","SERVO,1,1,90500","RECORD,1,2",
        "V,-60000,500,101","V,-60000,0,90","V,-60000,500,95,3","MOVE,1,,100","MOVE,1,0, 100"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(ModHc13Host_Parse(bad[i],strlen(bad[i]),&c)!=HOST_OK);
    assert(ModHc13Host_Parse("MOVE,1,0\0,100",13,&c)!=HOST_OK);
}
static void handshake_and_calibration(void)
{
    reset(false);input("GET,1,INFO\nGET,2,CAL\nGET,3,STATE\nPING,4\n");
    assert(!moves && !stops && !servos && !records);
    assert(strstr(output,"INFO,1,2,12345,0,230400,200,20,500,100,1000,60,500\n"));
    assert(strstr(output,"CAL,2,0,0,-1,90000,NA,135000,90000,0,180000,NA,NA,4000\n"));
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
    parser_tests();handshake_and_calibration();prep_aliases_and_delays();pick_threshold_and_lock();
    candidates_and_trigger();folding_stop_and_faults();ordinary_move_record_and_pressure();frame_recovery();
    puts("HC13 V2 tests passed: preparation, linked pick, one-shot threshold, no late closure, candidates, cancellation, recording and framing.");return 0;
}
