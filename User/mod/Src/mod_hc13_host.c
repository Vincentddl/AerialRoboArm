#include "mod_hc13_host.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Pure C, V2 only. No transport or actuator is touched except via hooks. */
static const char *const errors[]={"NONE","BAD_FRAME","BAD_NUMBER","OUT_OF_RANGE","NOT_READY",
    "BUSY","MODE_DENIED","NOT_CALIBRATED","HX8_OFFLINE","IMU_OFFLINE","IMU_RATE_LOW","FAULT",
    "MOTION_TIMEOUT","FEEDBACK_STALE","VISION_STALE","LINK_LOST","STOPPED","REQUEST_CONFLICT",
    "STALE_REQUEST","UNSUPPORTED","FAULT"}; /* Internal TX_FAILED uses V2's FAULT wire code. */
const char *ModHc13Host_ErrorName(HostError_t e)
{ return (unsigned)e<sizeof(errors)/sizeof(errors[0])?errors[e]:"FAULT"; }
const char *ModHc13Host_ModeName(HostMode_t m)
{ return m==HOST_MANUAL?"MANUAL":m==HOST_VISION_DEMO?"VISION_DEMO":"VISION_PICK"; }
uint32_t ModHc13Host_FeedbackPeriod(const Hc13Host_t *c)
{ return c->active && c->flow==HOST_FLOW_PICK?5U:50U; }
static const char *verb_name(HostVerb_t v)
{
    static const char *const names[]={"MOVE","HOME","STOP","MODE","SHAPE","GRIP","SERVO","RECORD",
        "GET","GET","GET","PING","V","PREP","PICK","TRIGGER"};
    return (unsigned)v<sizeof(names)/sizeof(names[0])?names[v]:"UNKNOWN";
}
static bool number(const char *s,uint32_t *out)
{
    uint32_t v=0;if (!s || !*s) return false;
    for (;*s;s++) {
        if (*s<'0' || *s>'9') return false;
        uint32_t d=(uint32_t)(*s-'0');
        if (v>UINT32_MAX/10U || (v==UINT32_MAX/10U && d>UINT32_MAX%10U)) return false;
        v=v*10U+d;
    }
    *out=v;return true;
}
static bool signed_number(const char *s,int32_t *out)
{
    bool negative=*s=='-';uint32_t v;
    if (negative) s++;
    if (!number(s,&v) || v>(negative?2147483648UL:2147483647UL)) return false;
    *out=negative?(int32_t)(0U-v):(int32_t)v;return true;
}
HostError_t ModHc13Host_Parse(const char *line,size_t length,HostCommand_t *out)
{
    if (!line || !out) return HOST_BAD_FRAME;
    memset(out,0,sizeof(*out));
    if (length && line[length-1]=='\r') length--;
    if (!length || length>HOST_LINE_MAX) return HOST_BAD_FRAME;
    char text[HOST_LINE_MAX+1U];char *f[6];unsigned count=1;f[0]=text;
    for (size_t i=0;i<length;i++) {
        if ((unsigned char)line[i]<33U || (unsigned char)line[i]>126U) return HOST_BAD_FRAME;
        text[i]=line[i];
        if (text[i]==',') {
            if (count==6U) return HOST_BAD_FRAME;
            text[i]=0;f[count++]=text+i+1;
        }
    }
    text[length]=0;
    for (unsigned i=0;i<count;i++) if (!*f[i]) return HOST_BAD_FRAME;
    if (!strcmp(f[0],"V")) {
        uint32_t speed,confidence;
        if (count!=4U) return HOST_BAD_FRAME;
        if (!signed_number(f[1],&out->value) || !number(f[2],&speed) || !number(f[3],&confidence)) return HOST_BAD_NUMBER;
        if (out->value< -90000 || out->value>85000 || !speed || speed>500U || confidence>100U) return HOST_OUT_OF_RANGE;
        out->verb=HOST_VISION;out->speed=(uint16_t)speed;out->confidence=(uint8_t)confidence;return HOST_OK;
    }
    if (count<2U || !number(f[1],&out->id)) return HOST_BAD_NUMBER;
    if (!out->id) return HOST_OUT_OF_RANGE;
    if (!strcmp(f[0],"GET") && count==3U) {
        if (!strcmp(f[2],"INFO")) out->verb=HOST_GET_INFO;
        else if (!strcmp(f[2],"CAL")) out->verb=HOST_GET_CAL;
        else if (!strcmp(f[2],"STATE")) out->verb=HOST_GET_STATE;
        else return HOST_BAD_FRAME;
    } else if (!strcmp(f[0],"PING") && count==2U) out->verb=HOST_PING;
    else if ((!strcmp(f[0],"MOVE") && count==4U) || (!strcmp(f[0],"PICK") && count==5U)) {
        uint32_t speed,lead=0;
        if (!signed_number(f[2],&out->value) || !number(f[3],&speed) || (count==5U && !number(f[4],&lead))) return HOST_BAD_NUMBER;
        if (out->value< -90000 || out->value>85000 || out->value%100 || !speed || speed>500U) return HOST_OUT_OF_RANGE;
        if (count==5U && (lead<3000U || lead>5000U || lead%100U)) return HOST_OUT_OF_RANGE;
        out->verb=count==5U?HOST_PICK:HOST_MOVE;out->speed=(uint16_t)speed;out->lead_mdeg=(uint16_t)lead;
    } else if (!strcmp(f[0],"HOME") && count==2U) out->verb=HOST_HOME;
    else if (!strcmp(f[0],"PREP") && count==2U) out->verb=HOST_PREP;
    else if (!strcmp(f[0],"TRIGGER") && count==2U) out->verb=HOST_TRIGGER;
    else if (!strcmp(f[0],"STOP") && count==2U) out->verb=HOST_STOP;
    else if (!strcmp(f[0],"MODE") && count==3U) {
        out->verb=HOST_MODE;
        if (!strcmp(f[2],"MANUAL")) out->value=HOST_MANUAL;
        else if (!strcmp(f[2],"VISION_DEMO")) out->value=HOST_VISION_DEMO;
        else if (!strcmp(f[2],"VISION_PICK")) out->value=HOST_VISION_PICK;
        else return HOST_BAD_FRAME;
    } else if (!strcmp(f[0],"SHAPE") && count==3U) {
        out->verb=HOST_SHAPE;
        if (!strcmp(f[2],"NORMAL")) out->value=0;
        else if (!strcmp(f[2],"FOLDED")) out->value=1;
        else return HOST_BAD_FRAME;
    } else if (!strcmp(f[0],"GRIP") && count==3U) {
        out->verb=HOST_GRIP;
        if (!strcmp(f[2],"OPEN")) out->value=1;
        else if (!strcmp(f[2],"CLOSE")) out->value=0;
        else return HOST_BAD_FRAME;
    } else if (!strcmp(f[0],"SERVO") && count==4U) {
        uint32_t channel;
        if (!number(f[2],&channel) || !signed_number(f[3],&out->value)) return HOST_BAD_NUMBER;
        if (channel!=1U || out->value<0 || out->value>180000 || out->value%1000) return HOST_OUT_OF_RANGE;
        out->verb=HOST_SERVO;
    } else if (!strcmp(f[0],"RECORD") && count==3U) {
        if (!signed_number(f[2],&out->value)) return HOST_BAD_NUMBER;
        if (out->value!=0 && out->value!=1) return HOST_OUT_OF_RANGE;
        out->verb=HOST_RECORD;
    } else return HOST_BAD_FRAME;
    return HOST_OK;
}

static bool queue_line(Hc13Host_t *c,const char *text)
{
    size_t n=strlen(text);
    if (!n || n>HOST_LINE_MAX+1U || c->tx_count>=HOST_TX_SLOTS) { c->queue_pressure++;return false; }
    unsigned index=(c->tx_head+c->tx_count)%HOST_TX_SLOTS;
    memcpy(c->tx[index].bytes,text,n+1U);c->tx[index].length=(uint16_t)n;c->tx_count++;return true;
}
static bool queue_format(Hc13Host_t *c,const char *fmt,...)
{
    char text[HOST_LINE_MAX+2U];va_list a;va_start(a,fmt);int n=vsnprintf(text,sizeof(text),fmt,a);va_end(a);
    return n>0 && (size_t)n<sizeof(text) && queue_line(c,text);
}
static void numeric(char *out,size_t cap,bool valid,int32_t value)
{ if (valid) (void)snprintf(out,cap,"%ld",(long)value);else (void)snprintf(out,cap,"NA"); }
static void snapshot(Hc13Host_t *c,HostSnapshot_t *s,uint32_t now)
{ memset(s,0,sizeof(*s));c->hooks.snapshot(s,now); }
static bool near(int32_t value,int32_t target,int32_t tolerance)
{ int64_t d=(int64_t)value-target;return d>=-tolerance && d<=tolerance; }
static bool state_frame(Hc13Host_t *c,uint32_t now,uint32_t reply)
{
    HostSnapshot_t s;snapshot(c,&s,now);
    char pos[16],target[16],voltage[12],grip[12];
    numeric(pos,sizeof(pos),s.hx_online,s.pos_mdeg);numeric(target,sizeof(target),s.target_valid,s.target_mdeg);
    numeric(voltage,sizeof(voltage),s.hx_online,s.voltage_mv);numeric(grip,sizeof(grip),c->pwm_valid,c->grip_mdeg);
    const char *shape="UNKNOWN",*motion=s.phase==2?"SETTLING":s.phase?"MOVING":c->active?"SETTLING":"IDLE";
    if (s.hx_healthy && !s.phase && !c->active && (s.ready || c->prepared)) {
        if (near(s.pos_mdeg,c->config.vertical_mdeg,1000)) shape="NORMAL";
        if (c->config.fold_calibrated && near(s.pos_mdeg,c->config.folded_mdeg,1000)) shape="FOLDED";
    }
    HostError_t fault=c->fault;
    if (fault==HOST_OK) fault=!s.hx_online?HOST_HX8_OFFLINE:!s.hx_healthy?HOST_FAULT:
        s.record_on&&!s.imu_online?HOST_IMU_OFFLINE:HOST_OK;
    if (fault==HOST_FAULT || fault==HOST_MOTION_TIMEOUT || fault==HOST_FEEDBACK_STALE) motion="FAULT";
    static const char *const grip_actions[]={"UNKNOWN","OPEN_CMD","CLOSE_CMD","ANGLE_CMD"};
    return queue_format(c,"STATE,%lu,%s,%s,%s,%u,%u,%s,%s,%s,%u,%u,%u,%s,%s,%lu,%s,%lu\n",
        (unsigned long)now,ModHc13Host_ModeName(c->mode),shape,motion,s.hx_online?1U:0U,s.imu_online?1U:0U,
        pos,target,voltage,s.imu_online?s.imu_hz:0U,s.record_on?1U:0U,c->mode!=HOST_VISION_PICK?1U:0U,
        grip,grip_actions[c->grip_action],(unsigned long)c->highest_id,ModHc13Host_ErrorName(fault),(unsigned long)reply);
}
static bool same(const HostCommand_t *a,const HostCommand_t *b)
{ return a->verb==b->verb && a->id==b->id && a->value==b->value && a->speed==b->speed && a->confidence==b->confidence && a->lead_mdeg==b->lead_mdeg; }
static HostCacheEntry_t *cached(Hc13Host_t *c,uint32_t id)
{
    for (unsigned i=0;i<HOST_CACHE_SLOTS;i++) if (c->cache[i].valid && c->cache[i].command.id==id) return &c->cache[i];
    return NULL;
}
static void remember(Hc13Host_t *c,const HostCommand_t *cmd,const char *reply)
{
    HostCacheEntry_t *e=cached(c,cmd->id);
    if (!e) { e=&c->cache[c->cache_next];c->cache_next=(uint8_t)((c->cache_next+1U)%HOST_CACHE_SLOTS); }
    e->valid=true;e->command=*cmd;(void)snprintf(e->reply,sizeof(e->reply),"%s",reply);
    if (cmd->id>c->highest_id) c->highest_id=cmd->id;
}
static void answer(Hc13Host_t *c,const HostCommand_t *cmd,HostError_t error,const char *result)
{
    char text[HOST_REPLY_MAX];
    if (error!=HOST_OK) (void)snprintf(text,sizeof(text),"ERR,%lu,%s\n",(unsigned long)cmd->id,ModHc13Host_ErrorName(error));
    else (void)snprintf(text,sizeof(text),"ACK,%lu,%s,%s\n",(unsigned long)cmd->id,verb_name(cmd->verb),result);
    remember(c,cmd,text);(void)queue_line(c,text);
}
static HostError_t apply_servo(Hc13Host_t *c,int32_t angle,uint8_t action)
{
    HostError_t e=c->hooks.servo(angle);
    if (e==HOST_OK) { c->pwm_valid=true;c->grip_mdeg=angle;c->grip_action=action; }
    return e;
}
static void invalidate_ready(Hc13Host_t *c) { c->prepared=false;c->candidate_valid=false; }
static void event(Hc13Host_t *c,HostEventKind_t kind,uint32_t now,int32_t pos,HostError_t reason)
{
    if (c->event_count>=8U) { c->event_overflow=true;return; }
    HostSnapshot_t s;snapshot(c,&s,now);
    HostEvent_t *e=&c->events[(c->event_head+c->event_count)%8U];
    e->kind=kind;e->reason=reason;e->t_ms=now;e->pos_mdeg=pos;e->pos_valid=s.hx_online;c->event_count++;
}
static void finish_pending(Hc13Host_t *c,HostError_t reason,uint32_t now,int32_t pos)
{
    if (!c->active) return;
    c->finish_pending=true;c->finish_reason=reason;c->finish_pos_mdeg=pos;c->finish_ms=now;
    c->flow=HOST_FLOW_IDLE;
    if (reason!=HOST_OK) { invalidate_ready(c);c->fault=reason; }
}
static HostError_t abort_flow(Hc13Host_t *c,HostError_t reason,uint32_t now,bool hold)
{
    HostSnapshot_t s;snapshot(c,&s,now);invalidate_ready(c);
    c->flow=HOST_FLOW_IDLE;c->arrived=false;c->closed_once=false;
    c->event_head=0;c->event_count=0;c->event_overflow=false;c->finish_event_queued=false;
    HostError_t e=hold?c->hooks.stop(now):HOST_OK;
    finish_pending(c,reason,now,s.pos_mdeg);
    return e;
}
static void begin(Hc13Host_t *c,const HostCommand_t *cmd,HostFlow_t flow,int32_t target,uint32_t now)
{
    c->active=true;c->active_command=*cmd;c->active_command.value=target;c->flow=flow;
    c->stage_ms=c->action_ms=now;c->closed_once=false;c->arrived=false;c->engine_done=false;
    c->finish_pending=false;c->finish_event_queued=false;c->event_overflow=false;c->event_head=c->event_count=0;
    invalidate_ready(c);c->fault=HOST_OK;
}
static bool preparation_calibrated(const Hc13Host_t *c)
{ return c->config.grip_calibrated && c->config.open_wait_calibrated; }
static HostError_t start_pick(Hc13Host_t *c,const HostCommand_t *cmd,const HostSnapshot_t *s,uint32_t now)
{
    if (c->mode==HOST_VISION_DEMO) return HOST_MODE_DENIED;
    if (!c->prepared || !near(s->pos_mdeg,c->config.vertical_mdeg,1000)) return HOST_NOT_READY;
    if (!c->config.grip_calibrated || !c->config.open_wait_calibrated || !c->config.close_wait_calibrated) return HOST_NOT_CALIBRATED;
    if (!s->hx_online) return HOST_HX8_OFFLINE;
    if (!s->hx_healthy) return HOST_NOT_READY;
    if (s->record_on && !s->imu_ready) return HOST_IMU_RATE_LOW;
    int64_t extent=(int64_t)c->config.extension_sign*(cmd->value-c->config.vertical_mdeg);
    int64_t travel=(int64_t)c->config.extension_sign*(cmd->value-s->pos_mdeg);
    if (cmd->value< -90000 || cmd->value>85000 || extent<=0 || extent>c->config.max_extension_mdeg ||
        travel<=cmd->lead_mdeg || cmd->lead_mdeg<3000U || cmd->lead_mdeg>5000U ||
        !cmd->speed || cmd->speed>c->config.max_speed) return HOST_OUT_OF_RANGE;
    HostError_t e=c->hooks.move(cmd->value,cmd->speed,HOST_PICK,now);
    if (e==HOST_OK) {
        begin(c,cmd,HOST_FLOW_PICK,cmd->value,now);c->pick_start_mdeg=s->pos_mdeg;c->last_feedback_seq=s->feedback_seq;
        event(c,HOST_EVENT_START,now,s->pos_mdeg,HOST_OK);
    }
    return e;
}
static HostError_t apply(Hc13Host_t *c,const HostCommand_t *cmd,const HostSnapshot_t *s,uint32_t now,const char **result)
{
    *result="APPLIED";
    if (cmd->verb==HOST_STOP) {
        HostError_t e=abort_flow(c,HOST_STOPPED,now,true);c->owner=false;c->fault=e;return e;
    }
    if (cmd->verb==HOST_MODE) {
        if (c->active || s->phase) {
            HostError_t e=abort_flow(c,HOST_STOPPED,now,true);
            if (e!=HOST_OK) return e;
        }
        invalidate_ready(c);c->fault=HOST_OK;c->mode=(HostMode_t)cmd->value;*result=ModHc13Host_ModeName(c->mode);return HOST_OK;
    }
    if (s->phase || c->active) return HOST_BUSY;
    if ((cmd->verb==HOST_MOVE || cmd->verb==HOST_HOME || cmd->verb==HOST_SHAPE || cmd->verb==HOST_SERVO) && c->mode!=HOST_MANUAL)
        return HOST_MODE_DENIED;
    if (cmd->verb==HOST_PREP && c->mode==HOST_VISION_DEMO) return HOST_MODE_DENIED;
    if (cmd->verb==HOST_GRIP && c->mode==HOST_VISION_DEMO) return HOST_MODE_DENIED;
    if (cmd->verb==HOST_SERVO || cmd->verb==HOST_GRIP) {
        if (cmd->verb==HOST_GRIP && !c->config.grip_calibrated) return HOST_NOT_CALIBRATED;
        HostError_t e=apply_servo(c,cmd->verb==HOST_SERVO?cmd->value:
            cmd->value?c->config.grip_open_mdeg:c->config.grip_close_mdeg,cmd->verb==HOST_SERVO?3U:cmd->value?1U:2U);
        if (e==HOST_OK) invalidate_ready(c);
        return e;
    }
    if (cmd->verb==HOST_RECORD) {
        if (cmd->value && !s->imu_online) return HOST_IMU_OFFLINE;
        if (cmd->value && !s->imu_ready) return HOST_IMU_RATE_LOW;
        *result=cmd->value?"ON":"OFF";return c->hooks.record(cmd->value!=0);
    }
    if (cmd->verb==HOST_PICK || cmd->verb==HOST_TRIGGER) {
        HostCommand_t locked=*cmd;
        if (cmd->verb==HOST_TRIGGER) {
            if (c->mode!=HOST_VISION_PICK) return HOST_MODE_DENIED;
            if (!c->candidate_valid || now-c->candidate_ms>500U) { c->candidate_valid=false;return HOST_VISION_STALE; }
            locked.value=c->candidate.value;locked.speed=c->candidate.speed;locked.lead_mdeg=c->config.default_lead_mdeg;
        }
        HostError_t e=start_pick(c,&locked,s,now);
        if (e==HOST_OK) *result="ACCEPTED";
        return e;
    }
    bool prep=cmd->verb==HOST_HOME || cmd->verb==HOST_PREP || (cmd->verb==HOST_SHAPE && !cmd->value);
    bool fold=cmd->verb==HOST_SHAPE && cmd->value;
    if (prep && !preparation_calibrated(c)) return HOST_NOT_CALIBRATED;
    if (fold && (!c->config.fold_calibrated || !c->config.grip_calibrated || !c->config.close_wait_calibrated)) return HOST_NOT_CALIBRATED;
    if (cmd->verb==HOST_MOVE || prep || fold) {
        if (!s->hx_online) return HOST_HX8_OFFLINE;
        if (!s->hx_healthy || (prep?!s->recovery_ready:!s->ready)) return HOST_NOT_READY;
        if (fold) {
            if (c->config.folded_mdeg< -90000 || c->config.folded_mdeg>85000) return HOST_OUT_OF_RANGE;
            HostError_t e=apply_servo(c,c->config.grip_close_mdeg,2);
            if (e!=HOST_OK) return e;
            begin(c,cmd,HOST_FLOW_FOLD_CLOSE_WAIT,c->config.folded_mdeg,now);
            event(c,HOST_EVENT_CLOSE,now,s->pos_mdeg,HOST_OK);*result="ACCEPTED";return HOST_OK;
        }
        if (!prep && s->record_on && !s->imu_ready) return HOST_IMU_RATE_LOW;
        int32_t target=prep?c->config.vertical_mdeg:cmd->value;
        HostError_t e=c->hooks.move(target,prep?30U:cmd->speed,prep?HOST_HOME:HOST_MOVE,now);
        if (e==HOST_OK) {
            begin(c,cmd,prep?HOST_FLOW_PREP_MOVE:HOST_FLOW_POSITION,target,now);
            event(c,prep?HOST_EVENT_PREPARING:HOST_EVENT_START,now,s->pos_mdeg,HOST_OK);*result="ACCEPTED";
        }
        return e;
    }
    return HOST_UNSUPPORTED;
}
static void handle(Hc13Host_t *c,const HostCommand_t *cmd,uint32_t now)
{
    HostSnapshot_t s;snapshot(c,&s,now);
    if (cmd->verb==HOST_VISION) {
        if (c->mode==HOST_VISION_PICK && c->prepared && !c->active && s.hx_healthy && cmd->confidence>=c->config.min_confidence) {
            c->candidate=*cmd;c->candidate_valid=true;c->candidate_ms=now;c->last_valid_ms=now;
        }
        return; /* Candidate only; never a motor command. */
    }
    c->last_valid_ms=now;
    if (cmd->verb==HOST_PING) { (void)queue_format(c,"PONG,%lu,%lu\n",(unsigned long)cmd->id,(unsigned long)now);return; }
    if (cmd->verb==HOST_GET_INFO) {
        (void)queue_format(c,"INFO,%lu,2,%lu,%lu,230400,200,20,%u,%u,%u,%u,500\n",(unsigned long)cmd->id,
            (unsigned long)c->boot_id,(unsigned long)c->highest_id,c->config.max_speed,c->config.ramp_ms,c->config.post_ms,c->config.min_confidence);return;
    }
    if (cmd->verb==HOST_GET_CAL) {
        char fold[16],open[16],close[16],ow[12],cw[12];
        numeric(fold,sizeof(fold),c->config.fold_calibrated,c->config.folded_mdeg);
        numeric(open,sizeof(open),c->config.grip_calibrated,c->config.grip_open_mdeg);
        numeric(close,sizeof(close),c->config.grip_calibrated,c->config.grip_close_mdeg);
        numeric(ow,sizeof(ow),c->config.open_wait_calibrated,c->config.open_wait_ms);
        numeric(cw,sizeof(cw),c->config.close_wait_calibrated,c->config.close_wait_ms);
        (void)queue_format(c,"CAL,%lu,%ld,%ld,%d,%lu,%s,%s,%s,0,180000,%s,%s,%u\n",(unsigned long)cmd->id,
            (long)c->config.vertical_mdeg,(long)c->config.vertical_mdeg,c->config.extension_sign,
            (unsigned long)c->config.max_extension_mdeg,fold,open,close,ow,cw,c->config.default_lead_mdeg);return;
    }
    if (cmd->verb==HOST_GET_STATE) { (void)state_frame(c,now,cmd->id);return; }
    HostCacheEntry_t *entry=cached(c,cmd->id);
    if (entry) {
        if (same(&entry->command,cmd)) (void)queue_line(c,entry->reply);
        else (void)queue_format(c,"ERR,%lu,REQUEST_CONFLICT\n",(unsigned long)cmd->id);
        return;
    }
    if (cmd->id<=c->highest_id) { (void)queue_format(c,"ERR,%lu,STALE_REQUEST\n",(unsigned long)cmd->id);return; }
    bool priority=cmd->verb==HOST_STOP || cmd->verb==HOST_MODE;
    if (!priority && c->tx_count>HOST_TX_SLOTS-3U) { (void)queue_format(c,"ERR,%lu,BUSY\n",(unsigned long)cmd->id);return; }
    if (priority) { c->tx_count=0;c->tx_head=0; }
    const char *result;HostError_t e=apply(c,cmd,&s,now,&result);answer(c,cmd,e,result);
    if (e==HOST_OK && cmd->verb!=HOST_RECORD && cmd->verb!=HOST_STOP) c->owner=true;
}

static void advance(Hc13Host_t *c,HostSnapshot_t *s,uint32_t now)
{
    if (!c->active || c->finish_pending) return;
    if (!s->hx_healthy) { (void)abort_flow(c,HOST_FEEDBACK_STALE,now,true);return; }
    if (c->engine_done && !c->arrived && (c->flow==HOST_FLOW_PREP_MOVE || c->flow==HOST_FLOW_POSITION ||
                                         c->flow==HOST_FLOW_FOLD_MOVE || c->flow==HOST_FLOW_PICK)) {
        (void)abort_flow(c,HOST_FAULT,now,true);return;
    }
    if ((c->flow==HOST_FLOW_POSITION || c->flow==HOST_FLOW_PREP_MOVE || c->flow==HOST_FLOW_FOLD_MOVE ||
         c->flow==HOST_FLOW_PICK) && !c->engine_done && now-c->action_ms>10000U) {
        (void)abort_flow(c,HOST_MOTION_TIMEOUT,now,true);return;
    }
    if (c->flow==HOST_FLOW_PREP_MOVE && c->arrived) {
        if (!near(s->pos_mdeg,c->config.vertical_mdeg,500)) { (void)abort_flow(c,HOST_NOT_READY,now,true);return; }
        HostError_t e=apply_servo(c,c->config.grip_open_mdeg,1);
        if (e!=HOST_OK) { (void)abort_flow(c,e,now,true);return; }
        c->flow=HOST_FLOW_PREP_OPEN_WAIT;c->stage_ms=now;event(c,HOST_EVENT_OPEN,now,s->pos_mdeg,HOST_OK);
    }
    if (c->flow==HOST_FLOW_PREP_OPEN_WAIT && c->engine_done && now-c->stage_ms>=c->config.open_wait_ms) {
        if (!near(s->pos_mdeg,c->config.vertical_mdeg,500)) { (void)abort_flow(c,HOST_NOT_READY,now,true);return; }
        c->prepared=true;c->candidate_valid=false;event(c,HOST_EVENT_READY,now,s->pos_mdeg,HOST_OK);
        finish_pending(c,HOST_OK,now,s->pos_mdeg);return;
    }
    if (c->flow==HOST_FLOW_FOLD_CLOSE_WAIT && now-c->stage_ms>=c->config.close_wait_ms) {
        HostError_t e=c->hooks.move(c->config.folded_mdeg,30,HOST_SHAPE,now);
        if (e!=HOST_OK) { (void)abort_flow(c,e,now,true);return; }
        c->flow=HOST_FLOW_FOLD_MOVE;c->action_ms=now;event(c,HOST_EVENT_START,now,s->pos_mdeg,HOST_OK);
    }
    if (c->flow==HOST_FLOW_PICK && !c->closed_once && s->feedback_seq!=c->last_feedback_seq) {
        c->last_feedback_seq=s->feedback_seq;
        int64_t total=(int64_t)c->config.extension_sign*(c->active_command.value-c->pick_start_mdeg);
        int64_t progress=(int64_t)c->config.extension_sign*(s->pos_mdeg-c->pick_start_mdeg);
        if (progress>=total-c->active_command.lead_mdeg) {
            HostError_t e=apply_servo(c,c->config.grip_close_mdeg,2);
            if (e!=HOST_OK) { (void)abort_flow(c,e,now,true);return; }
            c->closed_once=true;c->close_ms=now;event(c,HOST_EVENT_CLOSE,now,s->pos_mdeg,HOST_OK);
        }
    }
    if (c->flow==HOST_FLOW_PICK && c->engine_done && !c->closed_once) {
        (void)abort_flow(c,HOST_NOT_READY,now,true);return;
    }
    bool finished=c->engine_done && (c->flow==HOST_FLOW_POSITION || c->flow==HOST_FLOW_FOLD_MOVE ||
        (c->flow==HOST_FLOW_PICK && c->closed_once && now-c->close_ms>=c->config.close_wait_ms));
    if (finished) {
        if (!near(s->pos_mdeg,c->active_command.value,1000)) { (void)abort_flow(c,HOST_NOT_READY,now,true);return; }
        finish_pending(c,HOST_OK,now,s->pos_mdeg);
    }
}
static void flush_events(Hc13Host_t *c)
{
    static const char *const stages[]={"PREPARING","OPEN_APPLIED","READY","START","CLOSE_APPLIED","ARRIVED","END","ABORT"};
    while (c->event_count && c->tx_count<HOST_TX_SLOTS) {
        HostEvent_t *e=&c->events[c->event_head];char pos[16];numeric(pos,sizeof(pos),e->pos_valid,e->pos_mdeg);
        if (!queue_format(c,"MOTION,%lu,%s,%lu,%ld,%s,%s\n",(unsigned long)c->active_command.id,stages[e->kind],
                (unsigned long)e->t_ms,(long)c->active_command.value,pos,ModHc13Host_ErrorName(e->reason))) break;
        c->event_head=(uint8_t)((c->event_head+1U)%8U);c->event_count--;
    }
}
void ModHc13Host_Init(Hc13Host_t *c,const HostConfig_t *cfg,const HostHooks_t *hooks,uint32_t boot,uint32_t now)
{
    memset(c,0,sizeof(*c));c->config=*cfg;c->hooks=*hooks;c->mode=HOST_MANUAL;
    c->boot_id=boot?boot:1U;c->last_valid_ms=c->last_acc_send_ms=now;
}
void ModHc13Host_Receive(Hc13Host_t *c,const uint8_t *bytes,size_t length,uint32_t now)
{
    for (size_t i=0;i<length;i++) {
        uint8_t b=bytes[i];c->last_rx_ms=now;
        if (b=='\n') {
            if (c->discard_input) { c->invalid_frames++;(void)queue_format(c,"ERR,0,BAD_FRAME\n"); }
            else if (c->input_used && !(c->input_used==1U && c->input[0]=='\r')) {
                HostCommand_t cmd;HostError_t e=ModHc13Host_Parse(c->input,c->input_used,&cmd);
                if (e==HOST_OK) handle(c,&cmd,now);
                else { c->invalid_frames++;(void)queue_format(c,"ERR,%lu,%s\n",(unsigned long)cmd.id,ModHc13Host_ErrorName(e)); }
            }
            c->input_used=0;c->discard_input=false;
        } else if (!c->discard_input) {
            if ((b<33U && b!='\r') || b>126U || (c->input_used>=HOST_LINE_MAX && b!='\r') || c->input_used>=sizeof(c->input) ||
                (c->input_used && c->input[c->input_used-1U]=='\r')) c->discard_input=true;
            else c->input[c->input_used++]=(char)b;
        }
    }
}
void ModHc13Host_InputLost(Hc13Host_t *c,uint32_t now)
{
    c->input_used=0;c->discard_input=true;c->invalid_frames++;invalidate_ready(c);
    if (c->active) (void)abort_flow(c,HOST_FEEDBACK_STALE,now,true);
}
void ModHc13Host_MotionArrived(Hc13Host_t *c,uint32_t now,int32_t pos)
{
    if (!c->active || c->finish_pending || c->flow==HOST_FLOW_IDLE || c->arrived) return;
    c->arrived=true;event(c,HOST_EVENT_ARRIVED,now,pos,HOST_OK);
}
void ModHc13Host_MotionFinished(Hc13Host_t *c,HostError_t reason,uint32_t now,int32_t pos)
{
    if (!c->active || c->finish_pending) return;
    if (reason!=HOST_OK) { (void)abort_flow(c,reason,now,false);return; }
    if (c->flow==HOST_FLOW_IDLE) return; /* Stop/cancel callback cannot rearm anything. */
    c->engine_done=true;c->finish_pos_mdeg=pos;
}
void ModHc13Host_Poll(Hc13Host_t *c,uint32_t now)
{
    HostSnapshot_t s;snapshot(c,&s,now);
    if ((c->input_used || c->discard_input) && now-c->last_rx_ms>500U) {
        c->input_used=0;c->discard_input=false;c->invalid_frames++;
    }
    if (c->candidate_valid && now-c->candidate_ms>500U) c->candidate_valid=false;
    if (c->owner && now-c->last_valid_ms>3000U) {
        if (c->active) (void)abort_flow(c,HOST_LINK_LOST,now,true);
        else invalidate_ready(c);
        c->owner=false;c->fault=HOST_LINK_LOST;
    }
    if (c->event_overflow) (void)abort_flow(c,HOST_FAULT,now,true);
    advance(c,&s,now);
    if (c->event_overflow) (void)abort_flow(c,HOST_FAULT,now,true);
    if (c->finish_pending && !c->finish_event_queued) {
        event(c,c->finish_reason==HOST_OK?HOST_EVENT_END:HOST_EVENT_ABORT,c->finish_ms,c->finish_pos_mdeg,c->finish_reason);
        c->finish_event_queued=true;
    }
    flush_events(c);
    if (c->finish_pending && !c->event_count && c->tx_count<HOST_TX_SLOTS) {
        char reply[HOST_REPLY_MAX];
        if (c->finish_reason==HOST_OK) (void)snprintf(reply,sizeof(reply),"DONE,%lu,%s,%ld\n",
            (unsigned long)c->active_command.id,verb_name(c->active_command.verb),(long)c->finish_pos_mdeg);
        else (void)snprintf(reply,sizeof(reply),"ERR,%lu,%s\n",(unsigned long)c->active_command.id,ModHc13Host_ErrorName(c->finish_reason));
        HostCacheEntry_t *entry=cached(c,c->active_command.id);
        if (entry) (void)snprintf(entry->reply,sizeof(entry->reply),"%s",reply);
        (void)queue_line(c,reply);c->active=false;c->finish_pending=false;c->flow=HOST_FLOW_IDLE;
    }
    if (c->tx_count) {
        HostTxFrame_t *f=&c->tx[c->tx_head];
        if (c->hooks.send(f->bytes,f->length)) { c->tx_head=(uint8_t)((c->tx_head+1U)%HOST_TX_SLOTS);c->tx_count--; }
    } else if (c->mode!=HOST_VISION_PICK && s.imu_online && s.acc_count!=c->last_acc_count &&
               now-c->last_acc_send_ms>=50U && now-c->last_rx_ms>=3U) {
        char line[96];int n=snprintf(line,sizeof(line),"ACC,%lu,%lu,%ld,%ld,%ld\n",(unsigned long)c->acc_seq,
            (unsigned long)s.acc_ms,(long)s.acc_mg[0],(long)s.acc_mg[1],(long)s.acc_mg[2]);
        if (n>0 && (size_t)n<sizeof(line) && c->hooks.send(line,(size_t)n)) {
            c->acc_seq++;c->last_acc_send_ms=now;c->last_acc_count=s.acc_count;
        }
    }
}
