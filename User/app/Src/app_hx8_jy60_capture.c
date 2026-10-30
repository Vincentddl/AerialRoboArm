#include "app_hx8_jy60_capture.h"
#if defined(HX8_JY60_CAPTURE)
#include "bsp_uart.h"
#include "drv_jy60.h"
#include "drv_fsus.h"
#include "drv_capture_command.h"
#include "mod_startup_angle.h"
#include "SEGGER_RTT.h"
#include "iwdg.h"
#include "app_jy61p_diagnostic.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static Jy60Parser_t imu;
static FsusFeedback_t fb;
static StartupAngleGate_t gate;
static StartupAngleGate_t recovery_gate;
static bool recovery_active,recovery_ready;
static float recovery_start;
static uint32_t imu_seq, hx_seq, drops, command_id, start_ms, poll_ms, last_fb_ms;
static uint32_t arrived_ms, post_ms=1000, drop_baseline;
static uint32_t checksum_baseline;
static uint32_t uart_baseline;
static uint32_t request_id;
static uint16_t speed=500, ramp=100;
static uint8_t phase, settled;
static bool have_fb, ready, query_pending, previous_angle_valid,manual_move_active;
static bool manual_mode=true; /* User enables experiment recording explicitly. */
static bool mode_pending,pending_manual_mode;
static float target, previous_angle;
static uint8_t reply[64];
static uint16_t reply_len;
static char line[32];
static uint8_t line_len;
static bool line_overflow;
#if defined(JY61P_CAPTURE)
/* About 45 kB/s of text at 600 IMU packets/s: buffer short host stalls. */
static char trace_buffer[8192];
static bool imu_rate_ok;
static uint32_t rate_acc,rate_gyro,rate_angle;
static void emit(const char *fmt,...);

static void imu_delay(uint32_t ms)
{
    uint32_t start=HAL_GetTick();
    while (HAL_GetTick()-start<ms) (void)HAL_IWDG_Refresh(&hiwdg);
}
static bool imu_write_register(uint8_t address,uint16_t value)
{
    uint8_t tx[]={0xFF,0xAA,address,(uint8_t)value,(uint8_t)(value>>8)};
    bool ok=BSP_UART_ImuWrite(tx,sizeof(tx))==ARA_OK;
    imu_delay(address==0x69U ? 200U : 20U);
    return ok;
}
static bool imu_probe(uint32_t duration,uint32_t min_frames)
{
    uint32_t start=HAL_GetTick(),recoveries=BSP_UART_RxDma_GetRecoveries();
    uint8_t bytes[64];DrvJy60_Init(&imu);
    while (HAL_GetTick()-start<duration) {
        (void)HAL_IWDG_Refresh(&hiwdg);
        uint16_t n=BSP_UART_Read(BSP_UART_ELRS,bytes,sizeof(bytes));
        for (unsigned i=0;i<n;i++) (void)DrvJy60_Push(&imu,bytes[i],HAL_GetTick());
    }
    return imu.good_frames>=min_frames && imu.checksum_errors==0 && BSP_UART_RxDma_GetRecoveries()==recoveries;
}
/* Bounded startup configuration, before accepting any actuator command.
 * No calibration/zeroing and no repeated EEPROM saves. RX frames verify the
 * link; measured rates in the main loop verify the requested output rate. */
static bool imu_configure(void)
{
    static const uint32_t bauds[]={115200,9600,230400,57600,38400,19200,4800};
    imu_delay(1200); /* Manufacturer specifies 1000 ms sensor startup. */
    for (unsigned b=0;b<sizeof(bauds)/sizeof(bauds[0]);b++) {
        if (BSP_UART_ImuSetBaud(bauds[b])!=ARA_OK) continue;
        if (!imu_probe(700,3)) continue;
        emit("[CAP] IMU_LINK baud=%lu good=%lu\r\n",(unsigned long)bauds[b],(unsigned long)imu.good_frames);
        if (!imu_write_register(0x69,0xB588) || !imu_write_register(0x04,0x0006)) return false;
        if (BSP_UART_ImuSetBaud(115200)!=ARA_OK) return false;
        if (!imu_probe(700,3)) {
            emit("[CAP] IMU_CONFIG_FAILED reason=baud_not_confirmed fallback_baud=%lu check_MCU_TX_to_IMU_RX=1\r\n",(unsigned long)bauds[b]);
            (void)BSP_UART_ImuSetBaud(bauds[b]);DrvJy60_Init(&imu);return false;
        }
        if (!imu_write_register(0x69,0xB588) ||
            !imu_write_register(0x02,0x000E) || /* ACC + GYRO + ANGLE only */
            !imu_write_register(0x03,0x000B) || /* 200 Hz */
            !imu_write_register(0x1F,0x0002)) return false; /* 98 Hz bandwidth */
        /* Drop startup/configuration backlog before measuring packet rates. */
        if (BSP_UART_ImuSetBaud(115200)!=ARA_OK) return false;
        if (!imu_probe(1000,540) || imu.acceleration.count<180 || imu.gyro.count<180 || imu.angle.count<180) {
            emit("[CAP] IMU_CONFIG_FAILED reason=rate_not_confirmed acc=%lu gyro=%lu angle=%lu\r\n",
                (unsigned long)imu.acceleration.count,(unsigned long)imu.gyro.count,(unsigned long)imu.angle.count);
            DrvJy60_Init(&imu);return false;
        }
        DrvJy60_Init(&imu);return true;
    }
    (void)BSP_UART_ImuSetBaud(115200);DrvJy60_Init(&imu);
    return false;
}
#else
static char trace_buffer[4096];
#endif

/* Atomic lines: RTT drops whole lines rather than printing misleading fragments. */
static void emit(const char *fmt,...)
{
    char text[256];va_list args;va_start(args,fmt);
    int n=vsnprintf(text,sizeof(text),fmt,args);va_end(args);
    if (n<=0 || n>=(int)sizeof(text)) { drops++;return; }
    if (SEGGER_RTT_Write(1,text,(unsigned)n)!=(unsigned)n) drops++;
    if (!strncmp(text,"[CAP]",5)) (void)SEGGER_RTT_Write(0,text,(unsigned)n);
}
static bool imu_live(uint32_t now)
{
#if defined(JY61P_CAPTURE)
    return imu_rate_ok && imu.acceleration.count && imu.angle.count && imu.gyro.count &&
        now-imu.acceleration.timestamp_ms<50U && now-imu.angle.timestamp_ms<50U && now-imu.gyro.timestamp_ms<50U;
#else
    return imu.angle.count && imu.gyro.count && now-imu.angle.timestamp_ms<200U && now-imu.gyro.timestamp_ms<200U;
#endif
}
static bool hx_feedback_healthy(uint32_t now)
{
    return have_fb && now-last_fb_ms<200U && isfinite(fb.angle_deg) &&
        fb.angle_deg>=-180.0f && fb.angle_deg<=180.0f &&
        fb.voltage_mv>=9000 && fb.voltage_mv<=12600 && !(fb.status&0xFEU);
}
static bool hx_healthy(uint32_t now)
{
    return hx_feedback_healthy(now) && (recovery_active ?
        ModStartupAngle_RecoveryPathOk(fb.angle_deg,recovery_start) :
        fb.angle_deg>=ARA_MAIN_ARM_ANGLE_MIN_DEG && fb.angle_deg<=ARA_MAIN_ARM_ANGLE_MAX_DEG);
}
static void set_record_mode(bool enabled)
{
    manual_mode=!enabled;ready=false;ModStartupAngle_Reset(&gate);
    emit("[CAP] MODE mode=%s record=%u imu_required=%u action=wait_for_stable_HX8\r\n",
        manual_mode ? "MANUAL_HX8" : "CAPTURE",enabled,enabled);
}
static void finish(const char *reason,uint32_t now,bool stop)
{
    if (stop) {
        uint8_t tx[FSUS_TX_BUF_SIZE];
        uint16_t n=DrvFsus_EncodeStop(tx,0,FSUS_STOP_MODE_HOLD,0);
        (void)BSP_UART_Fsus_Send(tx,n);
    }
    if (recovery_active)
        emit("[CAP] RECOVERY_END id=%lu t_ms=%lu reason=%s angle_mdeg=%ld\r\n",
            (unsigned long)command_id,(unsigned long)now,reason,(long)(fb.angle_deg*1000));
    else if (manual_move_active)
        emit("[CAP] MANUAL_END id=%lu t_ms=%lu reason=%s angle_mdeg=%ld\r\n",
            (unsigned long)command_id,(unsigned long)now,reason,(long)(fb.angle_deg*1000));
    else
        emit("[CAP] END id=%lu t_ms=%lu reason=%s drops=%lu imu_errors=%lu imu_seq=%lu hx_seq=%lu uart_recoveries=%lu\r\n",
            (unsigned long)command_id,(unsigned long)now,reason,
            (unsigned long)(drops-drop_baseline),(unsigned long)(imu.checksum_errors-checksum_baseline),
            (unsigned long)imu_seq,(unsigned long)hx_seq,(unsigned long)(BSP_UART_RxDma_GetRecoveries()-uart_baseline));
    manual_move_active=false;
    recovery_active=false;recovery_ready=false;ModStartupAngle_Reset(&recovery_gate);
    phase=0;ready=false;ModStartupAngle_Reset(&gate);
    if (mode_pending) {
        mode_pending=false;set_record_mode(!pending_manual_mode);
    }
}
static void on_feedback(uint32_t now)
{
    fb.timestamp_ms=now;last_fb_ms=now;have_fb=true;hx_seq++;
    emit("[HX8] seq=%lu t_ms=%lu angle_mdeg=%ld voltage=%d current=%d status=0x%02X\r\n",
        (unsigned long)hx_seq,(unsigned long)now,(long)(fb.angle_deg*1000),fb.voltage_mv,fb.current_ma,fb.status);
    if (!phase) {
        ready=ModStartupAngle_Update(&gate,&fb,true,true,now);
        recovery_ready=ModStartupAngle_UpdateRecovery(&recovery_gate,&fb,true,true,now);
        return;
    }
    if (!hx_healthy(now)) { finish("hx8_fault",now,true);return; }
    if (phase!=1) return;
    bool quiet=previous_angle_valid && fabsf(fb.angle_deg-previous_angle)<=0.3f;
    previous_angle=fb.angle_deg;previous_angle_valid=true;
    if (fabsf(fb.angle_deg-target)<=(recovery_active ? 0.5f : 1.0f) && quiet) { if (settled<5) settled++; }
    else settled=0;
    /* Five fresh 50 ms observations; never count cached values. */
    if (settled>=5) {
        phase=2;arrived_ms=now;
        emit("[CAP] %s id=%lu t_ms=%lu angle_mdeg=%ld\r\n",recovery_active ? "RECOVERY_ARRIVED" : manual_move_active ? "MANUAL_ARRIVED" : "ARRIVED",
             (unsigned long)command_id,(unsigned long)now,(long)(fb.angle_deg*1000));
    }
}
static void monitor(uint32_t now)
{
    if (query_pending) {
        reply_len+=BSP_UART_Fsus_Recv(reply+reply_len,(uint16_t)(sizeof(reply)-reply_len));
        for (unsigned i=0;i+21U<=reply_len;i++) {
            if (DrvFsus_ParseServoMonitor(reply+i,21,0,now,&fb)==FSUS_PARSE_OK) {
                query_pending=false;on_feedback(now);break;
            }
        }
        if (query_pending && now-poll_ms>=25U) query_pending=false;
    }
    if (query_pending || now-poll_ms<50U) return;
    uint8_t tx[FSUS_TX_BUF_SIZE];uint16_t n=DrvFsus_EncodeServoMonitor(tx,0);
    poll_ms=now;reply_len=0;
    if (BSP_UART_Fsus_EnsureRxArmed()!=ARA_OK) return;
    BSP_UART_Fsus_Flush();
    query_pending=BSP_UART_Fsus_Send(tx,n)==ARA_OK;
}
static void command(uint32_t now)
{
    if (line_overflow) { emit("[CAP] REJECT reason=line_too_long\r\n");return; }
    CaptureCommand_t parsed;
    CaptureCommandResult_t parsed_result=DrvCaptureCommand_Parse(line,line_len,&parsed);
    if (parsed_result!=CAP_CMD_OK) {
        char hex[65];
        for (unsigned i=0;i<line_len;i++) (void)snprintf(hex+i*2,3,"%02X",(unsigned char)line[i]);
        hex[line_len*2]=0;
        emit("[CAP] REJECT reason=bad_input error=%u bytes=%u hex=%s\r\n",parsed_result,line_len,hex);
        return;
    }
    if (parsed.key=='g' && !manual_mode) {
        request_id++;
        emit("[CAP] REQUEST request_id=%lu t_ms=%lu target_mdeg=%ld\r\n",
            (unsigned long)request_id,(unsigned long)now,(long)parsed.value*1000L);
    }
    if (parsed.key=='?') {
        emit("[CAP] HELP r 1=record_ON; r 0=record_OFF; h=zero_30deg_s; m=toggle; g <deg>; v <1..500>; a <20..2000>; w <0..5000>; x=stop; boot_record=OFF\r\n");return;
    }
    if (parsed.key=='x') {
        if (phase) finish("operator_stop",now,true);
        else {
            uint8_t tx[FSUS_TX_BUF_SIZE];uint16_t n=DrvFsus_EncodeStop(tx,0,FSUS_STOP_MODE_HOLD,0);
            (void)BSP_UART_Fsus_Send(tx,n);ready=false;ModStartupAngle_Reset(&gate);
        }
        return;
    }
    if (parsed.key=='r') {
        if (parsed.value!=0 && parsed.value!=1) { emit("[CAP] REJECT reason=record_requires_0_or_1\r\n");return; }
        bool enabled=parsed.value==1;
        if (phase) {
            mode_pending=true;pending_manual_mode=!enabled;
            emit("[CAP] RECORD_SWITCH record=%u after_current_move=1\r\n",enabled);
        } else {
            mode_pending=false;set_record_mode(enabled);
        }
        return;
    }
    if (phase) {
        if (parsed.key=='g' && manual_mode) emit("[CAP] MANUAL_REJECT reason=busy\r\n");
        else if (parsed.key=='g') emit("[CAP] REJECT request_id=%lu reason=busy\r\n",(unsigned long)request_id);
        else emit("[CAP] REJECT reason=busy\r\n");
        return;
    }
    if (parsed.key=='m') {
        set_record_mode(manual_mode);
        return;
    }
    if (parsed.key=='h') {
        if (!recovery_ready || !hx_feedback_healthy(now)) {
            emit("[CAP] RECOVERY_REJECT reason=unstable_or_unhealthy_feedback\r\n");return;
        }
        uint8_t tx[FSUS_TX_BUF_SIZE];
        uint16_t n=DrvFsus_EncodeSetAngleByVelocity(tx,0,0.0f,30.0f,500,500,0);
        query_pending=false;BSP_UART_Fsus_Flush();
        if (!n || BSP_UART_Fsus_Send(tx,n)!=ARA_OK) {
            emit("[CAP] RECOVERY_REJECT reason=tx_failed\r\n");return;
        }
        recovery_start=fb.angle_deg;recovery_active=true;manual_move_active=true;
        start_ms=BSP_UART_Fsus_GetLastTxStartMs();command_id++;target=0;
        phase=1;settled=0;previous_angle_valid=false;
        emit("[CAP] RECOVERY_START id=%lu t_ms=%lu from_mdeg=%ld target_mdeg=0 speed=30 acc_ms=500 dec_ms=500 record=OFF\r\n",
            (unsigned long)command_id,(unsigned long)start_ms,(long)(recovery_start*1000));
        return;
    }
    long value=parsed.value;
    if (parsed.key=='v' && value>=1 && value<=500) speed=(uint16_t)value;
    else if (parsed.key=='a' && value>=20 && value<=2000) ramp=(uint16_t)value;
    else if (parsed.key=='w' && value>=0 && value<=5000) post_ms=(uint32_t)value;
    else if (parsed.key=='g') {
        if (value<ARA_MAIN_ARM_ANGLE_MIN_DEG || value>ARA_MAIN_ARM_ANGLE_MAX_DEG) {
            if (manual_mode) emit("[CAP] MANUAL_REJECT reason=angle_limit\r\n");
            else emit("[CAP] REJECT request_id=%lu reason=angle_limit\r\n",(unsigned long)request_id);
            return;
        }
        if (!ready || !hx_healthy(now) || (!manual_mode && !imu_live(now))) {
            if (manual_mode) emit("[CAP] MANUAL_REJECT reason=not_ready hx8=%u\r\n",hx_healthy(now));
            else emit("[CAP] REJECT request_id=%lu reason=not_ready hx8=%u imu=%u\r\n",
                      (unsigned long)request_id,hx_healthy(now),imu_live(now));
            return;
        }
        uint8_t tx[FSUS_TX_BUF_SIZE];
        uint16_t n=DrvFsus_EncodeSetAngleByVelocity(tx,0,(float)value,(float)speed,ramp,ramp,0);
        query_pending=false;BSP_UART_Fsus_Flush();
        if (!n || BSP_UART_Fsus_Send(tx,n)!=ARA_OK) {
            if (manual_mode) emit("[CAP] MANUAL_REJECT reason=tx_failed\r\n");
            else emit("[CAP] REJECT request_id=%lu reason=tx_failed\r\n",(unsigned long)request_id);
            return;
        }
        start_ms=BSP_UART_Fsus_GetLastTxStartMs();
        command_id++;target=(float)value;settled=0;previous_angle_valid=false;phase=1;drop_baseline=drops;
        manual_move_active=manual_mode;
        checksum_baseline=imu.checksum_errors;
        uart_baseline=BSP_UART_RxDma_GetRecoveries();
        if (manual_mode)
            emit("[CAP] MANUAL_START id=%lu t_ms=%lu target_mdeg=%ld speed=%u acc_ms=%u dec_ms=%u record=OFF\r\n",
                 (unsigned long)command_id,(unsigned long)start_ms,value*1000,speed,ramp,ramp);
        else
            emit("[CAP] START id=%lu request_id=%lu t_ms=%lu tx_end_ms=%lu target_mdeg=%ld speed=%u acc_ms=%u dec_ms=%u post_ms=%lu imu_seq=%lu hx_seq=%lu\r\n",
                 (unsigned long)command_id,(unsigned long)request_id,(unsigned long)start_ms,(unsigned long)HAL_GetTick(),value*1000,
                 speed,ramp,ramp,(unsigned long)post_ms,(unsigned long)imu_seq,(unsigned long)hx_seq);
        return;
    } else { emit("[CAP] REJECT reason=bad_setting\r\n");return; }
    emit("[CAP] CONFIG speed=%u ramp_ms=%u post_ms=%lu\r\n",speed,ramp,(unsigned long)post_ms);
}
void App_Hx8Jy60Capture_Run(void)
{
    SEGGER_RTT_Init();
#if defined(JY61P_DIAGNOSTIC)
    (void)SEGGER_RTT_ConfigUpBuffer(0,"diagnostic",trace_buffer,sizeof(trace_buffer),SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    BSP_UART_Init();App_Jy61pDiagnostic_Run();
#else
    (void)SEGGER_RTT_ConfigUpBuffer(1,"capture",trace_buffer,sizeof(trace_buffer),SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    DrvJy60_Init(&imu);ModStartupAngle_Reset(&gate);BSP_UART_Init();
#if defined(JY61P_CAPTURE)
    emit("[CAP] BOOT firmware=HX8_JY61P_CAPTURE imu=JY61P baud=115200 requested_hz=200 record=0 pwm=OFF autohome=OFF\r\n");
    bool configured=imu_configure();
    emit("[CAP] IMU_CONFIG verified=%u requested_baud=115200 requested_hz=200\r\n",configured);
#else
    emit("[CAP] BOOT firmware=HX8_JY60_CAPTURE jy60=USART1_9600 hx8=USART2_115200 record=0 pwm=OFF autohome=OFF\r\n");
#endif
    uint32_t heartbeat=HAL_GetTick(),last_key_ms=0;
    for (;;) {
        (void)HAL_IWDG_Refresh(&hiwdg);
        uint32_t now=HAL_GetTick();uint8_t bytes[64];uint16_t count;
        while ((count=BSP_UART_Read(BSP_UART_ELRS,bytes,sizeof(bytes)))!=0) {
            for (unsigned i=0;i<count;i++) {
                if (!DrvJy60_Push(&imu,bytes[i],HAL_GetTick())) continue;
                const Jy60Vector_t *v=imu.last_type==0x51 ? &imu.acceleration : imu.last_type==0x52 ? &imu.gyro : &imu.angle;
                imu_seq++;
#if defined(JY61P_CAPTURE)
                /* Compact transport only: retain every value, type and timestamp. */
                emit("[I] %lu,%lu,%u,%ld,%ld,%ld,%d\r\n",
#else
                emit("[IMU] seq=%lu t_ms=%lu type=%u x=%ld y=%ld z=%ld tail=%d\r\n",
#endif
                    (unsigned long)imu_seq,(unsigned long)v->timestamp_ms,imu.last_type,
                    (long)v->milli[0],(long)v->milli[1],(long)v->milli[2],v->tail_raw);
            }
        }
        monitor(now);
        uint8_t keys[32];unsigned n=SEGGER_RTT_Read(0,keys,sizeof(keys));
        for (unsigned i=0;i<n;i++) {
            if (!keys[i]) continue;
            uint32_t key_ms=HAL_GetTick();
            if ((line_len || line_overflow) && key_ms-last_key_ms>1000U) {
                line_len=0;line_overflow=false;emit("[CAP] INPUT partial_line_expired\r\n");
            }
            last_key_ms=key_ms;
            if (keys[i]=='\n' || keys[i]=='\r') {
                if (line_len || line_overflow) { line[line_len]=0;command(HAL_GetTick()); }
                line_len=0;line_overflow=false;
            } else if (line_len<sizeof(line)-1) line[line_len++]=(char)keys[i];
            else line_overflow=true;
        }
        now=HAL_GetTick();
        if (phase && (!hx_healthy(now) || (!manual_move_active && !imu_live(now)))) finish("feedback_timeout_or_fault",now,true);
        else if (phase==1 && now-start_ms>10000U) finish("motion_timeout",now,true);
        else if (phase==2 && now-arrived_ms>=post_ms) finish("complete",now,false);
        if (now-heartbeat>=1000U) {
#if defined(JY61P_CAPTURE)
            uint32_t elapsed=now-heartbeat;
            uint32_t acc_hz=(imu.acceleration.count-rate_acc)*1000U/elapsed;
            uint32_t gyro_hz=(imu.gyro.count-rate_gyro)*1000U/elapsed;
            uint32_t angle_hz=(imu.angle.count-rate_angle)*1000U/elapsed;
            imu_rate_ok=acc_hz>=180U && acc_hz<=220U && gyro_hz>=180U && gyro_hz<=220U && angle_hz>=180U && angle_hz<=220U;
            rate_acc=imu.acceleration.count;rate_gyro=imu.gyro.count;rate_angle=imu.angle.count;
            emit("[CAP] IMU_RATE acc_hz=%lu gyro_hz=%lu angle_hz=%lu rate_ok=%u period_ms=%lu discarded=%lu\r\n",
                (unsigned long)acc_hz,(unsigned long)gyro_hz,(unsigned long)angle_hz,imu_rate_ok,
                (unsigned long)imu.angle_period_ms,(unsigned long)imu.discarded_bytes);
#endif
            heartbeat=now;
            emit("[CAP] READY t_ms=%lu mode=%s record=%u ready=%u recovery_ready=%u phase=%u hx8=%u imu=%u angle_mdeg=%ld voltage=%d hx_tx=%lu hx_rx=%lu checksum_err=%lu drops_total=%lu\r\n",
                (unsigned long)now,manual_mode ? "MANUAL_HX8" : "CAPTURE",!manual_mode,
                !phase&&ready&&hx_healthy(now)&&(manual_mode||imu_live(now)),!phase&&recovery_ready&&hx_feedback_healthy(now),phase,
                hx_healthy(now),imu_live(now),have_fb ? (long)(fb.angle_deg*1000) : 0L,
                have_fb ? fb.voltage_mv : 0,(unsigned long)BSP_UART_Fsus_GetTxBytes(),
                (unsigned long)BSP_UART_Fsus_GetRxBytes(),(unsigned long)imu.checksum_errors,(unsigned long)drops);
        }
    }
#endif
}
#endif
