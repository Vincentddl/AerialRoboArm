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
#if defined(HC13_HOST_LINK)
#include "mod_hc13_host.h"
#include "app_hc13_host_config.h"
#include "tim.h"
static Hc13Host_t host;
static bool host_target_valid,host_pwm_started;
static uint16_t host_acc_hz;
static uint32_t host_rx_overflows;
static uint32_t host_settled_since_ms;
static uint32_t host_follow_tx_ms;
static bool host_follow_tx_ever;
static bool host_following_grace(uint32_t now)
{ return host.following || (host_follow_tx_ever && now-host_follow_tx_ms<500U); }
enum { CAPTURE_PRE_MS=250, CAPTURE_MOTION_MS=1750, CAPTURE_PRE_ROWS=52 };
enum { CAPTURE_LIMIT=1, CAPTURE_IMU_IO=2, CAPTURE_IMU_STALE=4 };
static struct {
    uint8_t accel[HOST_CAPTURE_ACC_LIMIT][HOST_CAPTURE_ACC_BYTES];
    uint8_t angles[HOST_CAPTURE_HX_LIMIT][HOST_CAPTURE_HX_BYTES];
    uint8_t pre[CAPTURE_PRE_ROWS][HOST_CAPTURE_ACC_BYTES];
    uint32_t id, start, end, settle_check_ms;
    uint16_t acc_count, hx_count;
    uint8_t pre_next, pre_count, flags;
    bool active, sealed;
} wireless_capture;

static void capture_word(uint8_t *bytes,uint32_t value,unsigned size)
{
    for (unsigned i=0;i<size;i++) bytes[i]=(uint8_t)(value>>(8U*i));
}

static void capture_acc(uint32_t at,const int32_t *axes,bool idle)
{
    uint8_t *row;
    if (wireless_capture.active) {
        if (at-wireless_capture.start>CAPTURE_MOTION_MS ||
            wireless_capture.acc_count>=HOST_CAPTURE_ACC_LIMIT) {
            wireless_capture.flags|=CAPTURE_LIMIT;
            return;
        }
        row=wireless_capture.accel[wireless_capture.acc_count++];
    } else {
        if (!idle) { wireless_capture.pre_count=0;return; }
        row=wireless_capture.pre[wireless_capture.pre_next];
        wireless_capture.pre_next=(wireless_capture.pre_next+1U)%CAPTURE_PRE_ROWS;
        if (wireless_capture.pre_count<CAPTURE_PRE_ROWS) wireless_capture.pre_count++;
    }
    capture_word(row,at,2U);
    for (unsigned i=0;i<3U;i++) capture_word(row+2U+2U*i,(uint32_t)axes[i],2U);
}

static void capture_angle(uint32_t at,int32_t angle_mdeg)
{
    if (wireless_capture.hx_count>=HOST_CAPTURE_HX_LIMIT) {
        wireless_capture.flags|=CAPTURE_LIMIT;
        return;
    }
    uint8_t *row=wireless_capture.angles[wireless_capture.hx_count++];
    capture_word(row,at,2U);
    capture_word(row+2, (uint32_t)angle_mdeg,4U);
}

static bool capture_read(uint32_t id,uint16_t page,HostCapturePage_t *out)
{
    if (!wireless_capture.sealed || wireless_capture.id!=id) return false;
    const uint32_t accel_bytes=wireless_capture.acc_count*HOST_CAPTURE_ACC_BYTES;
    const uint32_t total=accel_bytes+wireless_capture.hx_count*HOST_CAPTURE_HX_BYTES;
    const uint32_t offset=(uint32_t)page*HOST_CAPTURE_PAGE_BYTES;
    if (page && offset>=total) return false;
    out->start_ms=wireless_capture.start;
    out->end_ms=wireless_capture.end;
    out->acc_count=wireless_capture.acc_count;
    out->hx_count=wireless_capture.hx_count;
    out->flags=wireless_capture.flags;
    out->length=0;
    const uint8_t *accel=(const uint8_t *)&wireless_capture.accel;
    const uint8_t *angles=(const uint8_t *)&wireless_capture.angles;
    for (uint32_t i=offset;i<total && out->length<HOST_CAPTURE_PAGE_BYTES;i++)
        out->data[out->length++]=i<accel_bytes?accel[i]:angles[i-accel_bytes];
    return true;
}
#endif

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
enum { IMU_STALE_MS=250, IMU_REARM_INTERVAL_MS=1000 };
static bool imu_recovering;
static uint32_t imu_last_rearm_ms,imu_rate_start_ms;
static uint8_t imu_rearm_attempts,imu_scan_index;
static uint32_t imu_recovery_step_ms;
typedef enum { IMU_REC_IDLE, IMU_REC_SCAN, IMU_REC_BAUD_CONFIRM, IMU_REC_RATE_CONFIRM } ImuRecoveryStage_t;
static ImuRecoveryStage_t imu_recovery_stage;
static const uint32_t imu_bauds[]={115200,9600,230400,57600,38400,19200,4800};
/* About 45 kB/s of text at 600 IMU packets/s: buffer short host stalls. */
#if defined(HC13_HOST_LINK)
static char trace_buffer[4096]; /* Leave RAM for the host response/cache queues. */
#else
static char trace_buffer[8192];
#endif
static bool imu_rate_ok;
static uint32_t rate_acc,rate_gyro,rate_angle;
static void emit(const char *fmt,...);

static void imu_reset_parser(uint32_t now)
{
    DrvJy60_Init(&imu);
    rate_acc=rate_gyro=rate_angle=0;
    imu_rate_start_ms=now;
#if defined(HC13_HOST_LINK)
    host_acc_hz=0;
#endif
}

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
    imu_delay(1200); /* Manufacturer specifies 1000 ms sensor startup. */
    for (unsigned b=0;b<sizeof(imu_bauds)/sizeof(imu_bauds[0]);b++) {
        if (BSP_UART_ImuSetBaud(imu_bauds[b])!=ARA_OK) continue;
        if (!imu_probe(700,3)) continue;
        emit("[CAP] IMU_LINK baud=%lu good=%lu\r\n",(unsigned long)imu_bauds[b],(unsigned long)imu.good_frames);
        if (!imu_write_register(0x69,0xB588) || !imu_write_register(0x04,0x0006)) return false;
        if (BSP_UART_ImuSetBaud(115200)!=ARA_OK) return false;
        if (!imu_probe(700,3)) {
            emit("[CAP] IMU_CONFIG_FAILED reason=baud_not_confirmed fallback_baud=%lu check_MCU_TX_to_IMU_RX=1\r\n",(unsigned long)imu_bauds[b]);
            (void)BSP_UART_ImuSetBaud(imu_bauds[b]);DrvJy60_Init(&imu);return false;
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
    return !imu_recovering && imu_rate_ok && imu.acceleration.count && imu.angle.count && imu.gyro.count &&
        now-imu.acceleration.timestamp_ms<50U && now-imu.angle.timestamp_ms<50U && now-imu.gyro.timestamp_ms<50U;
#else
    return imu.angle.count && imu.gyro.count && now-imu.angle.timestamp_ms<200U && now-imu.gyro.timestamp_ms<200U;
#endif
}
#if defined(JY61P_CAPTURE)
static bool imu_stream_stale(uint32_t now)
{
    return !imu.acceleration.count || now-imu.acceleration.timestamp_ms>=IMU_STALE_MS ||
           !imu.gyro.count || now-imu.gyro.timestamp_ms>=IMU_STALE_MS ||
           !imu.angle.count || now-imu.angle.timestamp_ms>=IMU_STALE_MS;
}

static void imu_poll_recovery(uint32_t now)
{
    if (imu_recovery_stage==IMU_REC_IDLE && !imu_stream_stale(now) &&
        imu_rate_ok && !imu_recovering) {
        imu_rearm_attempts=0;
        return;
    }
#if defined(HC13_HOST_LINK)
    /* Keep the indication compatible with hosts that only know CAPTURE_IMU_IO. */
    if (wireless_capture.active) wireless_capture.flags|=CAPTURE_IMU_IO|CAPTURE_IMU_STALE;
#endif
    /* Never change USART1 settings during a movement or an active capture. */
    if (phase
#if defined(HC13_HOST_LINK)
        || wireless_capture.active || host.active || host.following
#endif
       ) return;
    if (imu_recovery_stage!=IMU_REC_IDLE) {
        if (imu_recovery_stage==IMU_REC_SCAN) {
            if (!imu_recovery_step_ms) {
                if (imu_scan_index>=sizeof(imu_bauds)/sizeof(imu_bauds[0])) {
                    emit("[CAP] IMU_SCAN_FAILED rx_bytes=%lu good=%lu checksum=%lu uart_errors=%lu\r\n",
                         (unsigned long)BSP_UART_Elrs_GetRxBytes(),(unsigned long)imu.good_frames,
                         (unsigned long)imu.checksum_errors,(unsigned long)BSP_UART_ImuGetRxErrors());
                    imu_scan_index=0;
                }
                if (BSP_UART_ImuSetBaud(imu_bauds[imu_scan_index])!=ARA_OK) {
                    imu_scan_index++;
                    return;
                }
                imu_reset_parser(now);
                imu_recovery_step_ms=now;
                return;
            }
            if (now-imu_recovery_step_ms<700U) return;
            if (imu.good_frames<3U) {
                imu_scan_index++;
                imu_recovery_step_ms=0;
                return;
            }
            emit("[CAP] IMU_SCAN_LINK baud=%lu good=%lu\r\n",
                 (unsigned long)imu_bauds[imu_scan_index],(unsigned long)imu.good_frames);
            if (!imu_write_register(0x69,0xB588) || !imu_write_register(0x04,0x0006) ||
                BSP_UART_ImuSetBaud(115200)!=ARA_OK) {
                imu_scan_index++;
                imu_recovery_step_ms=0;
                return;
            }
            now=HAL_GetTick();
            imu_reset_parser(now);
            imu_recovery_step_ms=now;
            imu_recovery_stage=IMU_REC_BAUD_CONFIRM;
            return;
        }
        if (imu_recovery_stage==IMU_REC_BAUD_CONFIRM) {
            if (now-imu_recovery_step_ms<700U) return;
            if (imu.good_frames<3U ||
                !imu_write_register(0x69,0xB588) ||
                !imu_write_register(0x02,0x000E) ||
                !imu_write_register(0x03,0x000B) ||
                !imu_write_register(0x1F,0x0002) ||
                BSP_UART_ImuSetBaud(115200)!=ARA_OK) {
                imu_scan_index++;
                imu_recovery_step_ms=0;
                imu_recovery_stage=IMU_REC_SCAN;
                return;
            }
            now=HAL_GetTick();
            imu_reset_parser(now);
            imu_recovery_step_ms=now;
            imu_recovery_stage=IMU_REC_RATE_CONFIRM;
            return;
        }
        if (now-imu_recovery_step_ms<1100U) return;
        uint32_t elapsed=now-imu_recovery_step_ms;
        uint32_t acc_hz=imu.acceleration.count*1000U/elapsed;
        uint32_t gyro_hz=imu.gyro.count*1000U/elapsed;
        uint32_t angle_hz=imu.angle.count*1000U/elapsed;
        if (acc_hz>=180U && acc_hz<=220U && gyro_hz>=180U && gyro_hz<=220U &&
            angle_hz>=180U && angle_hz<=220U &&
            !imu_stream_stale(now)) {
            imu_rate_ok=true;
            imu_recovering=false;
            imu_rearm_attempts=0;
            imu_recovery_stage=IMU_REC_IDLE;
#if defined(HC13_HOST_LINK)
            host_acc_hz=(uint16_t)acc_hz;
#endif
            emit("[CAP] IMU_RECONFIGURED baud=115200 acc=%lu gyro=%lu angle=%lu\r\n",
                 (unsigned long)imu.acceleration.count,(unsigned long)imu.gyro.count,
                 (unsigned long)imu.angle.count);
        } else {
            emit("[CAP] IMU_RATE_RETRY acc=%lu gyro=%lu angle=%lu rx_bytes=%lu\r\n",
                 (unsigned long)imu.acceleration.count,(unsigned long)imu.gyro.count,
                 (unsigned long)imu.angle.count,(unsigned long)BSP_UART_Elrs_GetRxBytes());
            imu_scan_index=0;
            imu_recovery_step_ms=0;
            imu_recovery_stage=IMU_REC_SCAN;
        }
        rate_acc=imu.acceleration.count;rate_gyro=imu.gyro.count;rate_angle=imu.angle.count;
        imu_rate_start_ms=now;
        return;
    }
    if (now-imu_last_rearm_ms<IMU_REARM_INTERVAL_MS) return;
    imu_last_rearm_ms=now;
    imu_recovering=true;
    imu_rate_ok=false;
    if (++imu_rearm_attempts>=3U) {
        imu_recovery_stage=IMU_REC_SCAN;
        imu_scan_index=0;
        imu_recovery_step_ms=0;
        emit("[CAP] IMU_SCAN_START rx_bytes=%lu checksum=%lu uart_errors=%lu\r\n",
             (unsigned long)BSP_UART_Elrs_GetRxBytes(),(unsigned long)imu.checksum_errors,
             (unsigned long)BSP_UART_ImuGetRxErrors());
        return;
    }
    /* Discard only the partial frame. Preserve sequence and error counters. */
    imu.used=0;
    rate_acc=imu.acceleration.count;rate_gyro=imu.gyro.count;rate_angle=imu.angle.count;
    imu_rate_start_ms=now;
    AraStatus_t result=BSP_UART_ImuRestartRx();
    emit("[CAP] IMU_REARM t_ms=%lu result=%s uart_errors=%lu rearm_failures=%lu checksum_err=%lu\r\n",
         (unsigned long)now,result==ARA_OK?"armed":"failed",(unsigned long)BSP_UART_ImuGetRxErrors(),
         (unsigned long)BSP_UART_ImuGetRearmFailures(),(unsigned long)imu.checksum_errors);
}
#endif
static bool hx_feedback_healthy(uint32_t now)
{
    /* Fast HX8 motion may delay one monitor reply. Keep a bounded running
     * grace; admission to a new move still requires a <200 ms sample. */
    bool moving=phase!=0U
#if defined(HC13_HOST_LINK)
        || host_following_grace(now)
#endif
        ;
    return have_fb && now-last_fb_ms<(moving ? 400U : 200U) && isfinite(fb.angle_deg) &&
        fb.angle_deg>=-180.0f && fb.angle_deg<=180.0f &&
        fb.voltage_mv>=9000 && fb.voltage_mv<=12600 && !(fb.status&0xFEU);
}
#if defined(HC13_HOST_LINK)
_Static_assert(HOST_MAX_ANGLE_MDEG == ARA_MAIN_ARM_ANGLE_MAX_DEG * 1000L,
               "HC13 and HX8 angle limits must agree");
#endif
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
#if defined(HC13_HOST_LINK)
    if (wireless_capture.active) {
        wireless_capture.end=now;
        if (now-wireless_capture.start>CAPTURE_MOTION_MS) wireless_capture.flags|=CAPTURE_LIMIT;
        if (imu.checksum_errors!=checksum_baseline ||
            BSP_UART_RxDma_GetRecoveries()!=uart_baseline) wireless_capture.flags|=CAPTURE_IMU_IO;
        wireless_capture.active=false;
        wireless_capture.sealed=true;
    }
    HostError_t host_reason=!strcmp(reason,"complete")?HOST_OK:!strcmp(reason,"operator_stop")?HOST_STOPPED:
        !strcmp(reason,"motion_timeout")?HOST_MOTION_TIMEOUT:!strcmp(reason,"tx_failed")?HOST_TX_FAILED:HOST_FEEDBACK_STALE;
    ModHc13Host_MotionFinished(&host,host_reason,now,(int32_t)lroundf(fb.angle_deg*1000.0f));
#endif
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
#if defined(HC13_HOST_LINK)
    if (wireless_capture.active && now-wireless_capture.start<=CAPTURE_MOTION_MS)
        capture_angle(now,(int32_t)lroundf(fb.angle_deg*1000.0f));
#endif
    emit("[HX8] seq=%lu t_ms=%lu angle_mdeg=%ld voltage=%d current=%d status=0x%02X\r\n",
        (unsigned long)hx_seq,(unsigned long)now,(long)(fb.angle_deg*1000),fb.voltage_mv,fb.current_ma,fb.status);
    if (!phase) {
        ready=ModStartupAngle_Update(&gate,&fb,true,true,now);
        recovery_ready=ModStartupAngle_UpdateRecovery(&recovery_gate,&fb,true,true,now);
        return;
    }
    if (!hx_healthy(now)) { finish("hx8_fault",now,true);return; }
    if (phase!=1) return;
#if defined(HC13_HOST_LINK)
    /* Faster observation must not shorten or relax the existing stop criteria. */
    if (wireless_capture.active) {
        if (now-wireless_capture.settle_check_ms<50U) return;
        wireless_capture.settle_check_ms=now;
    }
#endif
    bool quiet=previous_angle_valid && fabsf(fb.angle_deg-previous_angle)<=0.3f;
    previous_angle=fb.angle_deg;previous_angle_valid=true;
    float settle_tolerance_deg=recovery_active ? 0.5f : 1.0f;
#if defined(HC13_HOST_LINK)
    if (host.flow==HOST_FLOW_PICK)
        settle_tolerance_deg=(float)HOST_PICK_SETTLE_TOLERANCE_MDEG/1000.0f;
#endif
    if (fabsf(fb.angle_deg-target)<=settle_tolerance_deg && quiet) {
#if defined(HC13_HOST_LINK)
        if (!settled) host_settled_since_ms=now;
#endif
        if (settled<5) settled++;
    }
    else settled=0;
    /* Never count cached values. Faster PICK polling must not shorten the
     * historical ~200 ms quiet confirmation interval to merely 20 ms. */
    bool confirmed=settled>=5;
#if defined(HC13_HOST_LINK)
    if (host.flow==HOST_FLOW_PICK) confirmed=confirmed && now-host_settled_since_ms>=200U;
#endif
    if (confirmed) {
        phase=2;arrived_ms=now;
        emit("[CAP] %s id=%lu t_ms=%lu angle_mdeg=%ld\r\n",recovery_active ? "RECOVERY_ARRIVED" : manual_move_active ? "MANUAL_ARRIVED" : "ARRIVED",
             (unsigned long)command_id,(unsigned long)now,(long)(fb.angle_deg*1000));
#if defined(HC13_HOST_LINK)
        ModHc13Host_MotionArrived(&host,now,(int32_t)lroundf(fb.angle_deg*1000.0f));
#endif
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
    uint32_t period=50U;
#if defined(HC13_HOST_LINK)
    if (phase==1U) period=ModHc13Host_FeedbackPeriod(&host);
    if (wireless_capture.active && now-wireless_capture.start<=CAPTURE_MOTION_MS) period=10U;
    if (host_follow_tx_ever && now-host_follow_tx_ms<12U) return;
#endif
    if (query_pending || now-poll_ms<period) return;
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
#if defined(HC13_HOST_LINK)
static void host_snapshot(HostSnapshot_t *s,uint32_t now)
{
    s->hx_have_feedback=have_fb;
    s->hx_feedback_age_ms=have_fb?now-last_fb_ms:UINT32_MAX;
    s->hx_last_angle_valid=have_fb&&isfinite(fb.angle_deg);
    s->hx_last_pos_mdeg=s->hx_last_angle_valid?(int32_t)lroundf(fb.angle_deg*1000.0f):0;
    s->hx_last_voltage_mv=have_fb?(uint16_t)fb.voltage_mv:0;
    s->hx_last_current_ma=have_fb?(int16_t)fb.current_ma:0;
    s->hx_last_status=have_fb?fb.status:0;
    s->hx_online=have_fb && now-last_fb_ms<((phase || host_following_grace(now)) ? 400U : 200U) && isfinite(fb.angle_deg);
    s->hx_healthy=hx_feedback_healthy(now);s->ready=ready;s->recovery_ready=recovery_ready;
    s->imu_online=imu.acceleration.count && now-imu.acceleration.timestamp_ms<250U;
    s->imu_ready=imu_live(now);s->record_on=!manual_mode;s->target_valid=host_target_valid;
    s->phase=phase;s->pos_mdeg=s->hx_online?(int32_t)lroundf(fb.angle_deg*1000.0f):0;
    s->target_mdeg=(int32_t)lroundf(target*1000.0f);s->voltage_mv=s->hx_online?(uint16_t)fb.voltage_mv:0;
    s->imu_hz=imu_recovering?0U:host_acc_hz;s->acc_count=imu.acceleration.count;s->acc_ms=imu.acceleration.timestamp_ms;
    s->imu_rx_bytes=BSP_UART_Elrs_GetRxBytes();s->imu_good_frames=imu.good_frames;
    s->imu_checksum_errors=imu.checksum_errors;s->imu_uart_errors=BSP_UART_ImuGetRxErrors();
    s->imu_rearm_failures=BSP_UART_ImuGetRearmFailures();s->imu_recovery_stage=(uint8_t)imu_recovery_stage;
    s->motion_start_ms=start_ms;
    s->feedback_seq=hx_seq;
    for (unsigned i=0;i<3U;i++) s->acc_mg[i]=imu.acceleration.milli[i];
}
static HostError_t host_stop(uint32_t now)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];uint16_t n=DrvFsus_EncodeStop(tx,0,FSUS_STOP_MODE_HOLD,0);
    bool sent=n && BSP_UART_Fsus_Send(tx,n)==ARA_OK;
    if (phase) finish(sent?"operator_stop":"tx_failed",now,false);
    ready=false;ModStartupAngle_Reset(&gate);
    if (!sent) return HOST_TX_FAILED;
    return hx_feedback_healthy(now)?HOST_OK:HOST_HX8_OFFLINE;
}
static HostError_t host_servo(int32_t mdeg)
{
    if (mdeg<0 || mdeg>180000 || mdeg%1000) return HOST_OUT_OF_RANGE;
    uint16_t degrees=(uint16_t)(mdeg/1000);
    uint16_t pulse=degrees<=90U ? (uint16_t)(500U+degrees*34U/3U) :
        (uint16_t)(1520U+(degrees-90U)*98U/9U);
    /* Load the requested pulse BEFORE starting the first PWM frame. */
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pulse);
    if (!host_pwm_started) {
        if (HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1)!=HAL_OK) return HOST_FAULT;
        host_pwm_started=true;
    }
    return HOST_OK;
}
static HostError_t host_record(bool enabled)
{
    if (phase) return HOST_BUSY;
    manual_mode=!enabled;mode_pending=false;
    emit("[CAP] MODE mode=%s record=%u imu_required=%u action=host_record_switch\r\n",
        enabled?"CAPTURE":"MANUAL_HX8",enabled,enabled);
    return HOST_OK;
}
static bool host_send(const char *text,size_t length)
{ return BSP_UART_HC13_TrySend((const uint8_t *)text,(uint16_t)length)==ARA_OK; }
static HostError_t host_move_with_ramp(int32_t mdeg,uint16_t velocity,HostVerb_t kind,uint16_t requested_ramp,uint32_t now)
{
    bool home=kind==HOST_HOME,pick=kind==HOST_PICK;
    if (kind!=HOST_MOVE && kind!=HOST_HOME && kind!=HOST_SHAPE && kind!=HOST_PICK) return HOST_UNSUPPORTED;
    if (phase) return HOST_BUSY;
    if (!hx_feedback_healthy(now)) return HOST_HX8_OFFLINE;
    if (!home && (mdeg< -90000 || mdeg>ARA_MAIN_ARM_ANGLE_MAX_DEG*1000L)) return HOST_OUT_OF_RANGE;
    if (!velocity || velocity>500U) return HOST_OUT_OF_RANGE;
    if (home ? !recovery_ready : (!pick && !ready)) return HOST_NOT_READY;
    if (!home && !hx_healthy(now)) return HOST_NOT_READY;
    if (!home && kind!=HOST_SHAPE && !manual_mode && !imu_live(now)) return HOST_IMU_RATE_LOW;
    uint16_t motion_ramp=(home || kind==HOST_SHAPE)?500U:requested_ramp;
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint16_t n=DrvFsus_EncodeSetAngleByVelocity(tx,0,(float)mdeg/1000.0f,(float)velocity,motion_ramp,motion_ramp,0);
    query_pending=false;BSP_UART_Fsus_Flush();
    if (!n || BSP_UART_Fsus_Send(tx,n)!=ARA_OK) return HOST_TX_FAILED;
    start_ms=BSP_UART_Fsus_GetLastTxStartMs();target=(float)mdeg/1000.0f;host_target_valid=true;
    command_id++;settled=0;previous_angle_valid=false;phase=1;
    recovery_active=home;recovery_start=fb.angle_deg;
    manual_move_active=manual_mode || home || kind==HOST_SHAPE;drop_baseline=drops;checksum_baseline=imu.checksum_errors;
    uart_baseline=BSP_UART_RxDma_GetRecoveries();
    if (home) emit("[CAP] RECOVERY_START id=%lu t_ms=%lu from_mdeg=%ld target_mdeg=%ld speed=30 acc_ms=500 dec_ms=500 record=OFF\r\n",
        (unsigned long)command_id,(unsigned long)start_ms,(long)(fb.angle_deg*1000),(long)mdeg);
    else if (manual_mode) emit("[CAP] MANUAL_START id=%lu t_ms=%lu target_mdeg=%ld speed=%u acc_ms=%u dec_ms=%u record=OFF\r\n",
        (unsigned long)command_id,(unsigned long)start_ms,(long)mdeg,velocity,motion_ramp,motion_ramp);
    else {
        request_id++;
        emit("[CAP] REQUEST request_id=%lu t_ms=%lu target_mdeg=%ld\r\n",(unsigned long)request_id,(unsigned long)start_ms,(long)mdeg);
        emit("[CAP] START id=%lu request_id=%lu t_ms=%lu tx_end_ms=%lu target_mdeg=%ld speed=%u acc_ms=%u dec_ms=%u post_ms=%lu imu_seq=%lu hx_seq=%lu\r\n",
            (unsigned long)command_id,(unsigned long)request_id,(unsigned long)start_ms,(unsigned long)HAL_GetTick(),(long)mdeg,
            velocity,motion_ramp,motion_ramp,(unsigned long)post_ms,(unsigned long)imu_seq,(unsigned long)hx_seq);
    }
    return HOST_OK;
}
static HostError_t host_move(int32_t mdeg,uint16_t velocity,HostVerb_t kind,uint32_t now)
{ return host_move_with_ramp(mdeg,velocity,kind,ramp,now); }
static HostError_t host_follow_move(int32_t mdeg,uint16_t velocity,uint32_t now)
{
    if (phase) return HOST_BUSY;
    if (!hx_feedback_healthy(now)) return HOST_HX8_OFFLINE;
    if (mdeg< -90000 || mdeg>ARA_MAIN_ARM_ANGLE_MAX_DEG*1000L) return HOST_OUT_OF_RANGE;
    if (!velocity || velocity>500U) return HOST_OUT_OF_RANGE;
    if (!hx_healthy(now)) return HOST_NOT_READY;
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint16_t n=DrvFsus_EncodeSetAngleByVelocity(tx,0,(float)mdeg/1000.0f,(float)velocity,ramp,ramp,0);
    query_pending=false;BSP_UART_Fsus_Flush();
    if (!n || BSP_UART_Fsus_Send(tx,n)!=ARA_OK) return HOST_TX_FAILED;
    host_follow_tx_ms=now;host_follow_tx_ever=true;
    target=(float)mdeg/1000.0f;host_target_valid=true;
    ready=false;ModStartupAngle_Reset(&gate);
    return HOST_OK;
}
static HostError_t host_experiment_move(int32_t mdeg,uint16_t velocity,uint16_t ramp_ms,uint32_t now)
{
    if (ramp_ms<20U || ramp_ms>2000U) return HOST_OUT_OF_RANGE;
    return host_move_with_ramp(mdeg,velocity,HOST_MOVE,ramp_ms,now);
}
static void capture_begin(uint32_t id)
{
    wireless_capture.id=id;
    wireless_capture.start=start_ms;
    wireless_capture.end=start_ms;
    wireless_capture.flags=0;
    wireless_capture.acc_count=0;
    wireless_capture.hx_count=0;
    wireless_capture.sealed=false;
    wireless_capture.settle_check_ms=start_ms;
    unsigned first=(wireless_capture.pre_next+CAPTURE_PRE_ROWS-wireless_capture.pre_count)%CAPTURE_PRE_ROWS;
    for (unsigned i=0;i<wireless_capture.pre_count;i++) {
        const uint8_t *row=wireless_capture.pre[(first+i)%CAPTURE_PRE_ROWS];
        uint16_t at=(uint16_t)(row[0]|((unsigned)row[1]<<8));
        uint16_t age=(uint16_t)((uint16_t)start_ms-at);
        if (age>0U && age<=CAPTURE_PRE_MS)
            memcpy(wireless_capture.accel[wireless_capture.acc_count++],row,HOST_CAPTURE_ACC_BYTES);
    }
    wireless_capture.pre_count=0;
    if (have_fb && start_ms-last_fb_ms<=CAPTURE_PRE_MS)
        capture_angle(last_fb_ms,(int32_t)lroundf(fb.angle_deg*1000.0f));
    wireless_capture.active=true;
}
static uint32_t host_boot_id(void)
{
    /* Backup counter survives MCU resets; cycle timing adds variation on cold
     * starts. This is a session identifier, not a cryptographic random number. */
    __HAL_RCC_PWR_CLK_ENABLE();__HAL_RCC_BKP_CLK_ENABLE();HAL_PWR_EnableBkUpAccess();
    uint32_t counter=((BKP->DR2&0xFFFFU)<<16)|(BKP->DR1&0xFFFFU);counter++;
    BKP->DR1=counter&0xFFFFU;BKP->DR2=counter>>16;HAL_PWR_DisableBkUpAccess();
    const uint32_t *uid=(const uint32_t *)0x1FFFF7E8UL;
    return (counter*2654435761UL)^uid[0]^DWT->CYCCNT^HAL_GetTick();
}
static void host_init(void)
{
    const HostConfig_t cfg={.vertical_mdeg=HOST_VERTICAL_MDEG,
        .folded_mdeg=HOST_FOLD_MDEG,.grip_open_mdeg=HOST_GRIP_OPEN_MDEG,.grip_close_mdeg=HOST_GRIP_CLOSE_MDEG,
        .fold_calibrated=HOST_FOLD_CALIBRATED,.grip_calibrated=HOST_GRIP_CONFIGURED,
        .open_wait_calibrated=HOST_OPEN_WAIT_MS>=0,.close_wait_calibrated=HOST_CLOSE_WAIT_MS>=0,
        .open_wait_ms=HOST_OPEN_WAIT_MS>=0?HOST_OPEN_WAIT_MS:0,.close_wait_ms=HOST_CLOSE_WAIT_MS>=0?HOST_CLOSE_WAIT_MS:0,
        .default_lead_mdeg=HOST_DEFAULT_LEAD_MDEG,
        .pc_working_mdeg=HOST_PC_WORKING_MDEG,.pc_folded_mdeg=HOST_PC_FOLDED_MDEG,
        .extension_sign=HOST_EXTENSION_SIGN,.max_extension_mdeg=HOST_MAX_EXTENSION_MDEG,
        .max_speed=500,.ramp_ms=100,.post_ms=1000,.min_confidence=HOST_VISION_MIN_CONFIDENCE};
    const HostHooks_t hooks={.snapshot=host_snapshot,.move=host_move,.follow_move=host_follow_move,.experiment_move=host_experiment_move,
        .capture_begin=capture_begin,.capture_read=capture_read,.stop=host_stop,.servo=host_servo,
        .record=host_record,.send=host_send};
    ModHc13Host_Init(&host,&cfg,&hooks,host_boot_id(),HAL_GetTick());
    /* Discard commands accumulated during sensor startup/configuration.
     * The PC queries again; an old queued actuator request must not start here. */
    uint8_t discarded[64];
    while (BSP_UART_HC13_ReadRaw(discarded,sizeof(discarded))) { }
    host_rx_overflows=BSP_UART_HC13_GetRxOverflows();
}
#endif
void App_Hx8Jy60Capture_Run(void)
{
    SEGGER_RTT_Init();
#if defined(JY61P_DIAGNOSTIC)
    (void)SEGGER_RTT_ConfigUpBuffer(0,"diagnostic",trace_buffer,sizeof(trace_buffer),SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    BSP_UART_Init();App_Jy61pDiagnostic_Run();
#else
    (void)SEGGER_RTT_ConfigUpBuffer(1,"capture",trace_buffer,sizeof(trace_buffer),SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    DrvJy60_Init(&imu);ModStartupAngle_Reset(&gate);BSP_UART_Init();
#if defined(HC13_HOST_LINK)
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    emit("[CAP] HOST build=HC13_V2 usart3=115200 pwm=DEFERRED rtt=READONLY\r\n");
#endif
#if defined(JY61P_CAPTURE)
    emit("[CAP] BOOT firmware=HX8_JY61P_CAPTURE imu=JY61P baud=115200 requested_hz=200 record=0 pwm=OFF autohome=OFF\r\n");
    bool configured=imu_configure();
    imu_recovering=!configured;
    emit("[CAP] IMU_CONFIG verified=%u requested_baud=115200 requested_hz=200\r\n",configured);
#if defined(HC13_HOST_LINK)
    host_init();
#endif
#else
    emit("[CAP] BOOT firmware=HX8_JY60_CAPTURE jy60=USART1_9600 hx8=USART2_115200 record=0 pwm=OFF autohome=OFF\r\n");
#endif
    uint32_t heartbeat=HAL_GetTick(),last_key_ms=0;
#if defined(JY61P_CAPTURE)
    imu_rate_start_ms=heartbeat;
    imu_last_rearm_ms=heartbeat; /* Allow the first full rate window after startup configuration. */
#endif
    for (;;) {
        (void)HAL_IWDG_Refresh(&hiwdg);
        uint32_t now=HAL_GetTick();uint8_t bytes[64];uint16_t count;
        while ((count=BSP_UART_Read(BSP_UART_ELRS,bytes,sizeof(bytes)))!=0) {
            for (unsigned i=0;i<count;i++) {
                if (!DrvJy60_Push(&imu,bytes[i],HAL_GetTick())) continue;
                const Jy60Vector_t *v=imu.last_type==0x51 ? &imu.acceleration : imu.last_type==0x52 ? &imu.gyro : &imu.angle;
                imu_seq++;
#if defined(HC13_HOST_LINK)
                if (imu.last_type==0x51) capture_acc(v->timestamp_ms,v->milli,phase==0 && !host.following);
#endif
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
#if defined(JY61P_CAPTURE)
        imu_poll_recovery(HAL_GetTick());
#endif
#if defined(HC13_HOST_LINK)
        (void)BSP_UART_HC13_EnsureRxArmed();
        uint32_t overflows=BSP_UART_HC13_GetRxOverflows();
        if (overflows!=host_rx_overflows) {
            host_rx_overflows=overflows;
            while (BSP_UART_HC13_ReadRaw(bytes,sizeof(bytes))) { }
            ModHc13Host_InputLost(&host,HAL_GetTick());
        }
        while ((count=BSP_UART_HC13_ReadRaw(bytes,sizeof(bytes)))!=0)
            ModHc13Host_Receive(&host,bytes,count,HAL_GetTick());
        monitor(HAL_GetTick()); /* Process STOP/MODE before fresh-frame closure decisions. */
        /* Consume old console input without executing it in unified-host mode. */
        uint8_t ignored_keys[32];(void)SEGGER_RTT_Read(0,ignored_keys,sizeof(ignored_keys));
#else
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
#endif
        now=HAL_GetTick();
        if (phase && (!hx_healthy(now) || (!manual_move_active && !imu_live(now)))) finish("feedback_timeout_or_fault",now,true);
        else if (phase==1 && now-start_ms>10000U) finish("motion_timeout",now,true);
        else if (phase==2 && now-arrived_ms>=
#if defined(HC13_HOST_LINK)
                 (manual_move_active?0U:post_ms)
#else
                 post_ms
#endif
                 ) finish("complete",now,false);
#if defined(JY61P_CAPTURE)
        if (now-imu_rate_start_ms>=1000U) {
            uint32_t elapsed=now-imu_rate_start_ms;
            imu_rate_start_ms=now;
            uint32_t acc_hz=(imu.acceleration.count-rate_acc)*1000U/elapsed;
#if defined(HC13_HOST_LINK)
            host_acc_hz=(uint16_t)acc_hz;
#endif
            uint32_t gyro_hz=(imu.gyro.count-rate_gyro)*1000U/elapsed;
            uint32_t angle_hz=(imu.angle.count-rate_angle)*1000U/elapsed;
            if (imu_recovery_stage==IMU_REC_IDLE)
                imu_rate_ok=acc_hz>=180U && acc_hz<=220U && gyro_hz>=180U && gyro_hz<=220U && angle_hz>=180U && angle_hz<=220U;
            rate_acc=imu.acceleration.count;rate_gyro=imu.gyro.count;rate_angle=imu.angle.count;
            if (imu_recovering && imu_rate_ok && !imu_stream_stale(now)) {
                imu_recovering=false;
                emit("[CAP] IMU_RESUMED t_ms=%lu acc_hz=%lu\r\n",(unsigned long)now,(unsigned long)acc_hz);
            }
            emit("[CAP] IMU_RATE acc_hz=%lu gyro_hz=%lu angle_hz=%lu rate_ok=%u period_ms=%lu discarded=%lu\r\n",
                (unsigned long)acc_hz,(unsigned long)gyro_hz,(unsigned long)angle_hz,imu_rate_ok,
                (unsigned long)imu.angle_period_ms,(unsigned long)imu.discarded_bytes);
        }
#endif
        if (now-heartbeat>=1000U) {
            heartbeat=now;
            emit("[CAP] READY t_ms=%lu mode=%s record=%u ready=%u recovery_ready=%u phase=%u hx8=%u imu=%u angle_mdeg=%ld voltage=%d hx_tx=%lu hx_rx=%lu checksum_err=%lu drops_total=%lu\r\n",
                (unsigned long)now,manual_mode ? "MANUAL_HX8" : "CAPTURE",!manual_mode,
                !phase&&ready&&hx_healthy(now)&&(manual_mode||imu_live(now)),!phase&&recovery_ready&&hx_feedback_healthy(now),phase,
                hx_healthy(now),imu_live(now),have_fb ? (long)(fb.angle_deg*1000) : 0L,
                have_fb ? fb.voltage_mv : 0,(unsigned long)BSP_UART_Fsus_GetTxBytes(),
                (unsigned long)BSP_UART_Fsus_GetRxBytes(),(unsigned long)imu.checksum_errors,(unsigned long)drops);
        }
#if defined(HC13_HOST_LINK)
        ModHc13Host_Poll(&host,HAL_GetTick());
#endif
    }
#endif
}
#endif
