#ifndef MOD_HC13_HOST_H
#define MOD_HC13_HOST_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HOST_LINE_MAX 160U
#define HOST_TX_SLOTS 6U
#define HOST_CACHE_SLOTS 8U
#define HOST_REPLY_MAX 80U
typedef enum { HOST_MANUAL, HOST_VISION_DEMO, HOST_VISION_PICK } HostMode_t;
typedef enum { HOST_MOVE, HOST_HOME, HOST_STOP, HOST_MODE, HOST_SHAPE, HOST_GRIP,
    HOST_SERVO, HOST_RECORD, HOST_GET_INFO, HOST_GET_CAL, HOST_GET_STATE, HOST_PING, HOST_VISION,
    HOST_PREP, HOST_PICK, HOST_TRIGGER } HostVerb_t;
typedef enum { HOST_OK, HOST_BAD_FRAME, HOST_BAD_NUMBER, HOST_OUT_OF_RANGE, HOST_NOT_READY,
    HOST_BUSY, HOST_MODE_DENIED, HOST_NOT_CALIBRATED, HOST_HX8_OFFLINE, HOST_IMU_OFFLINE,
    HOST_IMU_RATE_LOW, HOST_FAULT, HOST_MOTION_TIMEOUT, HOST_FEEDBACK_STALE, HOST_VISION_STALE,
    HOST_LINK_LOST, HOST_STOPPED, HOST_REQUEST_CONFLICT, HOST_STALE_REQUEST, HOST_UNSUPPORTED,
    HOST_TX_FAILED } HostError_t;
typedef struct {
    HostVerb_t verb;
    uint32_t id;
    int32_t value;
    uint16_t speed;
    uint8_t confidence;
    uint16_t lead_mdeg;
} HostCommand_t;
typedef struct {
    bool hx_online, hx_healthy, ready, recovery_ready, imu_online, imu_ready, record_on, target_valid;
    uint8_t phase;
    int32_t pos_mdeg, target_mdeg;
    uint16_t voltage_mv, imu_hz;
    uint32_t acc_count, acc_ms, motion_start_ms, feedback_seq;
    int32_t acc_mg[3];
} HostSnapshot_t;
typedef struct {
    int32_t vertical_mdeg, folded_mdeg, grip_open_mdeg, grip_close_mdeg;
    bool fold_calibrated, grip_calibrated, open_wait_calibrated, close_wait_calibrated;
    int8_t extension_sign;
    uint32_t max_extension_mdeg;
    uint16_t max_speed, ramp_ms, post_ms, open_wait_ms, close_wait_ms, default_lead_mdeg;
    uint8_t min_confidence;
} HostConfig_t;
typedef struct {
    void (*snapshot)(HostSnapshot_t *out, uint32_t now_ms);
    HostError_t (*move)(int32_t target_mdeg, uint16_t speed, HostVerb_t kind, uint32_t now_ms);
    HostError_t (*stop)(uint32_t now_ms);
    HostError_t (*servo)(int32_t angle_mdeg);
    HostError_t (*record)(bool enabled);
    /* False means transport busy; the module keeps critical queued replies. */
    bool (*send)(const char *line, size_t length);
} HostHooks_t;
typedef struct {
    bool valid;
    HostCommand_t command;
    char reply[HOST_REPLY_MAX];
} HostCacheEntry_t;
typedef struct {
    char bytes[HOST_LINE_MAX+2U];
    uint16_t length;
} HostTxFrame_t;
typedef enum { HOST_FLOW_IDLE, HOST_FLOW_POSITION, HOST_FLOW_PREP_MOVE, HOST_FLOW_PREP_OPEN_WAIT,
    HOST_FLOW_FOLD_CLOSE_WAIT, HOST_FLOW_FOLD_MOVE, HOST_FLOW_PICK } HostFlow_t;
typedef enum { HOST_EVENT_PREPARING, HOST_EVENT_OPEN, HOST_EVENT_READY, HOST_EVENT_START,
    HOST_EVENT_CLOSE, HOST_EVENT_ARRIVED, HOST_EVENT_END, HOST_EVENT_ABORT } HostEventKind_t;
typedef struct {
    HostEventKind_t kind;
    HostError_t reason;
    uint32_t t_ms;
    int32_t pos_mdeg;
    bool pos_valid;
} HostEvent_t;
typedef struct {
    HostConfig_t config;
    HostHooks_t hooks;
    HostMode_t mode;
    HostError_t fault, finish_reason;
    uint32_t boot_id, highest_id, last_valid_ms, last_state_ms, last_acc_send_ms,
        last_acc_count, acc_seq, last_rx_ms, last_vision_ms;
    bool owner, pwm_valid, finish_pending, active, candidate_valid;
    HostCommand_t candidate;
    uint32_t candidate_ms;
    HostFlow_t flow;
    bool prepared, closed_once, engine_done, arrived, finish_event_queued, event_overflow;
    uint32_t stage_ms, action_ms, close_ms, last_feedback_seq, finish_ms;
    int32_t pick_start_mdeg;
    HostEvent_t events[8];
    uint8_t event_head, event_count;
    int32_t grip_mdeg, finish_pos_mdeg;
    uint8_t grip_action;
    HostCommand_t active_command;
    HostCacheEntry_t cache[HOST_CACHE_SLOTS];
    uint8_t cache_next;
    HostTxFrame_t tx[HOST_TX_SLOTS];
    uint8_t tx_head, tx_count;
    char input[HOST_LINE_MAX+1U];
    uint16_t input_used;
    bool discard_input;
    uint32_t invalid_frames, queue_pressure;
} Hc13Host_t;

const char *ModHc13Host_ErrorName(HostError_t error);
const char *ModHc13Host_ModeName(HostMode_t mode);
uint32_t ModHc13Host_FeedbackPeriod(const Hc13Host_t *ctx);
HostError_t ModHc13Host_Parse(const char *line, size_t length, HostCommand_t *out);
void ModHc13Host_Init(Hc13Host_t *ctx, const HostConfig_t *config, const HostHooks_t *hooks,
                     uint32_t boot_id, uint32_t now_ms);
void ModHc13Host_Receive(Hc13Host_t *ctx, const uint8_t *bytes, size_t length, uint32_t now_ms);
void ModHc13Host_InputLost(Hc13Host_t *ctx, uint32_t now_ms);
void ModHc13Host_Poll(Hc13Host_t *ctx, uint32_t now_ms);
void ModHc13Host_MotionArrived(Hc13Host_t *ctx, uint32_t now_ms, int32_t pos_mdeg);
void ModHc13Host_MotionFinished(Hc13Host_t *ctx, HostError_t reason, uint32_t now_ms, int32_t pos_mdeg);
#endif
