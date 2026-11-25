#ifndef MOD_STARTUP_ANGLE_H
#define MOD_STARTUP_ANGLE_H
#include "ara_def.h"
#include "drv_fsus.h"

#define STARTUP_ANGLE_SETTLE_MS (300U)
#define STARTUP_ANGLE_SPREAD_DEG (0.5f)
typedef struct {
    bool active;
    uint32_t since_ms;
    uint32_t last_sample_ms;
    uint16_t samples;
    float min_deg;
    float max_deg;
} StartupAngleGate_t;

void ModStartupAngle_Reset(StartupAngleGate_t *gate);
/** Fresh, healthy, in-range readings must agree for 300 ms before readiness. */
bool ModStartupAngle_Update(StartupAngleGate_t *gate, const FsusFeedback_t *feedback,
                            bool valid, bool fresh, uint32_t now_ms);
/* Explicit encoder-zero recovery only. Does not expand normal control limits. */
bool ModStartupAngle_UpdateRecovery(StartupAngleGate_t *gate,const FsusFeedback_t *feedback,
                                    bool valid,bool fresh,uint32_t now_ms);
bool ModStartupAngle_RecoveryPathOk(float angle,float start);
#endif
