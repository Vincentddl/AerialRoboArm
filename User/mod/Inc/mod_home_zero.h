#ifndef MOD_HOME_ZERO_H
#define MOD_HOME_ZERO_H
#include "drv_fsus.h"

#define HOME_ZERO_TIMEOUT_MS (5000U)
#define HOME_ZERO_TOLERANCE_DEG (0.5f)
typedef enum { HOME_ZERO_WAITING, HOME_ZERO_MOVING, HOME_ZERO_DONE, HOME_ZERO_FAILED } HomeZeroStage_t;
typedef struct {
    HomeZeroStage_t stage;
    uint32_t started_ms;
    uint32_t sample_ms;
    bool have_sample;
    float previous_deg;
    uint8_t arrived_samples;
} HomeZero_t;
void ModHomeZero_Reset(HomeZero_t *home);
/** Return torque request. Only new healthy samples can complete homing. */
bool ModHomeZero_Update(HomeZero_t *home, const FsusFeedback_t *feedback,
                        bool valid, bool fresh, uint32_t age_ms,
                        bool permitted, uint32_t now_ms);
#endif
