#ifndef DRV_JY60_H
#define DRV_JY60_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t milli[3]; /* acceleration: mg; gyro: milli deg/s; angle: milli deg */
    int16_t tail_raw; /* Bytes 8..9: meaning varies by packet/revision; not calibrated temperature. */
    uint32_t timestamp_ms;
    uint32_t count;
} Jy60Vector_t;

typedef struct {
    uint8_t frame[11];
    uint8_t used;
    uint8_t last_type;
    uint32_t rx_bytes;
    uint32_t good_frames;
    uint32_t checksum_errors;
    uint32_t discarded_bytes;
    uint32_t angle_period_ms;
    Jy60Vector_t acceleration;
    Jy60Vector_t gyro;
    Jy60Vector_t angle;
} Jy60Parser_t;

void DrvJy60_Init(Jy60Parser_t *parser);
bool DrvJy60_Push(Jy60Parser_t *parser, uint8_t byte, uint32_t now_ms);
#endif
