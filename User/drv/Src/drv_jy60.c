#include "drv_jy60.h"
#include <string.h>

void DrvJy60_Init(Jy60Parser_t *p) { if (p) memset(p, 0, sizeof(*p)); }

static int16_t read_i16(const uint8_t *data)
{
    uint16_t u = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)(u < 0x8000U ? (int32_t)u : (int32_t)u - 65536);
}

bool DrvJy60_Push(Jy60Parser_t *p, uint8_t byte, uint32_t now_ms)
{
    if (!p) return false;
    p->rx_bytes++;
    p->frame[p->used++] = byte;
    /* Slide one byte on failure, preserving a header inside a broken packet. */
    for (;;) {
        if (p->frame[0] != 0x55U ||
            (p->used >= 2U && (p->frame[1] < 0x51U || p->frame[1] > 0x53U))) {
            memmove(p->frame, p->frame + 1, --p->used);
            p->discarded_bytes++;
            if (!p->used) return false;
            continue;
        }
        if (p->used < 11U) return false;
        uint8_t sum = 0;
        for (unsigned i=0; i<10U; i++) sum = (uint8_t)(sum + p->frame[i]);
        if (sum != p->frame[10]) {
            p->checksum_errors++;
            memmove(p->frame, p->frame + 1, --p->used);
            p->discarded_bytes++;
            continue;
        }
        p->last_type = p->frame[1];
        Jy60Vector_t *v = p->last_type == 0x51U ? &p->acceleration
                           : p->last_type == 0x52U ? &p->gyro : &p->angle;
        int32_t scale = p->last_type == 0x51U ? 16000
                         : p->last_type == 0x52U ? 2000000 : 180000;
        for (unsigned i=0; i<3U; i++) {
            /* 64-bit intermediate avoids gyro-scale overflow. */
            v->milli[i] = (int32_t)((int64_t)read_i16(&p->frame[2U + i*2U]) * scale / 32768);
        }
        v->tail_raw = read_i16(&p->frame[8]);
        if (p->last_type == 0x53U && v->count) p->angle_period_ms = now_ms - v->timestamp_ms;
        v->timestamp_ms = now_ms;
        v->count++;
        p->good_frames++;
        p->used = 0U;
        return true;
    }
}
