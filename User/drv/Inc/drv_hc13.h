/**
 * @file drv_hc13.h
 * @brief L2 Driver: HC-13 transparent UART link and vision packet parser.
 *
 * Reference demo:
 *   HC-13 vendor demo package
 *
 * The demo uses:
 *   - USART1 PA9/PA10, 9600 8N1
 *   - PB0 KEY pin, HIGH = transparent mode, LOW = AT mode
 *   - single-byte interrupt RX
 *
 * In AerialRoboArm the module is connected through USART3. BSP code feeds
 * received bytes into DrvHC13_PushRxByte() and can send AT commands built here.
 */

#ifndef DRV_HC13_H
#define DRV_HC13_H

#include "ara_def.h"

/* ============================================================================
 * Configuration
 * ========================================================================== */

#define HC13_UART_BAUD_DEFAULT           (9600U)
#define HC13_RX_BUF_SIZE                 (256U)
#define HC13_LINE_BUF_SIZE               (64U)
#define HC13_VISION_TTL_MS               (300U)

/* HC13 AT mode air-rate command. User requirement: S7 = highest rate. */
typedef enum {
    HC13_AIR_RATE_S1 = 1,
    HC13_AIR_RATE_S2 = 2,
    HC13_AIR_RATE_S3 = 3,
    HC13_AIR_RATE_S4 = 4,
    HC13_AIR_RATE_S5 = 5,
    HC13_AIR_RATE_S6 = 6,
    HC13_AIR_RATE_S7 = 7,
} HC13AirRate_t;

typedef struct {
    bool     target_present;
    int16_t  target_angle_deg;
    uint16_t target_speed;
    uint8_t  confidence;
    uint32_t timestamp_ms;
} HC13VisionSample_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief Reset all HC13 parser and RX state.
 * @note  Does not touch GPIO or UART; USART3 setup lives in BSP/HAL code.
 */
void DrvHC13_Init(void);

/**
 * @brief Build the AT command used to select air rate S1..S7.
 * @param rate  Desired HC13 air-rate level. S7 is the project default.
 * @param out   Destination buffer.
 * @param len   Destination capacity.
 * @return Bytes written, or 0 on parameter error.
 */
uint16_t DrvHC13_BuildSetAirRateCmd(HC13AirRate_t rate, uint8_t *out, uint16_t len);

/**
 * @brief Convenience wrapper for the project requirement: configure S7.
 */
uint16_t DrvHC13_BuildSetS7Cmd(uint8_t *out, uint16_t len);

/**
 * @brief Push one received transparent-mode byte into the software RX FIFO.
 * @note  Future UART ISR / DMA drain code should call this.
 */
void DrvHC13_PushRxByte(uint8_t byte);

/**
 * @brief Read raw bytes from the HC13 RX FIFO.
 */
uint16_t DrvHC13_Recv(uint8_t *data, uint16_t len);

/**
 * @brief Flush raw RX bytes and partial line parser state.
 */
void DrvHC13_Flush(void);

/**
 * @brief Drain RX bytes, parse PC vision text frames, and return freshest sample.
 *
 * Supported PC-side transparent text frames:
 *   V,<angle_deg>,<speed>,<confidence>\n
 *   VISION,<angle_deg>,<speed>,<confidence>\n
 *
 * Example:
 *   V,35,750,92\n
 *
 * angle_deg is an integer absolute HX8 g command produced by the empirical
 * camera/servo calibration; it is not the camera optical offset. Angle is
 * clamped to the mechanism range [-100,+100]. Speed is requested deg/s, where
 * zero selects the safe AUTO default; the execution layer applies an additional
 * safety cap. Confidence is clamped to [0,100]. A sample remains valid for
 * HC13_VISION_TTL_MS after its timestamp.
 */
bool DrvHC13_PollVision(uint32_t now_ms, HC13VisionSample_t *out);

/**
 * @brief Diagnostics: cumulative bytes accepted into the RX FIFO.
 */
uint32_t DrvHC13_GetRxBytes(void);

/**
 * @brief Diagnostics: cumulative valid vision frames parsed.
 */
uint32_t DrvHC13_GetVisionFrames(void);

#endif /* DRV_HC13_H */
