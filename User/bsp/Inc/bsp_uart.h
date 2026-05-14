/**
 * @file bsp_uart.h
 * @brief UART Driver with DMA RingBuffer + Half-Duplex transaction API.
 *
 * Existing APIs (Printf / Send_DMA / Read / SetRxCpltCallback) continue to
 * work for USART3 (debug) and USART1 (ELRS).
 *
 * The HD_* extension is designed for ST3215 serial bus servo on USART2 with
 * hardware half-duplex (USART_CR3_HDSEL = 1). The upper layer (task_motion)
 * composes a protocol frame via drv_st3215, pushes it through HalfDuplex_Transact,
 * then waits for the response with HD_WaitRx. Time budget at 1 Mbps is 2 ms
 * per round-trip.
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include "ara_def.h"
#include <stdarg.h>

/* --- Device Definition --- */
typedef enum {
    BSP_UART_DEBUG = 0,
    BSP_UART_ELRS,       /**< ELRS receiver input (USART1). */
    BSP_UART_ST3215,     /**< ST3215 serial bus servo, half-duplex (USART2). */
    BSP_UART_NUM
} BspUart_Dev_t;

/* =============================================================================
 * Standard API (unchanged from demo_v6)
 * ============================================================================= */

void BSP_UART_Init(void);

/* Debug / Logging */
void BSP_UART_Printf(const char *format, ...);

/* DMA Send (Non-blocking) */
AraStatus_t BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);

/* RingBuffer Read */
uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);

/* Async Receive Callback */
void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, AraCallback_t cb);

/* =============================================================================
 * Half-Duplex API (new in demo_v7)
 *
 * Designed for ST3215 bus but not hardcoded to it. The caller provides the
 * protocol bytes. This layer only manages direction switching and DMA.
 *
 * STAGE 1 (electric motor absent) implementation: stub that records the
 * attempted transact and always reports RX timeout. Allows the full control
 * stack to compile and run on the bench without electrical bus activity.
 *
 * STAGE 2 (electric motor present) implementation: real half-duplex flow:
 *   1. Start DMA TX of tx_buf, len = tx_len
 *   2. Wait TC (transmit complete)
 *   3. Start DMA RX of rx_buf, len = expect_rx_len
 *   4. RX-complete ISR notifies the waiting task via xTaskNotifyGive
 * ============================================================================= */

/**
 * @brief Start a half-duplex transaction: send tx then arm RX.
 * @param dev              Must be BSP_UART_ST3215 in demo_v7.
 * @param tx_buf           Caller-owned buffer with the full TX frame.
 * @param tx_len           Number of bytes to transmit.
 * @param rx_buf           Caller-owned RX destination buffer. Must remain
 *                         valid until HD_WaitRx returns.
 * @param expect_rx_len    Number of bytes the caller expects to receive.
 * @return ARA_OK         on successful kickoff (does NOT mean TX completed).
 * @return ARA_ERR_PARAM  for NULL / invalid dev / zero length.
 * @return ARA_BUSY       when a previous transact is still in progress.
 * @note   Non-blocking. Returns before TX has finished sending on the wire.
 *         Pair with HD_WaitRx.
 */
AraStatus_t BSP_UART_HalfDuplex_Transact(BspUart_Dev_t dev,
                                         const uint8_t *tx_buf,
                                         uint16_t       tx_len,
                                         uint8_t       *rx_buf,
                                         uint16_t       expect_rx_len);

/**
 * @brief Wait until the RX half of the previous transact completes.
 * @param dev          UART device used in the preceding Transact call.
 * @param timeout_ms   Maximum wait in milliseconds. 2 ms at 1 Mbps is typical.
 * @param out_rx_len   Optional. Number of bytes actually received.
 * @return ARA_OK            full expected RX received.
 * @return ARA_TIMEOUT       deadline exceeded. Caller MUST invoke HD_AbortRx
 *                           before the next transact.
 * @return ARA_ERR_PARAM     invalid dev or called without matching Transact.
 * @return ARA_ERR_IO        DMA / HAL error reported by ISR.
 */
AraStatus_t BSP_UART_HD_WaitRx(BspUart_Dev_t dev,
                               uint32_t      timeout_ms,
                               uint16_t     *out_rx_len);

/**
 * @brief Abort the in-flight RX DMA. Required after HD_WaitRx returns TIMEOUT.
 */
void BSP_UART_HD_AbortRx(BspUart_Dev_t dev);

#endif /* BSP_UART_H */
