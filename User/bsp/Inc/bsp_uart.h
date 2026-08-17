/**
 * @file bsp_uart.h
 * @brief UART Driver with ELRS DMA RX, FSUS blocking TX / interrupt RX, and RTT logging.
 *
 * Existing Read APIs remain for USART1 (ELRS). Debug printing now goes through
 * SEGGER RTT, not USART3.
 *
 * The HD_* extension below is legacy ST3215 support kept for reference only.
 * The current HX8-U26H-M path uses the FSUS helpers.
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include "ara_def.h"
#include <stdarg.h>

/* --- Device Definition --- */
typedef enum {
    BSP_UART_DEBUG = 0,
    BSP_UART_ELRS,       /**< ELRS receiver input (USART1). */
    BSP_UART_ST3215,     /**< Legacy enum name; current USART2 bus-servo path is FSUS/HX8. */
    BSP_UART_HC13,        /**< HC13 transparent UART input (USART3). */
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
 * FSUS transaction API (USART2, full-duplex, interrupt RX + blocking TX)
 *
 * For Fashion Star UART Servo protocol. Replaces the old ST3215 HD state
 * machine. The lower-level HD API is kept below for reference but no longer
 * called by task_motion.
 *
 * Pattern: Send frame (blocking) → poll RX ring buffer until expected bytes
 * arrive or timeout.
 * ============================================================================= */

/**
 * @brief Send bytes on USART2 via blocking HAL_UART_Transmit.
 * @param data  Data to send.
 * @param len   Byte count.
 */
void BSP_UART_Fsus_Send(const uint8_t *data, uint16_t len);

/**
 * @brief Read bytes from USART2 RX ring buffer, up to len bytes.
 *        Returns immediately with whatever is available (non-blocking).
 * @param data  Destination buffer.
 * @param len   Maximum bytes to read.
 * @return Number of bytes actually read.
 */
uint16_t BSP_UART_Fsus_Recv(uint8_t *data, uint16_t len);

/**
 * @brief Flush the USART2 RX ring buffer.
 */
void BSP_UART_Fsus_Flush(void);

/**
 * @brief Bring-up diagnostics: cumulative TX/RX byte counters on USART2.
 *        TX counter increments after a successful HAL_UART_Transmit; RX
 *        counter increments inside the RxCpltCallback for each byte that
 *        actually lands in the ring buffer.
 */
uint32_t BSP_UART_Fsus_GetTxBytes(void);
uint32_t BSP_UART_Fsus_GetRxBytes(void);

/**
 * @brief Bring-up diagnostics: cumulative bytes received on USART1 (ELRS)
 *        DMA ring buffer. Non-zero means the DMA channel is seeing data.
 */
uint32_t BSP_UART_Elrs_GetRxBytes(void);

/**
 * @brief Bring-up diagnostics: cumulative bytes received from HC13 on
 *        USART3 and accepted into the HC13 parser FIFO.
 */
uint32_t BSP_UART_HC13_GetRxBytes(void);

/**
 * @brief Read raw HC13 bytes mirrored from USART3 RX for bring-up echo tests.
 *        This does not consume bytes from the HC13 parser FIFO.
 */
uint16_t BSP_UART_HC13_ReadRaw(uint8_t *data, uint16_t len);

/**
 * @brief Send bytes out through USART3 to the MCU-side HC13 module.
 */
void BSP_UART_HC13_Send(const uint8_t *data, uint16_t len);

/**
 * @brief Bring-up diagnostics: number of circular RX DMA restarts after UART
 *        errors on DMA-backed ports.
 */
uint32_t BSP_UART_RxDma_GetRecoveries(void);

/* =============================================================================
 * Half-Duplex API (legacy, ST3215 — kept for reference, not called currently)
 * ============================================================================= */

AraStatus_t BSP_UART_HalfDuplex_Transact(BspUart_Dev_t dev,
                                         const uint8_t *tx_buf,
                                         uint16_t       tx_len,
                                         uint8_t       *rx_buf,
                                         uint16_t       expect_rx_len);

AraStatus_t BSP_UART_HD_WaitRx(BspUart_Dev_t dev,
                               uint32_t      timeout_ms,
                               uint16_t     *out_rx_len);

void BSP_UART_HD_AbortRx(BspUart_Dev_t dev);

#endif /* BSP_UART_H */
