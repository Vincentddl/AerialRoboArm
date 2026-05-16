/**
 * @file drv_h13.h
 * @brief L2 Driver: HC-13 Bluetooth UART module (transparent mode, RX-only).
 *
 * HC-13 in transparent mode (KEY=HIGH) is a wireless serial cable.
 * MCU only receives — PC-side vision / commands arrive through this pipe.
 */

#ifndef DRV_H13_H
#define DRV_H13_H

#include "ara_def.h"

/* ============================================================================
 * Configuration
 * ========================================================================== */

#define H13_UART_BAUD            (9600U)
#define H13_RX_BUF_SIZE          (256U)

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief Initialise HC-13: set KEY pin HIGH (transparent mode) and
 *        arm USART interrupt RX.
 * @note  Call after BSP_UART_Init(). No RTOS dependency.
 */
void DrvH13_Init(void);

/**
 * @brief Read available bytes from the H13 RX ring buffer.
 * @param data  Destination buffer.
 * @param len   Maximum bytes to read.
 * @return Number of bytes actually copied.
 */
uint16_t DrvH13_Recv(uint8_t *data, uint16_t len);

/**
 * @brief Flush the H13 RX ring buffer.
 */
void DrvH13_Flush(void);

#endif /* DRV_H13_H */
