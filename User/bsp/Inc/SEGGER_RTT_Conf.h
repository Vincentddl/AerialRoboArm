/*********************************************************************
*       RTT configuration for AerialRoboArm (STM32F103 + FreeRTOS)
*********************************************************************/
#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#include <stdint.h>
#include <cmsis_gcc.h>

/* ---- Buffer sizes (bytes) ---- */
#define BUFFER_SIZE_UP                  (1024)  /* MCU -> PC  debug log */
#define BUFFER_SIZE_DOWN                (64)    /* PC  -> MCU console commands */

/* ---- Channel counts ---- */
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS   (2)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (2)

/* ---- Default write mode ---- */
#define SEGGER_RTT_MODE_DEFAULT         SEGGER_RTT_MODE_NO_BLOCK_SKIP

/* ---- FreeRTOS mutual exclusion (Cortex-M3 = no cache) ---- */
#define SEGGER_RTT_LOCK()               do { __disable_irq(); } while(0)
#define SEGGER_RTT_UNLOCK()             do { __enable_irq();  } while(0)

/* ---- Place control block in SRAM (not flashed) ---- */
#define SEGGER_RTT_IN_RAM               (1)

#endif
