/*********************************************************************
*       SEGGER RTT * minimal defaults for STM32F103 (Cortex-M3).
*       Replaces the internal SEGGER file that ships with J-Link.
*********************************************************************/
#ifndef SEGGER_RTT_CONF_DEFAULTS_H
#define SEGGER_RTT_CONF_DEFAULTS_H

/* Pull in user overrides first */
#include "SEGGER_RTT_Conf.h"

/* All configurable values have #ifndef guards in SEGGER_RTT.c / .h,
 * so this file just needs to exist.  Customise buffer sizes and
 * FreeRTOS lock macros in SEGGER_RTT_Conf.h. */

#endif
