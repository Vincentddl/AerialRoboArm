# LED smoke test — 2026-11-13

## Firmware

Source: `tools/hse_test/led_smoke_test.c`.
Built with ARM GCC, Cortex-M3, no standard library. Flash payload: 280 bytes.
Outputs: `led_smoke_test.elf`, `led_smoke_test.hex`.
Original application sources have not been modified.

CPU uses reset-default HSI. PC13 toggles with a software delay. No HAL,
RTOS, SysTick, peripheral interrupt, PWM or watchdog is initialized.
First eight output transitions run with maskable interrupts enabled;
thereafter PRIMASK disables maskable interrupts. This is a baseline test,
not a reproduction of the application's interrupt load.
HSE is enabled and polled with a bounded timeout; failure does not block blinking.
No MCO output is configured.

Result words at SRAM 0x20000000:

| Offset | Meaning |
| --- | --- |
| 0x00 | Magic 0x4c454454 |
| 0x04 | Phase: 1 = IRQ enabled, 2 = maskable IRQ disabled |
| 0x08 | PC13 transition counter |
| 0x0c | HSE ready: 1 = ready, 0 = timeout |
| 0x10 | RCC_CR snapshot |

To validate execution after programming, read these words twice at least
one second apart and confirm the transition counter increases. Read
GPIOC_ODR (0x4001100c) and observe the actual LED to check pin behavior.
The physical LED must be connected to PC13 for this test to be visible.

## Hardware attempts in this turn

- Normal SWD at 100 kHz: DPIDR 0x1ba01477 and Cortex-M3 detected;
  `external reset detected`, halt timeout.
- Connect under reset at 50 kHz: could not read DP IDR.
- Normal SWD at 50 kHz after explicit SRST deassert: CPU detected;
  `external reset detected`, halt timeout.

Flash backup command was placed after halt and was not reached.
No Flash erase or programming was performed. The LED firmware has not
run on the target. These results do not establish an interrupt fault,
an application initialization fault, or a damaged MCU.

Next physical checks: power-cycle target and probe, confirm stable 3.3 V,
common ground and correct SWD wiring. Measure NRST; it should normally
be near 3.3 V. Reset detection in the log alone does not prove the line
is continuously low. Retry backup before programming this test.

## Retry after user power-cycle

Normal 100 kHz connection still detects Cortex-M3 but cannot halt.
DHCSR read repeatedly: 0x02010001 (S_RESET_ST set, S_HALT clear).
RCC_CR, RCC_CFGR, RCC_CSR and Flash vector reads returned zero.
A 65536-byte Flash read completed but contained only zeros; this is
stored as `INVALID_all_zero_flash_read_20261113.bin`, NOT a usable backup.
Explicit SRST release and a direct DHCSR halt request did not change
the observed state. No Flash erase/program command was issued.
Suspect held/repeated reset or abnormal supply/access; measure physical
NRST and VDD before proceeding. The observations alone cannot identify
the failed component or confirm oscillator state.
