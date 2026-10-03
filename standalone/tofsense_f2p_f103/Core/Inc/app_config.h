#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* The TOFSense-F IIC 7-bit address is 0x08 + the configured module ID. */
#define TOFSENSE_I2C_ID              0U
#define TOFSENSE_I2C_CLOCK_HZ        400000U
#define TOFSENSE_POLL_PERIOD_MS      10U
#define TOFSENSE_I2C_TIMEOUT_MS      20U

/* Send one measurement line per N successful IIC reads to USB-TTL. */
#define TOFSENSE_DEBUG_DECIMATION    10U

#define DEBUG_UART_BAUDRATE          115200U
#define DEBUG_UART_TIMEOUT_MS        100U

#endif /* APP_CONFIG_H */
