#ifndef ARDUINO_COMPAT_H
#define ARDUINO_COMPAT_H

#include <stddef.h>
#include <stdint.h>

typedef uint8_t byte;

#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1
#define PIN_VL53_XSHUT 1

#ifdef __cplusplus
extern "C" {
#endif

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
void delay(uint32_t milliseconds);
void delayMicroseconds(uint32_t microseconds);
uint32_t millis(void);

#ifdef __cplusplus
}
#endif

#endif
