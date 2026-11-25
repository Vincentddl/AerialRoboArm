#include <stdint.h>
#include "stm32f103xb.h"

/* Fixed SRAM result: magic, phase, toggle count, HSE ready, RCC_CR. */
volatile uint32_t led_result[5]
    __attribute__((section(".hse_result"), used));

static void delay(volatile uint32_t n)
{
    while (n-- != 0U) { __NOP(); }
}

int main(void)
{
    /* Reset-default HSI; no HAL, RTOS, peripherals or watchdog started. */
    led_result[0] = 0x4c454454U;
    led_result[1] = 1U;
    led_result[2] = 0U;
    led_result[3] = 0U;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->BSRR = GPIO_BSRR_BS13;
    GPIOC->CRH = (GPIOC->CRH & ~(0xfU << 20)) | (2U << 20);

    /* HSE check is bounded and never used as the CPU clock. */
    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t i = 0; i < 500000U; ++i) {
        if (RCC->CR & RCC_CR_HSERDY) { led_result[3] = 1U; break; }
    }
    led_result[4] = RCC->CR;

    /* First eight transitions with IRQs enabled; then all maskable IRQs off. */
    for (;;) {
        if (led_result[2] == 8U) {
            __disable_irq();
            led_result[1] = 2U;
        }
        GPIOC->ODR ^= GPIO_ODR_ODR13;
        ++led_result[2];
        delay(400000U);
    }
}
