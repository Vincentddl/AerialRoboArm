#include <stdint.h>

#include "stm32f103xb.h"

#define HSE_TEST_MAGIC          0x48534554UL
#define HSE_TEST_STARTING       0x00000000UL
#define HSE_TEST_READY          0x00000001UL
#define HSE_TEST_FAILED         0x00000002UL

#define HSE_STARTUP_TIMEOUT     500000UL
#define LED_FAST_DELAY          200000UL
#define LED_SLOW_DELAY          1200000UL

typedef struct {
    uint32_t magic;
    uint32_t status;
    uint32_t rcc_cr;
    uint32_t rcc_cfgr;
    uint32_t wait_loops;
} HseTestResult;

/*
 * The linker fixes this structure at 0x20000000 so OpenOCD can read the
 * result without symbols:
 *   mdw 0x20000000 5
 */
volatile HseTestResult g_hse_test_result
    __attribute__((section(".hse_result"), used));

static void delay(volatile uint32_t count)
{
    while (count-- != 0U) {
        __NOP();
    }
}

static void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    (void)RCC->APB2ENR;

    /* PC13: 2 MHz push-pull output. */
    GPIOC->CRH = (GPIOC->CRH & ~(0xFUL << 20U)) | (0x2UL << 20U);
    GPIOC->BSRR = GPIO_BSRR_BS13;
}

static void led_toggle(void)
{
    GPIOC->ODR ^= GPIO_ODR_ODR13;
}

static void mco_output_hse_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    (void)RCC->APB2ENR;

    /* PA8: 50 MHz alternate-function push-pull output. */
    GPIOA->CRH = (GPIOA->CRH & ~0xFUL) | 0xBUL;

    /* Output the undivided HSE clock on PA8. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO) | RCC_CFGR_MCO_HSE;
}

int main(void)
{
    uint32_t loops = 0U;

    led_init();

    g_hse_test_result.magic = HSE_TEST_MAGIC;
    g_hse_test_result.status = HSE_TEST_STARTING;
    g_hse_test_result.rcc_cr = RCC->CR;
    g_hse_test_result.rcc_cfgr = RCC->CFGR;
    g_hse_test_result.wait_loops = 0U;

    /*
     * The MCU remains on its reset-default HSI clock. HSE failure therefore
     * cannot stop this test program from reporting the result.
     */
    RCC->CR |= RCC_CR_HSEON;

    while (((RCC->CR & RCC_CR_HSERDY) == 0U) &&
           (loops < HSE_STARTUP_TIMEOUT)) {
        loops++;
        if ((loops & 0x3FFFUL) == 0U) {
            g_hse_test_result.wait_loops = loops;
        }
    }

    g_hse_test_result.wait_loops = loops;
    g_hse_test_result.rcc_cr = RCC->CR;

    if ((RCC->CR & RCC_CR_HSERDY) != 0U) {
        mco_output_hse_init();
        g_hse_test_result.rcc_cfgr = RCC->CFGR;
        g_hse_test_result.status = HSE_TEST_READY;

        /* Success: PC13 blinks slowly; PA8 outputs the 8 MHz HSE clock. */
        for (;;) {
            led_toggle();
            delay(LED_SLOW_DELAY);
        }
    }

    g_hse_test_result.rcc_cfgr = RCC->CFGR;
    g_hse_test_result.status = HSE_TEST_FAILED;

    /* Failure: PC13 blinks quickly; PA8 remains inactive. */
    for (;;) {
        led_toggle();
        delay(LED_FAST_DELAY);
    }
}
