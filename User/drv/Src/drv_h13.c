/**
 * @file drv_h13.c
 * @brief L2 Driver: HC-13 Bluetooth UART module.
 *
 * KEY pin = HIGH keeps HC-13 in transparent (passthrough) mode permanently.
 * UART RX is interrupt-driven; bytes land in a ring buffer. No TX logic
 * (MCU only receives in this configuration).
 */

#include "drv_h13.h"
#include "stm32f1xx_hal.h"

/* KEY pin: PA4, active-low (low = AT mode, high = transparent). */
#define H13_KEY_PORT            GPIOA
#define H13_KEY_PIN             GPIO_PIN_4

/* =============================================================================
 * RX ring buffer
 * ============================================================================= */

static uint8_t  s_rx_buf[H13_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;   /* ISR writes */
static uint16_t s_rx_tail = 0U;            /* task reads */
static uint8_t  s_rx_byte;                 /* 1-byte IT target */

/* Check if USART3 is the H13 UART. The huart3 extern is in Core/Inc/usart.h. */
extern UART_HandleTypeDef huart3;

/* =============================================================================
 * Public API
 * ============================================================================= */

void DrvH13_Init(void)
{
    /* 1. Set KEY HIGH → permanent transparent mode. */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Pin   = H13_KEY_PIN,
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(H13_KEY_PORT, &gpio);
    HAL_GPIO_WritePin(H13_KEY_PORT, H13_KEY_PIN, GPIO_PIN_SET);

    /* 2. Flush the ring buffer. */
    s_rx_head = 0U;
    s_rx_tail = 0U;

    /* 3. Start single-byte interrupt receive on USART3. */
    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

uint16_t DrvH13_Recv(uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0U;
    }
    uint16_t count = 0U;
    while ((count < len) && (s_rx_tail != s_rx_head)) {
        data[count++] = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1U) % H13_RX_BUF_SIZE;
    }
    return count;
}

void DrvH13_Flush(void)
{
    s_rx_tail = s_rx_head;
}

/* =============================================================================
 * ISR callback — called from stm32f1xx_it.c / HAL_UART_RxCpltCallback
 *
 * The bsp_uart.c HAL_UART_RxCpltCallback must forward USART3 RX to this
 * function. This driver registers its ring buffer by being compiled in;
 * the actual forwarding is wired in bsp_uart.c.
 * ============================================================================= */

void DrvH13_RxIsrByte(uint8_t byte)
{
    uint16_t next = (s_rx_head + 1U) % H13_RX_BUF_SIZE;
    if (next != s_rx_tail) {   /* drop if full */
        s_rx_buf[s_rx_head] = byte;
        s_rx_head = next;
    }
}
