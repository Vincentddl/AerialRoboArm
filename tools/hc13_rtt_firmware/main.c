/* Isolated communication firmware: ONLY PB10/PB11 USART3 and debug RTT.
 * No USART1/2, timers, PWM, actuator drivers, RTOS, home or motion commands.
 * Request Lssssqqqqcccc\n -> reply Rssssqqqqcccc\n (each CRC covers 9 bytes).
 * This direct reply does NOT reproduce production's 20ms task/200ms ACK gate.
 */
#include "stm32f1xx.h"
#include "SEGGER_RTT.h"
#include <stdint.h>

volatile uint32_t hc13_rx_bytes, hc13_valid_probes, hc13_invalid_lines;
volatile uint32_t hc13_uart_errors, hc13_clock_error;
volatile uint32_t hc13_reply_service_cycles_max;

static int wait_bit(volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    for (uint32_t n=0;n<20000000U;n++) if ((*reg & mask)==expected) return 1;
    return 0;
}

static void clock_72mhz(void)
{
    RCC->CR |= RCC_CR_HSEON;
    if (!wait_bit(&RCC->CR,RCC_CR_HSERDY,RCC_CR_HSERDY)) {hc13_clock_error=1;return;}
    FLASH->ACR=FLASH_ACR_PRFTBE|FLASH_ACR_LATENCY_2;
    RCC->CFGR=RCC_CFGR_PPRE1_DIV2|RCC_CFGR_PLLSRC|RCC_CFGR_PLLMULL9;
    RCC->CR|=RCC_CR_PLLON;
    if (!wait_bit(&RCC->CR,RCC_CR_PLLRDY,RCC_CR_PLLRDY)) {hc13_clock_error=2;return;}
    RCC->CFGR|=RCC_CFGR_SW_PLL;
    if (!wait_bit(&RCC->CFGR,RCC_CFGR_SWS,RCC_CFGR_SWS_PLL)) {hc13_clock_error=3;return;}
    SystemCoreClock=72000000U;
}

static uint16_t crc16(const uint8_t *data, unsigned count)
{
    uint16_t crc=0xffffU;
    for (unsigned i=0;i<count;i++) {
        crc^=(uint16_t)data[i]<<8;
        for (unsigned bit=0;bit<8;bit++) crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
    }
    return crc;
}

static int hex(uint8_t c)
{
    if(c>='0' && c<='9')return c-'0';
    if(c>='A' && c<='F')return c-'A'+10;
    return -1;
}

static int valid(const uint8_t *line, unsigned length)
{
    if(length!=13U || line[0]!='L')return 0;
    uint16_t provided=0;
    for(unsigned i=1;i<13;i++) {
        int digit=hex(line[i]);if(digit<0)return 0;
        if(i>=9)provided=(uint16_t)((provided<<4)|(unsigned)digit);
    }
    return provided==crc16(line,9);
}

static void reply(uint8_t *line)
{
    static const char digits[]="0123456789ABCDEF";
    line[0]='R';uint16_t crc=crc16(line,9);
    for(unsigned i=0;i<4;i++)line[9+i]=(uint8_t)digits[(crc>>(12U-4U*i))&15U];
    line[13]='\n';
    for(unsigned i=0;i<14;i++) {
        while(!(USART3->SR&USART_SR_TXE)) {}
        USART3->DR=line[i];
    }
}

int main(void)
{
    clock_72mhz();
    SEGGER_RTT_Init();
    if(hc13_clock_error) {
        SEGGER_RTT_WriteString(0,"[HC13_RTT] CLOCK_ERROR; UART disabled\r\n");
        for(;;) {}
    }
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    RCC->APB2ENR|=RCC_APB2ENR_IOPBEN|RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR|=RCC_APB1ENR_USART3EN;
    GPIOB->CRH=(GPIOB->CRH&~(0xffU<<8))|(0x4bU<<8); /* PB10 AF PP; PB11 input. */
    USART3->BRR=(36000000U+230400U/2U)/230400U;
    USART3->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;
    SEGGER_RTT_WriteString(0,"[HC13_RTT] READY UART3=230400 PB10/PB11 PWM=OFF HX8=OFF immediate_reply=ON\r\n");
    uint8_t line[64];unsigned length=0;uint32_t heartbeat=DWT->CYCCNT;
    for(;;) {
        uint32_t sr=USART3->SR;
        if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) {
            uint8_t ch=(uint8_t)USART3->DR;hc13_rx_bytes++;
            if(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) {
                hc13_uart_errors++;length=0;continue;
            }
            if(ch=='\n') {
                uint32_t began=DWT->CYCCNT;
                if(valid(line,length)) {
                    reply(line);hc13_valid_probes++;
                    uint32_t cycles=DWT->CYCCNT-began;
                    if(cycles>hc13_reply_service_cycles_max)hc13_reply_service_cycles_max=cycles;
                } else hc13_invalid_lines++;
                length=0;
            } else if(length<sizeof(line)-1U) line[length++]=ch;
            else {length=0;hc13_invalid_lines++;}
        }
        if((uint32_t)(DWT->CYCCNT-heartbeat)>72000000U) {
            heartbeat=DWT->CYCCNT;
            SEGGER_RTT_WriteString(0,"[HC13_RTT] ALIVE\r\n");
        }
    }
}
