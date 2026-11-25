#if defined(JY61P_DIAGNOSTIC)
#include "app_jy61p_diagnostic.h"
#include "bsp_uart.h"
#include "drv_jy60.h"
#include "SEGGER_RTT.h"
#include "iwdg.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static Jy60Parser_t parser;
static uint32_t baud=115200;
/* Connector must be unplugged. Compare TX with RX as an unloaded GPIO
 * control, and log configuration/output-latch/input separately. */
static void unloaded_pin_test(void)
{
    uint32_t original_crl=GPIOB->CRL,original_cr1=USART1->CR1;
    uint32_t original_odr=GPIOB->ODR;
    (void)HAL_UART_Abort(&huart1);
    USART1->CR1&=~(USART_CR1_TE|USART_CR1_RE);
    BSP_UART_Printf("[DIAG] UNLOADED_START rcc=0x%lX lock=0x%lX original_crl=0x%lX\r\n",
        (unsigned long)RCC->APB2ENR,(unsigned long)GPIOB->LCKR,(unsigned long)original_crl);
    for (unsigned pin=6;pin<=7;pin++) {
        uint32_t mask=1U<<pin,shift=pin*4U;
        GPIOB->BSRR=mask;
        GPIOB->CRL=(GPIOB->CRL&~(0xFU<<shift))|(0x3U<<shift);
        for (unsigned step=0;step<3;step++) {
            unsigned expected=step!=1;
            GPIOB->BSRR=expected ? mask : mask<<16;
            HAL_Delay(10);(void)HAL_IWDG_Refresh(&hiwdg);
            BSP_UART_Printf("[DIAG] UNLOADED PB%u expected=%u odr=%u idr=%u mode=0x%lX lock=0x%lX\r\n",
                pin,expected,(unsigned)((GPIOB->ODR>>pin)&1U),(unsigned)((GPIOB->IDR>>pin)&1U),
                (unsigned long)((GPIOB->CRL>>shift)&0xFU),(unsigned long)GPIOB->LCKR);
        }
        GPIOB->CRL=original_crl;
    }
    GPIOB->BSRR=(original_odr&(GPIO_PIN_6|GPIO_PIN_7)) |
        ((~original_odr&(GPIO_PIN_6|GPIO_PIN_7))<<16);
    USART1->CR1=original_cr1;
    (void)BSP_UART_ImuSetBaud(baud);
    BSP_UART_Printf("[DIAG] UNLOADED_END uart_restored=1\r\n");
}
static void delay_us(uint32_t us)
{
    uint32_t start=DWT->CYCCNT,cycles=(SystemCoreClock/1000000U)*us;
    while ((uint32_t)(DWT->CYCCNT-start)<cycles) { }
}
static void software_query(void)
{
    const uint8_t query[]={0xFF,0xAA,0x27,0x02,0x00};
    uint32_t crl=GPIOB->CRL,cr1=USART1->CR1,odr=GPIOB->ODR;
    unsigned high=0,low=0,mismatch=0;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    uint32_t irq_mask=__get_PRIMASK();__disable_irq();
    USART1->CR1&=~USART_CR1_TE;
    GPIOB->BSRR=GPIO_PIN_6;
    GPIOB->CRL=(crl&~(0xFU<<24))|(0x3U<<24); /* PB6 push-pull GPIO. */
    delay_us(208);
    for (unsigned i=0;i<sizeof(query);i++) for (unsigned bit=0;bit<10;bit++) {
        unsigned value=bit==0 ? 0U : bit==9 ? 1U : (query[i]>>(bit-1))&1U;
        GPIOB->BSRR=value ? GPIO_PIN_6 : ((uint32_t)GPIO_PIN_6<<16);
        delay_us(52);unsigned actual=(GPIOB->IDR>>6)&1U;
        if (actual) high++;else low++;
        if (actual!=value) mismatch++;
        delay_us(52);
    }
    GPIOB->BSRR=GPIO_PIN_6;delay_us(104);GPIOB->CRL=crl;
    if (odr&GPIO_PIN_6) GPIOB->BSRR=GPIO_PIN_6;else GPIOB->BRR=GPIO_PIN_6;
    USART1->CR1=cr1;
    if (!irq_mask) __enable_irq();
    (void)HAL_IWDG_Refresh(&hiwdg);
    BSP_UART_Printf("[DIAG] SOFTWARE_QUERY baud=9600 high_read=%u low_read=%u mismatch=%u\r\n",high,low,mismatch);
}

static void drain_delay(uint32_t duration)
{
    uint32_t start=HAL_GetTick();uint8_t b[64];
    while (HAL_GetTick()-start<duration) {
        (void)HAL_IWDG_Refresh(&hiwdg);
        (void)BSP_UART_Read(BSP_UART_ELRS,b,sizeof(b));
    }
}
static uint32_t window(const char *label,uint32_t duration)
{
    uint32_t start=HAL_GetTick(),recoveries=BSP_UART_RxDma_GetRecoveries();
    uint8_t bytes[64],sample[512],reg_frame[11];unsigned used=0,reg_used=0;uint32_t raw=0,reg_replies=0;
    bool full_dump=!strcmp(label,"QUERY_RESPONSE");
    DrvJy60_Init(&parser);
    while (HAL_GetTick()-start<duration) {
        (void)HAL_IWDG_Refresh(&hiwdg);
        uint16_t n=BSP_UART_Read(BSP_UART_ELRS,bytes,sizeof(bytes));raw+=n;
        for (unsigned i=0;i<n;i++) {
            if (used<(full_dump ? sizeof(sample) : 32U)) sample[used++]=bytes[i];
            (void)DrvJy60_Push(&parser,bytes[i],HAL_GetTick());
            reg_frame[reg_used++]=bytes[i];
            if (reg_used==11) {
                uint8_t sum=0;for (unsigned j=0;j<10;j++) sum=(uint8_t)(sum+reg_frame[j]);
                if (reg_frame[0]==0x55 && reg_frame[1]==0x5F && sum==reg_frame[10]) reg_replies++;
                memmove(reg_frame,reg_frame+1,10);reg_used=10;
            }
        }
    }
    char hex[65];
    unsigned preview=used<32U ? used : 32U;
    for (unsigned i=0;i<preview;i++) (void)snprintf(hex+2*i,3,"%02X",sample[i]);
    hex[preview*2]=0;
    uint32_t elapsed=HAL_GetTick()-start;
    BSP_UART_Printf("[DIAG] %s baud=%lu raw=%lu good=%lu acc=%lu gyro=%lu angle=%lu errors=%lu recoveries=%lu\r\n",
        label,(unsigned long)baud,(unsigned long)raw,(unsigned long)parser.good_frames,
        (unsigned long)(parser.acceleration.count*1000/elapsed),(unsigned long)(parser.gyro.count*1000/elapsed),
        (unsigned long)(parser.angle.count*1000/elapsed),(unsigned long)parser.checksum_errors,
        (unsigned long)(BSP_UART_RxDma_GetRecoveries()-recoveries));
    BSP_UART_Printf("[DIAG] RAW baud=%lu hex=%s rx_pin=%u dma_left=%lu sr=0x%lX cr3=0x%lX\r\n",
        (unsigned long)baud,hex,(unsigned)((GPIOB->IDR>>7)&1U),
        (unsigned long)__HAL_DMA_GET_COUNTER(huart1.hdmarx),(unsigned long)USART1->SR,(unsigned long)USART1->CR3);
    BSP_UART_Printf("[DIAG] REGISTER_REPLIES count=%lu\r\n",(unsigned long)reg_replies);
    if (full_dump) for (unsigned off=0;off<used;off+=32U) {
        unsigned n=used-off<32U ? used-off : 32U;
        for (unsigned i=0;i<n;i++) (void)snprintf(hex+2*i,3,"%02X",sample[off+i]);
        hex[n*2]=0;BSP_UART_Printf("[DIAG] QUERY_RAW offset=%u hex=%s\r\n",off,hex);
    }
    /* Wrong-baud UART recovery can replay stale DMA memory. Never select
     * a noisy/recovering candidate merely because its parser saw headers. */
    return parser.checksum_errors==0 && BSP_UART_RxDma_GetRecoveries()==recoveries ? parser.good_frames : 0;
}
static bool set_baud(uint32_t next)
{
    baud=next;bool ok=BSP_UART_ImuSetBaud(baud)==ARA_OK;
    BSP_UART_Printf("[DIAG] LOCAL_BAUD baud=%lu ok=%u\r\n",(unsigned long)baud,ok);
    return ok;
}
static bool write_register(uint8_t reg,uint16_t value)
{
    uint8_t b[]={0xFF,0xAA,reg,(uint8_t)value,(uint8_t)(value>>8)};
    bool ok=BSP_UART_ImuWrite(b,sizeof(b))==ARA_OK;
    BSP_UART_Printf("[DIAG] WRITE reg=0x%02X value=0x%04X sent=%u baud=%lu\r\n",reg,value,ok,(unsigned long)baud);
    drain_delay(reg==0x69U ? 200U : 30U);
    return ok;
}
static void scan(void)
{
    static const uint32_t rates[]={115200,9600,230400,57600,38400,19200,4800};
    uint32_t best=0,selected=115200;
    BSP_UART_Printf("[DIAG] SCAN_START passive_only=1 no_sensor_writes=1\r\n");
    for (unsigned i=0;i<sizeof(rates)/sizeof(rates[0]);i++) {
        if (!set_baud(rates[i])) continue;
        drain_delay(50);uint32_t good=window("SCAN",1000);
        if (good>best) { best=good;selected=rates[i]; }
    }
    (void)set_baud(selected);drain_delay(100);
    BSP_UART_Printf("[DIAG] SCAN_END selected=%lu good=%lu action=%s\r\n",
        (unsigned long)selected,(unsigned long)best,best ? "READ_ONLY_MONITOR" : "CHECK_POWER_WIRING_OR_DISABLED_OUTPUT");
}
static void configure_steps(void)
{
    /* Only on explicit 'c'. Never calibrate, save EEPROM, zero or drive HX8. */
    BSP_UART_Printf("[DIAG] CONFIG_START\r\n");
    if (!write_register(0x69,0xB588) || !write_register(0x04,6) || !set_baud(115200)) return;
    if (!window("AFTER_BAUD",1000)) {
        BSP_UART_Printf("[DIAG] CONFIG_FAILED no_valid_frames_at_115200 restoring_local_9600\r\n");
        (void)set_baud(9600);(void)window("RESTORED_9600",1000);return;
    }
    if (!write_register(0x69,0xB588) || !write_register(0x02,0x000E)) return;
    (void)window("AFTER_CONTENT",1000);
    if (!write_register(0x69,0xB588) || !write_register(0x03,0x000B)) return;
    (void)window("AFTER_RATE",1000);
    if (!write_register(0x69,0xB588) || !write_register(0x1F,2)) return;
    (void)window("AFTER_BANDWIDTH",1000);
    BSP_UART_Printf("[DIAG] CONFIG_END verify_measured_rates=1\r\n");
}
void App_Jy61pDiagnostic_Run(void)
{
    BSP_UART_Printf("[DIAG] BOOT JY61P_ONLY USART1_PB6_TX_PB7_RX hx8_tx=OFF pwm=OFF commands=r:rescan,c:configure,q:read_registers\r\n");
    drain_delay(1200);scan();
    for (;;) {
        (void)window("MONITOR",1000);
        char key;
        while (SEGGER_RTT_Read(0,&key,1)==1) {
            if (key=='r') scan();
            else if (key=='c') configure_steps();
            else if (key=='p') unloaded_pin_test();
            else if (key=='q') {
                uint8_t query[]={0xFF,0xAA,0x27,0x02,0x00};
                unsigned high=0,low=0,edges=0,last=(GPIOB->IDR>>6)&1U;
                uint32_t started=HAL_GetTick();
                bool ok=HAL_UART_Transmit_IT(&huart1,query,sizeof(query))==HAL_OK;
                while (ok && huart1.gState==HAL_UART_STATE_BUSY_TX && HAL_GetTick()-started<50U) {
                    unsigned value=(GPIOB->IDR>>6)&1U;
                    if (value) high++;else low++;
                    if (value!=last) { edges++;last=value; }
                    (void)HAL_IWDG_Refresh(&hiwdg);
                }
                if (huart1.gState==HAL_UART_STATE_BUSY_TX) { (void)HAL_UART_AbortTransmit(&huart1);ok=false; }
                BSP_UART_Printf("[DIAG] READ_REGISTERS sent=%u baud=%lu tx_pin_high=%u tx_pin_low=%u tx_pin_edges=%u\r\n",ok,(unsigned long)baud,high,low,edges);
                BSP_UART_Printf("[DIAG] GPIO pb_crl=0x%lX afio_mapr=0x%lX brr=0x%lX cr1=0x%lX\r\n",(unsigned long)GPIOB->CRL,(unsigned long)AFIO->MAPR,(unsigned long)USART1->BRR,(unsigned long)USART1->CR1);
                (void)window("QUERY_RESPONSE",1000);
            }
            else if (key=='b') {
                if (baud==9600) { software_query();(void)window("QUERY_RESPONSE",1000); }
                else BSP_UART_Printf("[DIAG] SOFTWARE_QUERY requires_local_9600\r\n");
            }
        }
    }
}
#endif
