#include "app_jy60_demo.h"
#if defined(JY60_UART_DEMO)
#include "drv_jy60.h"
#include "bsp_uart.h"
#include "SEGGER_RTT.h"
#include "usart.h"
#include "iwdg.h"

static Jy60Parser_t parser;

static void print_angle(const char *label, int32_t milli_deg)
{
    uint32_t mag=(uint32_t)(milli_deg<0 ? -milli_deg : milli_deg);
    uint32_t hundredths=(mag+5U)/10U;
    BSP_UART_Printf("%s=%s%lu.%02lu",label,
        milli_deg<0 && hundredths ? "-" : "+",
        (unsigned long)(hundredths/100U),(unsigned long)(hundredths%100U));
}

static void print_vector(const char *label, const Jy60Vector_t *v, const char *unit)
{
    BSP_UART_Printf("[JY60] %s", label);
    for (unsigned i=0; i<3U; i++) {
        int32_t x=v->milli[i]; uint32_t mag=(uint32_t)(x<0 ? -x : x);
        BSP_UART_Printf(" %s%lu.%03lu", x<0 ? "-" : "+",
                        (unsigned long)(mag/1000U), (unsigned long)(mag%1000U));
    }
    BSP_UART_Printf(" %s tail_raw=%d age=%lu ms\r\n", unit, (int)v->tail_raw,
                    (unsigned long)(HAL_GetTick()-v->timestamp_ms));
}

void App_Jy60Demo_Run(void)
{
    SEGGER_RTT_Init();
    DrvJy60_Init(&parser);
    BSP_UART_Printf("\r\n[FW] JY60_UART_DEMO USART1 PB7=RX PB6=TX baud=9600 8N1; HX8/PWM/HC13 disabled\r\n");
    BSP_UART_Printf("[JY60] socket: pin1=GND pin2=5V pin3=MCU_RX<-JY60_TX pin4=MCU_TX->JY60_RX\r\n");
    BSP_UART_Printf("[INFO] Sensor:20Hz; display:1Hz. Roll=X Pitch=Y Yaw=Z (drifts). d=details\r\n");
    uint32_t printed=HAL_GetTick(), uart_errors=0;
    bool detailed=false;
    uint8_t first_bytes[16]; unsigned first_count=0;
    for (;;) {
        (void)HAL_IWDG_Refresh(&hiwdg);
        uint8_t byte;
        if (USART1->SR & (USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
            uart_errors++;
            __HAL_UART_CLEAR_OREFLAG(&huart1);
        }
        HAL_StatusTypeDef result=HAL_UART_Receive(&huart1,&byte,1,2);
        uint32_t now=HAL_GetTick();
        if (result==HAL_OK) {
            if (first_count<sizeof(first_bytes)) first_bytes[first_count++]=byte;
            (void)DrvJy60_Push(&parser,byte,now);
        } else if (result!=HAL_TIMEOUT) uart_errors++;
        uint8_t keys[8];
        unsigned key_count=SEGGER_RTT_Read(0,keys,sizeof(keys));
        for (unsigned i=0;i<key_count;i++) {
            if (keys[i]=='d') {
                detailed=!detailed;
                BSP_UART_Printf("[DISPLAY] %s\r\n", detailed ? "DETAIL" : "COMPACT");
            } else if (keys[i]=='?') {
                BSP_UART_Printf("[HELP] d=toggle details; sensor-only demo, no servo control\r\n");
            }
        }
        if ((uint32_t)(now-printed)<1000U) continue;
        printed=now;
        bool online=parser.angle.count && (uint32_t)(now-parser.angle.timestamp_ms)<200U;
        BSP_UART_Printf("[STATUS] %s | Sensor=%lums Display=1000ms | ChecksumErrors=%lu UartErrors=%lu\r\n",
            online ? "ONLINE" : parser.rx_bytes ? "ANGLE_STALE" : "NO_DATA",
            (unsigned long)parser.angle_period_ms,(unsigned long)parser.checksum_errors,
            (unsigned long)uart_errors);
        if (online) {
            BSP_UART_Printf("[ANGLE deg] ");
            print_angle("Roll(X)",parser.angle.milli[0]);
            BSP_UART_Printf(" | ");
            print_angle("Pitch(Y)",parser.angle.milli[1]);
            BSP_UART_Printf(" | ");
            print_angle("Yaw(Z)",parser.angle.milli[2]);
            BSP_UART_Printf("\r\n");
        } else BSP_UART_Printf("[ANGLE] No valid recent sample; check power and TX->PB7\r\n");
        if (!detailed) continue;
        BSP_UART_Printf("[JY60] %s rx=%lu good=%lu acc=%lu gyro=%lu angle=%lu checksum_err=%lu uart_err=%lu period=%lu ms\r\n",
            online ? "ONLINE" : parser.rx_bytes ? "NO_VALID_RECENT_ANGLE" : "NO_BYTES",
            (unsigned long)parser.rx_bytes,(unsigned long)parser.good_frames,
            (unsigned long)parser.acceleration.count,(unsigned long)parser.gyro.count,
            (unsigned long)parser.angle.count,(unsigned long)parser.checksum_errors,
            (unsigned long)uart_errors,(unsigned long)parser.angle_period_ms);
        if (parser.angle.count) print_vector("angle XYZ/RPY:",&parser.angle,"deg");
        if (parser.acceleration.count) print_vector("acc XYZ:",&parser.acceleration,"g");
        if (parser.gyro.count) print_vector("gyro XYZ:",&parser.gyro,"deg/s");
        if (!online) {
            BSP_UART_Printf("[JY60] first_bytes:");
            for (unsigned i=0;i<first_count;i++) BSP_UART_Printf(" %02X",first_bytes[i]);
            BSP_UART_Printf("\r\n");
        }
    }
}
#endif
