/*
 * Integration example only. Merge these calls into the main.c generated for
 * your exact STM32 board by STM32CubeMX. Do not replace CubeMX clock, GPIO,
 * cache, MPU, Ethernet, PHY, or linker configuration with this file.
 */

#include "main.h"
#include "lwip.h"
#include "modbus.h"
#include "modbus_tcp.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_LWIP_Init();

    mb_init();
    mbtcp_init();

    for (;;) {
        /* Present in CubeMX no-RTOS lwIP projects. */
        MX_LWIP_Process();

        /* Example application updates: */
        /* mb_set_ireg(0, latest_adc_value); */
        /* mb_set_dinput(0, HAL_GPIO_ReadPin(INPUT_GPIO_Port, INPUT_Pin)); */
        mbtcp_poll();
    }
}
