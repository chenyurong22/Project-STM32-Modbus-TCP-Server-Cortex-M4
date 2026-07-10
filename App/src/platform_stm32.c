#include "platform_port.h"

#include "lwip/opt.h"

#ifdef WITH_RTOS
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t mb_mutex;

void mb_lock(void)
{
    if (mb_mutex == NULL) {
        mb_mutex = xSemaphoreCreateMutex();
    }
    if (mb_mutex != NULL) {
        (void)xSemaphoreTake(mb_mutex, portMAX_DELAY);
    }
}

void mb_unlock(void)
{
    if (mb_mutex != NULL) {
        (void)xSemaphoreGive(mb_mutex);
    }
}
#endif

/* CubeMX commonly supplies sys_now(). This weak fallback uses the HAL tick. */
u32_t __attribute__((weak)) sys_now(void)
{
    extern volatile uint32_t uwTick;
    return (u32_t)uwTick;
}
