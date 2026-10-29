/*
 * OSAL backend for FreeRTOS (target build, e.g. STM32 + CMSIS + FreeRTOS
 * kernel V10/V11). Not compiled in the host build; see README.
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include "task.h"

#include "osal/osal.h"

extern IWDG_HandleTypeDef hiwdg;

osal_task_t osal_task_create(const char* name, osal_task_fn fn, void* arg, uint32_t stack_bytes,
                             uint32_t priority) {
  TaskHandle_t h = NULL;
  if (xTaskCreate(fn, name, (configSTACK_DEPTH_TYPE)(stack_bytes / sizeof(StackType_t)), arg,
                  (UBaseType_t)priority, &h) != pdPASS) {
    return NULL;
  }
  return (osal_task_t)h;
}

uint32_t osal_tick_ms(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

uint64_t osal_time_us(void) {
  /* DWT cycle counter extended by the tick count would be used here; the
   * millisecond tick is sufficient for the firmware itself. */
  return (uint64_t)osal_tick_ms() * 1000u;
}

void osal_delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

void osal_delay_until(uint32_t* prev_wake_ms, uint32_t period_ms) {
  TickType_t prev = pdMS_TO_TICKS(*prev_wake_ms);
  vTaskDelayUntil(&prev, pdMS_TO_TICKS(period_ms));
  *prev_wake_ms = (uint32_t)(prev * portTICK_PERIOD_MS);
}

static TickType_t to_ticks(uint32_t ms) { return ms == OSAL_WAIT_FOREVER ? portMAX_DELAY : pdMS_TO_TICKS(ms); }

osal_queue_t osal_queue_create(uint32_t length, uint32_t item_size) {
  return (osal_queue_t)xQueueCreate(length, item_size);
}

bool osal_queue_send(osal_queue_t q, const void* item, uint32_t timeout_ms) {
  return xQueueSend((QueueHandle_t)q, item, to_ticks(timeout_ms)) == pdPASS;
}

bool osal_queue_send_from_isr(osal_queue_t q, const void* item) {
  BaseType_t woken = pdFALSE;
  const bool ok = xQueueSendFromISR((QueueHandle_t)q, item, &woken) == pdPASS;
  portYIELD_FROM_ISR(woken);
  return ok;
}

bool osal_queue_receive(osal_queue_t q, void* item, uint32_t timeout_ms) {
  return xQueueReceive((QueueHandle_t)q, item, to_ticks(timeout_ms)) == pdPASS;
}

uint32_t osal_queue_count(osal_queue_t q) { return (uint32_t)uxQueueMessagesWaiting((QueueHandle_t)q); }

void osal_cpu_work_us(uint32_t us) { (void)us; }

void osal_hw_watchdog_start(uint32_t timeout_ms) {
  /* LSI = 32 kHz, prescaler 32 -> 1 ms per count */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = timeout_ms > 4095u ? 4095u : timeout_ms;
  HAL_IWDG_Init(&hiwdg);
}

void osal_hw_watchdog_kick(void) { HAL_IWDG_Refresh(&hiwdg); }

osal_reset_cause_t osal_reset_cause(void) {
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) return OSAL_RESET_WATCHDOG;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) return OSAL_RESET_SOFTWARE;
  return OSAL_RESET_POWER_ON;
}

void osal_system_reset(void) { NVIC_SystemReset(); }
