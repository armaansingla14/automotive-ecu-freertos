/*
 * Portable OS abstraction used by the ECU firmware.
 *
 * Two backends implement this API:
 *   port/freertos/osal_freertos.c  - FreeRTOS (xTaskCreate, xQueue*, vTaskDelayUntil)
 *   src/osal/osal_sim.cpp          - host simulation on a deterministic,
 *                                    priority-preemptive virtual-time kernel
 *                                    (POSIX ucontext tasks, see src/sim/kernel.cpp)
 *
 * Semantics follow FreeRTOS: fixed-priority preemptive scheduling, higher
 * number = higher priority, queues copy items by value, timeouts in ms.
 */
#ifndef OSAL_H
#define OSAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OSAL_WAIT_FOREVER 0xFFFFFFFFu
#define OSAL_NO_WAIT 0u

typedef struct osal_task_s* osal_task_t;
typedef struct osal_queue_s* osal_queue_t;
typedef void (*osal_task_fn)(void* arg);

osal_task_t osal_task_create(const char* name, osal_task_fn fn, void* arg, uint32_t stack_bytes,
                             uint32_t priority);

/* time */
uint32_t osal_tick_ms(void);
uint64_t osal_time_us(void);
void osal_delay_ms(uint32_t ms);
/* periodic wake-up without drift; *prev_wake is updated (vTaskDelayUntil) */
void osal_delay_until(uint32_t* prev_wake_ms, uint32_t period_ms);

/* queues */
osal_queue_t osal_queue_create(uint32_t length, uint32_t item_size);
bool osal_queue_send(osal_queue_t q, const void* item, uint32_t timeout_ms);
bool osal_queue_send_from_isr(osal_queue_t q, const void* item);
bool osal_queue_receive(osal_queue_t q, void* item, uint32_t timeout_ms);
uint32_t osal_queue_count(osal_queue_t q);

/* Accounts for CPU execution time of the calling task. On target this is a
 * no-op (real code takes real time); on the host simulation it advances the
 * task's virtual execution so that scheduling and latency are realistic. */
void osal_cpu_work_us(uint32_t us);

/* Independent hardware watchdog (IWDG). */
void osal_hw_watchdog_start(uint32_t timeout_ms);
void osal_hw_watchdog_kick(void);
/* Reset cause as latched by the MCU at boot. */
typedef enum { OSAL_RESET_POWER_ON = 0, OSAL_RESET_WATCHDOG = 1, OSAL_RESET_SOFTWARE = 2 } osal_reset_cause_t;
osal_reset_cause_t osal_reset_cause(void);
void osal_system_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* OSAL_H */
