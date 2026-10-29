// OSAL backend for the host simulation kernel.
#include "osal/osal.h"
#include "sim/kernel.hpp"

using sim::Kernel;

namespace {
inline uint64_t to_us(uint32_t ms) { return ms == OSAL_WAIT_FOREVER ? sim::kNever : uint64_t{ms} * 1000; }
}  // namespace

extern "C" {

osal_task_t osal_task_create(const char* name, osal_task_fn fn, void* arg, uint32_t stack_bytes, uint32_t prio) {
  return reinterpret_cast<osal_task_t>(Kernel::get().create_task(name, fn, arg, stack_bytes, prio));
}

uint32_t osal_tick_ms(void) { return static_cast<uint32_t>(Kernel::get().now_us() / 1000); }
uint64_t osal_time_us(void) { return Kernel::get().now_us(); }

void osal_delay_ms(uint32_t ms) {
  Kernel& k = Kernel::get();
  k.delay_until(k.now_us() + uint64_t{ms} * 1000);
}

void osal_delay_until(uint32_t* prev_wake_ms, uint32_t period_ms) {
  *prev_wake_ms += period_ms;
  Kernel::get().delay_until(uint64_t{*prev_wake_ms} * 1000);
}

osal_queue_t osal_queue_create(uint32_t length, uint32_t item_size) {
  return reinterpret_cast<osal_queue_t>(Kernel::get().create_queue(length, item_size));
}

bool osal_queue_send(osal_queue_t q, const void* item, uint32_t timeout_ms) {
  return Kernel::get().queue_send(reinterpret_cast<sim::Queue*>(q), item, to_us(timeout_ms));
}

bool osal_queue_send_from_isr(osal_queue_t q, const void* item) {
  return Kernel::get().queue_send_isr(reinterpret_cast<sim::Queue*>(q), item);
}

bool osal_queue_receive(osal_queue_t q, void* item, uint32_t timeout_ms) {
  return Kernel::get().queue_recv(reinterpret_cast<sim::Queue*>(q), item, to_us(timeout_ms));
}

uint32_t osal_queue_count(osal_queue_t q) {
  return static_cast<uint32_t>(reinterpret_cast<sim::Queue*>(q)->items.size());
}

void osal_cpu_work_us(uint32_t us) { Kernel::get().work(us); }

void osal_hw_watchdog_start(uint32_t timeout_ms) {
  Kernel& k = Kernel::get();
  k.wdg_start(*k.current_node(), uint64_t{timeout_ms} * 1000);
}

void osal_hw_watchdog_kick(void) {
  Kernel& k = Kernel::get();
  k.wdg_kick(*k.current_node());
}

osal_reset_cause_t osal_reset_cause(void) { return Kernel::get().current_node()->reset_cause; }

void osal_system_reset(void) {
  Kernel& k = Kernel::get();
  k.reset_node(*k.current_node(), OSAL_RESET_SOFTWARE);
  // the calling task no longer exists; hand control back for good
  if (k.current_task() != nullptr) k.block_forever();
}

}  // extern "C"
