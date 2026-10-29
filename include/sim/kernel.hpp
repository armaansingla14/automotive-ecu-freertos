#pragma once
#include <ucontext.h>

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "osal/osal.h"

namespace sim {

constexpr uint64_t kNever = ~uint64_t{0};

struct Node;
struct Queue;

enum class TaskState : uint8_t { Ready, Working, Blocked, Dead };

struct Task {
  int id = 0;
  Node* node = nullptr;
  std::string name;
  uint32_t prio = 0;
  osal_task_fn fn = nullptr;
  void* arg = nullptr;
  ucontext_t ctx{};
  std::unique_ptr<char[]> stack;
  size_t stack_size = 0;
  TaskState state = TaskState::Ready;
  uint64_t wake_us = kNever;    ///< timeout for Blocked
  uint64_t work_left_us = 0;    ///< remaining CPU work for Working
  uint64_t ready_seq = 0;       ///< FIFO order among equal priorities
  Queue* wait_q = nullptr;      ///< queue being waited on
  bool wait_send = false;       ///< waiting for space (true) or data (false)
  bool wait_ok = false;         ///< result of the wait
  uint64_t cpu_us = 0;          ///< consumed virtual CPU time
};

struct Queue {
  Node* node = nullptr;
  size_t item_size = 0;
  size_t capacity = 0;
  std::deque<std::vector<uint8_t>> items;
  std::vector<Task*> recv_waiters;
  std::vector<Task*> send_waiters;
  uint32_t isr_overflows = 0;
};

/// One simulated MCU: its tasks, queues, hardware watchdog and reset state.
struct Node {
  int id = 0;
  std::string name;
  std::function<void()> boot;  ///< firmware entry: creates queues/tasks (re-run after reset)
  std::vector<std::unique_ptr<Task>> tasks;
  std::vector<std::unique_ptr<Queue>> queues;
  bool up = false;               ///< firmware running (false during reset/boot)
  uint64_t boot_delay_us = 20000;  ///< reset -> firmware start
  uint64_t power_on_us = 0;        ///< first boot time after start()
  // independent watchdog
  bool wdg_on = false;
  uint64_t wdg_timeout_us = 0;
  uint64_t wdg_deadline_us = kNever;
  osal_reset_cause_t reset_cause = OSAL_RESET_POWER_ON;
  uint32_t resets = 0;
  std::vector<uint64_t> reset_times_us;
  std::function<void(Node&)> on_reset;  ///< hook for peripherals (CAN controller etc.)
  void* user = nullptr;                 ///< firmware instance pointer
};

/// Deterministic virtual-time kernel emulating one FreeRTOS scheduler per
/// node. Tasks are POSIX ucontext coroutines; exactly one task executes at a
/// time and code runs in zero virtual time except where it declares CPU work
/// (osal_cpu_work_us). Per node, the highest-priority Ready/Working task owns
/// the CPU, so a higher-priority task becoming ready preempts a lower one's
/// remaining work (preemption at work granularity).
class Kernel {
 public:
  static Kernel& get();

  void clear();  ///< remove all nodes and events, time = 0
  Node& add_node(const std::string& name, std::function<void()> boot);
  Node* node(int id) { return nodes_[static_cast<size_t>(id)].get(); }
  size_t node_count() const { return nodes_.size(); }
  void start();  ///< power-on all nodes at t = 0
  void run_until(uint64_t t_us);
  uint64_t now_us() const { return now_; }

  /// Schedule a callback in interrupt/hardware context.
  void at(uint64_t t_us, std::function<void()> fn);

  // --- used by the OSAL backend ---
  Node* current_node() { return cur_node_; }
  Task* current_task() { return cur_task_; }
  Task* create_task(const char* name, osal_task_fn fn, void* arg, size_t stack, uint32_t prio);
  Queue* create_queue(size_t len, size_t item);
  void delay_until(uint64_t t_us);
  void work(uint64_t us);
  bool queue_send(Queue* q, const void* item, uint64_t timeout_us);
  bool queue_send_isr(Queue* q, const void* item);
  bool queue_recv(Queue* q, void* item, uint64_t timeout_us);
  void wdg_start(Node& n, uint64_t timeout_us);
  void wdg_kick(Node& n);
  void reset_node(Node& n, osal_reset_cause_t cause);

  /// Leave the current (already killed) task; never returns.
  void block_forever() { block_current(); }

  uint64_t context_switches() const { return switches_; }

 private:
  struct Event {
    uint64_t t;
    uint64_t seq;
    std::function<void()> fn;
    bool operator>(const Event& o) const { return t != o.t ? t > o.t : seq > o.seq; }
  };

  static void trampoline();
  void switch_to(Task* t);
  void block_current();
  Task* pick(Node& n);
  void make_ready(Task* t);
  void boot_node(Node& n);
  void kill_tasks(Node& n);

  std::vector<std::unique_ptr<Node>> nodes_;
  std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events_;
  uint64_t now_ = 0;
  uint64_t seq_ = 0;
  uint64_t ready_seq_ = 0;
  int next_task_id_ = 0;
  ucontext_t kernel_ctx_{};
  Task* cur_task_ = nullptr;
  Node* cur_node_ = nullptr;
  uint64_t switches_ = 0;
  std::vector<std::unique_ptr<Task>> graveyard_;
};

}  // namespace sim
