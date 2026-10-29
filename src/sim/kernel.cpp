#include "sim/kernel.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace sim {

Kernel& Kernel::get() {
  static Kernel k;
  return k;
}

void Kernel::clear() {
  nodes_.clear();
  graveyard_.clear();
  while (!events_.empty()) events_.pop();
  now_ = 0;
  seq_ = 0;
  ready_seq_ = 0;
  next_task_id_ = 0;
  switches_ = 0;
  cur_task_ = nullptr;
  cur_node_ = nullptr;
}

Node& Kernel::add_node(const std::string& name, std::function<void()> boot) {
  auto n = std::make_unique<Node>();
  n->id = static_cast<int>(nodes_.size());
  n->name = name;
  n->boot = std::move(boot);
  nodes_.push_back(std::move(n));
  return *nodes_.back();
}

void Kernel::start() {
  for (auto& n : nodes_) {
    if (n->power_on_us == 0) {
      boot_node(*n);
    } else {
      Node* np = n.get();
      at(n->power_on_us, [this, np]() { boot_node(*np); });
    }
  }
}

void Kernel::at(uint64_t t_us, std::function<void()> fn) {
  events_.push(Event{std::max(t_us, now_), seq_++, std::move(fn)});
}

void Kernel::boot_node(Node& n) {
  n.up = true;
  Node* prev = cur_node_;
  cur_node_ = &n;
  n.boot();
  cur_node_ = prev;
}

Task* Kernel::create_task(const char* name, osal_task_fn fn, void* arg, size_t stack, uint32_t prio) {
  Node* n = cur_node_;
  if (n == nullptr) throw std::logic_error("osal_task_create outside of a node context");
  auto t = std::make_unique<Task>();
  t->id = next_task_id_++;
  t->node = n;
  t->name = name;
  t->prio = prio;
  t->fn = fn;
  t->arg = arg;
  t->stack_size = std::max<size_t>(stack, 32 * 1024);
  t->stack.reset(new char[t->stack_size]);
  getcontext(&t->ctx);
  t->ctx.uc_stack.ss_sp = t->stack.get();
  t->ctx.uc_stack.ss_size = t->stack_size;
  t->ctx.uc_link = nullptr;
  makecontext(&t->ctx, &Kernel::trampoline, 0);
  t->state = TaskState::Ready;
  t->ready_seq = ready_seq_++;
  n->tasks.push_back(std::move(t));
  return n->tasks.back().get();
}

Queue* Kernel::create_queue(size_t len, size_t item) {
  Node* n = cur_node_;
  if (n == nullptr) throw std::logic_error("osal_queue_create outside of a node context");
  auto q = std::make_unique<Queue>();
  q->node = n;
  q->capacity = len;
  q->item_size = item;
  n->queues.push_back(std::move(q));
  return n->queues.back().get();
}

void Kernel::trampoline() {
  Kernel& k = get();
  Task* t = k.cur_task_;
  t->fn(t->arg);
  t->state = TaskState::Dead;  // returning from a task function ends it
  swapcontext(&t->ctx, &k.kernel_ctx_);
}

void Kernel::switch_to(Task* t) {
  cur_task_ = t;
  cur_node_ = t->node;
  ++switches_;
  swapcontext(&kernel_ctx_, &t->ctx);
  cur_task_ = nullptr;
  cur_node_ = nullptr;
}

void Kernel::block_current() {
  Task* t = cur_task_;
  swapcontext(&t->ctx, &kernel_ctx_);
}

void Kernel::make_ready(Task* t) {
  t->state = TaskState::Ready;
  t->wake_us = kNever;
  t->ready_seq = ready_seq_++;
}

Task* Kernel::pick(Node& n) {
  Task* best = nullptr;
  for (auto& up : n.tasks) {
    Task* t = up.get();
    if (t->state != TaskState::Ready && t->state != TaskState::Working) continue;
    if (!best || t->prio > best->prio || (t->prio == best->prio && t->ready_seq < best->ready_seq)) best = t;
  }
  return best;
}

void Kernel::delay_until(uint64_t t_us) {
  Task* t = cur_task_;
  if (t_us <= now_) return;
  t->state = TaskState::Blocked;
  t->wake_us = t_us;
  t->wait_q = nullptr;
  block_current();
}

void Kernel::work(uint64_t us) {
  if (us == 0) return;
  Task* t = cur_task_;
  t->state = TaskState::Working;
  t->work_left_us = us;
  block_current();
}

namespace {
Task* pop_best_waiter(std::vector<Task*>& w) {
  if (w.empty()) return nullptr;
  auto it = std::max_element(w.begin(), w.end(), [](Task* a, Task* b) {
    return a->prio != b->prio ? a->prio < b->prio : a->ready_seq > b->ready_seq;
  });
  Task* t = *it;
  w.erase(it);
  return t;
}
}  // namespace

bool Kernel::queue_send(Queue* q, const void* item, uint64_t timeout_us) {
  Task* t = cur_task_;
  const uint64_t deadline = timeout_us == kNever ? kNever : now_ + timeout_us;
  for (;;) {
    if (q->items.size() < q->capacity) {
      const auto* p = static_cast<const uint8_t*>(item);
      q->items.emplace_back(p, p + q->item_size);
      if (Task* r = pop_best_waiter(q->recv_waiters)) make_ready(r);
      return true;
    }
    if (timeout_us == 0 || now_ >= deadline) return false;
    t->state = TaskState::Blocked;
    t->wake_us = deadline;
    t->wait_q = q;
    t->wait_send = true;
    q->send_waiters.push_back(t);
    block_current();
  }
}

bool Kernel::queue_send_isr(Queue* q, const void* item) {
  if (q->items.size() >= q->capacity) {
    ++q->isr_overflows;
    return false;
  }
  const auto* p = static_cast<const uint8_t*>(item);
  q->items.emplace_back(p, p + q->item_size);
  if (Task* r = pop_best_waiter(q->recv_waiters)) make_ready(r);
  return true;
}

bool Kernel::queue_recv(Queue* q, void* item, uint64_t timeout_us) {
  Task* t = cur_task_;
  const uint64_t deadline = timeout_us == kNever ? kNever : now_ + timeout_us;
  for (;;) {
    if (!q->items.empty()) {
      std::memcpy(item, q->items.front().data(), q->item_size);
      q->items.pop_front();
      if (Task* s = pop_best_waiter(q->send_waiters)) make_ready(s);
      return true;
    }
    if (timeout_us == 0 || now_ >= deadline) return false;
    t->state = TaskState::Blocked;
    t->wake_us = deadline;
    t->wait_q = q;
    t->wait_send = false;
    q->recv_waiters.push_back(t);
    block_current();
  }
}

void Kernel::wdg_start(Node& n, uint64_t timeout_us) {
  n.wdg_on = true;
  n.wdg_timeout_us = timeout_us;
  n.wdg_deadline_us = now_ + timeout_us;
}

void Kernel::wdg_kick(Node& n) {
  if (n.wdg_on) n.wdg_deadline_us = now_ + n.wdg_timeout_us;
}

void Kernel::kill_tasks(Node& n) {
  for (auto& q : n.queues) {
    q->recv_waiters.clear();
    q->send_waiters.clear();
  }
  for (auto& t : n.tasks) {
    t->state = TaskState::Dead;
    graveyard_.push_back(std::move(t));
  }
  n.tasks.clear();
  // queues stay allocated (stale handles may still exist) but are emptied
  for (auto& q : n.queues) q->items.clear();
}

void Kernel::reset_node(Node& n, osal_reset_cause_t cause) {
  kill_tasks(n);
  n.up = false;
  n.wdg_on = false;
  n.wdg_deadline_us = kNever;
  n.reset_cause = cause;
  ++n.resets;
  n.reset_times_us.push_back(now_);
  if (n.on_reset) n.on_reset(n);
  Node* np = &n;
  at(now_ + n.boot_delay_us, [this, np]() { boot_node(*np); });
}

void Kernel::run_until(uint64_t t_end) {
  for (;;) {
    // 1) run every task that has code to execute at the current instant
    bool progress = true;
    while (progress) {
      progress = false;
      for (auto& n : nodes_) {
        if (!n->up) continue;
        Task* t = pick(*n);
        if (t && t->state == TaskState::Ready) {
          switch_to(t);
          progress = true;
        }
      }
    }
    graveyard_.clear();

    // 2) next point in time where something happens
    uint64_t tn = events_.empty() ? kNever : events_.top().t;
    for (auto& n : nodes_) {
      if (!n->up) continue;
      for (auto& t : n->tasks) {
        if (t->state == TaskState::Blocked && t->wake_us < tn) tn = t->wake_us;
      }
      Task* w = pick(*n);
      if (w && w->state == TaskState::Working) tn = std::min(tn, now_ + w->work_left_us);
      if (n->wdg_on) tn = std::min(tn, n->wdg_deadline_us);
    }
    const uint64_t target = std::min(tn, t_end);

    // 3) advance time: the CPU owner of each node consumes its work
    const uint64_t dt = target - now_;
    if (dt > 0) {
      for (auto& n : nodes_) {
        if (!n->up) continue;
        Task* w = pick(*n);
        if (w && w->state == TaskState::Working) {
          const uint64_t d = std::min(dt, w->work_left_us);
          w->work_left_us -= d;
          w->cpu_us += d;
        }
      }
      now_ = target;
    }
    if (tn > t_end) return;

    // 4) fire due timeouts, finished work, watchdogs and events
    for (auto& n : nodes_) {
      if (!n->up) continue;
      for (auto& t : n->tasks) {
        if (t->state == TaskState::Working && t->work_left_us == 0) {
          t->state = TaskState::Ready;  // keeps its ready_seq: it was running
        } else if (t->state == TaskState::Blocked && t->wake_us <= now_) {
          if (t->wait_q) {
            auto& w = t->wait_send ? t->wait_q->send_waiters : t->wait_q->recv_waiters;
            w.erase(std::remove(w.begin(), w.end(), t.get()), w.end());
            t->wait_q = nullptr;
          }
          make_ready(t.get());
        }
      }
      if (n->wdg_on && n->wdg_deadline_us <= now_) reset_node(*n, OSAL_RESET_WATCHDOG);
    }
    while (!events_.empty() && events_.top().t <= now_) {
      auto fn = events_.top().fn;
      events_.pop();
      fn();
    }
  }
}

}  // namespace sim
