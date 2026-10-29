#include <string>
#include <vector>

#include "osal/osal.h"
#include "sim/kernel.hpp"
#include "testfw.hpp"

using sim::Kernel;

namespace {
std::vector<std::string> g_log;
std::vector<uint64_t> g_times;

Kernel& fresh() {
  Kernel& k = Kernel::get();
  k.clear();
  g_log.clear();
  g_times.clear();
  return k;
}

void periodic_task(void*) {
  uint32_t wake = osal_tick_ms();
  for (int i = 0; i < 5; ++i) {
    osal_delay_until(&wake, 10);
    g_times.push_back(osal_time_us());
  }
}
}  // namespace

TEST(Kernel, DelayUntilIsPeriodic) {
  Kernel& k = fresh();
  k.add_node("n", []() { osal_task_create("p", periodic_task, nullptr, 0, 1); });
  k.start();
  k.run_until(100000);
  REQUIRE(g_times.size() == 5);
  for (size_t i = 0; i < 5; ++i) CHECK_EQ(g_times[i], (i + 1) * 10000);
}

TEST(Kernel, DelayUntilNoDriftWithWork) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    osal_task_create("p", [](void*) {
      uint32_t wake = osal_tick_ms();
      for (int i = 0; i < 4; ++i) {
        osal_delay_until(&wake, 10);
        g_times.push_back(osal_time_us());
        osal_cpu_work_us(3000);
      }
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(100000);
  REQUIRE(g_times.size() == 4);
  CHECK_EQ(g_times[3], 40000u);
}

TEST(Kernel, HigherPriorityRunsFirst) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    osal_task_create("lo", [](void*) { g_log.push_back("lo"); }, nullptr, 0, 1);
    osal_task_create("hi", [](void*) { g_log.push_back("hi"); }, nullptr, 0, 5);
    osal_task_create("mid", [](void*) { g_log.push_back("mid"); }, nullptr, 0, 3);
  });
  k.start();
  k.run_until(1000);
  REQUIRE(g_log.size() == 3);
  CHECK_EQ(g_log[0], std::string("hi"));
  CHECK_EQ(g_log[1], std::string("mid"));
  CHECK_EQ(g_log[2], std::string("lo"));
}

TEST(Kernel, EqualPriorityFifo) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    osal_task_create("a", [](void*) { g_log.push_back("a"); }, nullptr, 0, 2);
    osal_task_create("b", [](void*) { g_log.push_back("b"); }, nullptr, 0, 2);
  });
  k.start();
  k.run_until(1000);
  REQUIRE(g_log.size() == 2);
  CHECK_EQ(g_log[0], std::string("a"));
}

TEST(Kernel, WorkIsPreemptedByHigherPriority) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    osal_task_create("lo", [](void*) {
      osal_cpu_work_us(10000);
      g_log.push_back("lo_done");
      g_times.push_back(osal_time_us());
    }, nullptr, 0, 1);
    osal_task_create("hi", [](void*) {
      osal_delay_ms(2);
      g_times.push_back(osal_time_us());
      osal_cpu_work_us(1000);
      g_log.push_back("hi_done");
    }, nullptr, 0, 5);
  });
  k.start();
  k.run_until(50000);
  REQUIRE(g_times.size() == 2);
  CHECK_EQ(g_times[0], 2000u);   // high-priority task started on time
  CHECK_EQ(g_times[1], 11000u);  // low task finished late by the preemption
  CHECK_EQ(g_log[0], std::string("hi_done"));
}

TEST(Kernel, NodesHaveIndependentCpus) {
  Kernel& k = fresh();
  for (int i = 0; i < 3; ++i) {
    k.add_node("n", []() {
      osal_task_create("w", [](void*) {
        osal_cpu_work_us(5000);
        g_times.push_back(osal_time_us());
      }, nullptr, 0, 1);
    });
  }
  k.start();
  k.run_until(20000);
  REQUIRE(g_times.size() == 3);
  for (auto t : g_times) CHECK_EQ(t, 5000u);
}

namespace {
osal_queue_t g_q;
}

TEST(Kernel, QueueReceiveBlocksUntilSend) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    g_q = osal_queue_create(4, sizeof(int));
    osal_task_create("rx", [](void*) {
      int v = 0;
      const bool ok = osal_queue_receive(g_q, &v, OSAL_WAIT_FOREVER);
      g_times.push_back(osal_time_us());
      g_log.push_back(ok ? std::to_string(v) : "fail");
    }, nullptr, 0, 3);
    osal_task_create("tx", [](void*) {
      osal_delay_ms(7);
      int v = 42;
      osal_queue_send(g_q, &v, 0);
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(20000);
  REQUIRE(g_log.size() == 1);
  CHECK_EQ(g_log[0], std::string("42"));
  CHECK_EQ(g_times[0], 7000u);
}

TEST(Kernel, QueueReceiveTimeout) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    g_q = osal_queue_create(1, 1);
    osal_task_create("rx", [](void*) {
      uint8_t v;
      g_log.push_back(osal_queue_receive(g_q, &v, 25) ? "got" : "timeout");
      g_times.push_back(osal_time_us());
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(100000);
  REQUIRE(g_log.size() == 1);
  CHECK_EQ(g_log[0], std::string("timeout"));
  CHECK_EQ(g_times[0], 25000u);
}

TEST(Kernel, QueueSendBlocksWhenFull) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    g_q = osal_queue_create(2, sizeof(int));
    osal_task_create("tx", [](void*) {
      for (int i = 0; i < 4; ++i) {
        osal_queue_send(g_q, &i, OSAL_WAIT_FOREVER);
        g_times.push_back(osal_time_us());
      }
    }, nullptr, 0, 3);
    osal_task_create("rx", [](void*) {
      for (int i = 0; i < 4; ++i) {
        osal_delay_ms(10);
        int v;
        osal_queue_receive(g_q, &v, 0);
        g_log.push_back(std::to_string(v));
      }
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(100000);
  REQUIRE(g_times.size() == 4);
  CHECK_EQ(g_times[1], 0u);
  CHECK_EQ(g_times[2], 10000u);  // third send waited for the first receive
  REQUIRE(g_log.size() == 4);
  CHECK_EQ(g_log[3], std::string("3"));
}

TEST(Kernel, IsrSendOverflowCounted) {
  Kernel& k = fresh();
  sim::Queue* q = nullptr;
  k.add_node("n", [&q]() { q = reinterpret_cast<sim::Queue*>(osal_queue_create(2, 1)); });
  k.start();
  uint8_t b = 1;
  CHECK(k.queue_send_isr(q, &b));
  CHECK(k.queue_send_isr(q, &b));
  CHECK(!k.queue_send_isr(q, &b));
  CHECK_EQ(q->isr_overflows, 1u);
}

TEST(Kernel, IsrWakesWaitingTask) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    g_q = osal_queue_create(4, sizeof(int));
    osal_task_create("rx", [](void*) {
      int v;
      if (osal_queue_receive(g_q, &v, OSAL_WAIT_FOREVER)) g_times.push_back(osal_time_us());
    }, nullptr, 0, 1);
  });
  k.start();
  k.at(3500, []() {
    int v = 1;
    osal_queue_send_from_isr(g_q, &v);
  });
  k.run_until(10000);
  REQUIRE(g_times.size() == 1);
  CHECK_EQ(g_times[0], 3500u);
}

TEST(Kernel, CalloutsInTimeOrder) {
  Kernel& k = fresh();
  k.at(300, []() { g_log.push_back("c"); });
  k.at(100, []() { g_log.push_back("a"); });
  k.at(200, []() { g_log.push_back("b"); });
  k.at(200, []() { g_log.push_back("b2"); });
  k.run_until(1000);
  REQUIRE(g_log.size() == 4);
  CHECK_EQ(g_log[0], std::string("a"));
  CHECK_EQ(g_log[1], std::string("b"));
  CHECK_EQ(g_log[2], std::string("b2"));
  CHECK_EQ(g_log[3], std::string("c"));
}

TEST(Kernel, WatchdogResetsAndReboots) {
  Kernel& k = fresh();
  sim::Node& n = k.add_node("n", []() {
    g_log.push_back(osal_reset_cause() == OSAL_RESET_WATCHDOG ? "boot_wdg" : "boot_por");
    osal_hw_watchdog_start(50);
    osal_task_create("kick", [](void*) {
      for (int i = 0; i < 3; ++i) {
        osal_delay_ms(20);
        osal_hw_watchdog_kick();
      }
      // stop kicking
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(200000);
  CHECK(n.resets >= 1u);
  REQUIRE(!n.reset_times_us.empty());
  CHECK_EQ(n.reset_times_us[0], 110000u);  // last kick at 60 ms + 50 ms
  REQUIRE(g_log.size() >= 2);
  CHECK_EQ(g_log[0], std::string("boot_por"));
  CHECK_EQ(g_log[1], std::string("boot_wdg"));
}

TEST(Kernel, KickedWatchdogNeverFires) {
  Kernel& k = fresh();
  sim::Node& n = k.add_node("n", []() {
    osal_hw_watchdog_start(50);
    osal_task_create("kick", [](void*) {
      uint32_t w = osal_tick_ms();
      for (;;) {
        osal_delay_until(&w, 20);
        osal_hw_watchdog_kick();
      }
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(1000000);
  CHECK_EQ(n.resets, 0u);
}

TEST(Kernel, RunawayTaskStarvesLowerButNotHigher) {
  Kernel& k = fresh();
  k.add_node("n", []() {
    osal_task_create("runaway", [](void*) {
      for (;;) osal_cpu_work_us(1000);
    }, nullptr, 0, 3);
    osal_task_create("low", [](void*) {
      for (;;) {
        osal_delay_ms(10);
        g_log.push_back("low");
      }
    }, nullptr, 0, 1);
    osal_task_create("high", [](void*) {
      for (;;) {
        osal_delay_ms(10);
        g_times.push_back(osal_time_us());
      }
    }, nullptr, 0, 5);
  });
  k.start();
  k.run_until(100000);
  CHECK(g_log.empty());
  CHECK(g_times.size() >= 9u);
}

TEST(Kernel, SoftwareResetFromTask) {
  Kernel& k = fresh();
  sim::Node& n = k.add_node("n", []() {
    g_log.push_back("boot");
    if (osal_reset_cause() == OSAL_RESET_POWER_ON) {
      osal_task_create("r", [](void*) {
        osal_delay_ms(5);
        osal_system_reset();
        g_log.push_back("unreachable");
      }, nullptr, 0, 1);
    }
  });
  k.start();
  k.run_until(100000);
  CHECK_EQ(n.resets, 1u);
  CHECK(n.reset_cause == OSAL_RESET_SOFTWARE);
  REQUIRE(g_log.size() == 2);
  CHECK_EQ(g_log[1], std::string("boot"));
}

TEST(Kernel, CpuTimeAccounted) {
  Kernel& k = fresh();
  sim::Node& n = k.add_node("n", []() {
    osal_task_create("w", [](void*) {
      for (int i = 0; i < 10; ++i) {
        osal_delay_ms(10);
        osal_cpu_work_us(500);
      }
    }, nullptr, 0, 1);
  });
  k.start();
  k.run_until(200000);
  REQUIRE(!n.tasks.empty());
  CHECK_EQ(n.tasks[0]->cpu_us, 5000u);
}

TEST_MAIN()
