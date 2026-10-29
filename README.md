# automotive-ecu-freertos

This is ECU firmware structured the way it would be written for an STM32 running FreeRTOS: tasks, queues, a supervising watchdog task, sensor acquisition, actuator control, and UDS diagnostics with DTCs and freeze frames. All of it goes through a small OS abstraction layer (`include/osal/osal.h`). On the host, that layer runs on a deterministic, virtual-time simulation kernel, so 20 ECU instances can run together on a simulated CAN FD bus. The network supports configurable bus load and error injection, and the experiments measure delivery rate, latency and watchdog recovery.

Everything reported here was produced by simulation on a PC. No microcontroller, CAN transceiver or FreeRTOS kernel was involved.

## What is in the repo

| Part | Files | Notes |
|---|---|---|
| OS abstraction | `include/osal/osal.h` | Tasks, `delay_until`, queues (task and ISR side), IWDG, reset cause, software reset. FreeRTOS semantics. |
| FreeRTOS backend | `port/freertos/osal_freertos.c` | Maps to `xTaskCreate`, `vTaskDelayUntil`, `xQueueSend/Receive(FromISR)`, `HAL_IWDG_*` and RCC reset flags. **Not built here** (no FreeRTOS or ARM toolchain in this environment). |
| Host backend | `src/osal/osal_sim.cpp`, `src/sim/kernel.cpp` | Virtual-time kernel. Each task is a POSIX `ucontext` coroutine. Each simulated MCU has a fixed-priority preemptive scheduler. Code runs in zero time except where it declares CPU work (`osal_cpu_work_us`), and a higher-priority task becoming ready preempts the remaining work of a lower one. Also models the hardware watchdog, reset and boot. Runs are fully deterministic for a given seed. |
| CAN FD model | `src/canfd/frame.cpp`, `src/canfd/bus.cpp` | Bit-level frame image (11-bit ID, FDF/BRS/ESI, DLC up to 64 bytes, stuff-count field, CRC-17/CRC-21). Exact dynamic stuff-bit counting and two-rate timing (500 kbit/s nominal, 2 Mbit/s data). Arbitration by lowest ID, controller TX buffers and acceptance filters. ISO 11898-1 fault confinement: TEC/REC, error passive with suspend transmission, bus-off and auto-recovery after 128 x 11 bits. Error frames and automatic retransmission. |
| ECU firmware | `src/ecu/ecu_node.cpp` | Seven tasks: watchdog (prio 6), CAN RX (5), CAN TX (5), sensor (4), control (3), actuator (3), diagnostics (2). Details below. |
| Diagnostics | `src/ecu/dtc.cpp`, `src/ecu/uds.cpp` | DTC manager with counter debouncing and ISO 14229 status bits (testFailed, testFailedThisOperationCycle, pending, confirmed). One freeze frame per DTC, kept in NVM across resets. UDS services 0x10, 0x11, 0x14, 0x19 (sub-functions 01/02/04), 0x22 and 0x3E, with negative responses. |
| Sensors / actuator | `src/ecu/sensors.cpp` | Three synthetic analogue channels with injectable faults (open circuit, stuck, spikes). Range, rate and stuck plausibility checks. Rate-limited actuator that moves to a safe state when commands time out. |
| Network + harness | `src/ecu/network.cpp`, `src/ecu/metrics.cpp` | 20 ECUs plus a background-traffic source and a UDS tester. Measurement probes record every produced message instance and every application-level reception. |
| Experiments | `tools/network_sim.cpp` | Writes `results/*.csv` and `results/summary_all.md`. |
| Tests | `tests/` (7 executables, 93 test cases) | Kernel scheduling semantics, CAN FD encoding/CRC/timing, bus arbitration and fault confinement, DTC/UDS, sensors/actuator/messages, firmware watchdog/diagnostics, and whole-network behaviour. |

### Firmware tasks

- **watchdog** (20 ms): each task records a heartbeat, and each heartbeat has its own deadline. The IWDG (100 ms) is refreshed only while every heartbeat is fresh. Three modes can be configured:
  - `supervised` stops refreshing and lets the IWDG reset the MCU.
  - `supervised_fast` triggers a software reset as soon as the fault is detected.
  - `unconditional` refreshes blindly. It is included as the common anti-pattern.
- **can_rx**: drains the RX queue filled by the CAN interrupt. It checks the application CRC-16 and sequence number, then hands each frame to sensor fusion, the actuator queue or the diagnostic queue.
- **can_tx**: moves frames from the TX queue into the three controller TX buffers. A frame is dropped once its deadline (one period) has passed. If all buffers are busy, the task can cancel a pending lower-priority frame (see experiment 3b).
- **sensor** (10/20/50/100 ms, depending on the node): samples the channels, runs the plausibility checks, reports DTCs, and broadcasts a 64-byte CAN FD frame.
- **control** (50 ms): sends a 16-byte command to the next ECU.
- **actuator** (10 ms): applies commands through a rate limiter. If no command arrives for 150 ms, it ramps to the safe value and sets the CommandTimeout DTC.
- **diag**: runs the UDS server. Responses go out on 0x740 + node. `0x11 01` performs a real software reset.

### Network

- 20 ECUs, numbered i = 0..19.
- Sensor periods cycle through 10, 20, 50 and 100 ms.
- ECU i consumes the sensor frames of ECUs i-1 and i-5 and the commands of ECU i-1. That gives 40 sensor and 20 command (instance, consumer) streams.
- Identifiers are deadline-monotonic by default: commands 0x080+i, sensors 0x100-0x113 ordered by period, diagnostics 0x700/0x740+i.
- Optional background traffic comes from 8 streams of 64-byte frames (IDs 0x140-0x147) with ±10% period jitter.
- A UDS tester polls one ECU every 100 ms.
- ECUs power up at random offsets of 0-20 ms, so their periodic tasks are not phase-aligned.

## Build, test, run

```bash
cmake -S . -B build -G Ninja && cmake --build build
cd build && ctest                                   # 7/7 executables, 93 test cases
./build/network_sim all --sim-ms 30000 --out results  # about 2 min
```

I built and ran everything with g++ 13.3 / CMake 3.28 in a Linux container (`docker run --rm -v "$PWD":/work -w /work devtools:latest ...`).

## Error model

Delivery and latency are measured at the **application layer**. A message instance counts as delivered only if the consumer's `can_rx` task processes it within one period of the time the producer's task created it. Latency is producer task to consumer task, so it includes the software queues, TX-buffer waits, arbitration, transmission, the RX interrupt and RX task scheduling.

Injected errors:

1. **Random bit errors** with a configurable BER, applied per bit on the wire (stuffed length, including ACK/EOF/IFS). For a hit frame, the first error position is uniform and further flips follow the same BER.
   - A flip in the arbitration or control field, or in the trailing delimiters, counts as an immediate bit, stuff or form error.
   - A flip in the data or CRC field is actually applied to the frame image. The receiver then recomputes CRC-17/21, and the frame is rejected only if the CRC fails. Undetected corruptions are counted (none occurred in any run).
   - A detected error produces an error frame (error flag 6 bits, delimiter 8 bits, intermission 3 bits at the nominal rate). TEC goes up by 8 for the transmitter and REC by 1 for the receivers, and the frame is retransmitted automatically.
2. **EMI windows**: time intervals with a raised BER.
3. **Defective transceiver**: one controller's own frames are hit with a 5% BER. This exercises error-passive operation, bus-off and auto-recovery.
4. **Bus load**: background frames at lower priority than all ECU traffic. With arbitrary ID assignment they are interleaved with it instead.
5. **Software faults**: a task enters a runaway loop (it consumes CPU and never blocks again) or deadlocks (it blocks forever on a queue). The fault is transient: a reset clears it.

Not modelled: propagation delay and bit-timing and synchronisation effects, ACK errors from a missing receiver, overload frames, transceiver wake-up, ISO-TP multi-frame transport, and real MCU execution times (the `osal_cpu_work_us` values are estimates, 10-300 us per job).

## Measured results (simulated)

All results come from `./build/network_sim all --sim-ms 30000 --out results`, with 30 s simulated per data point, the first second discarded, and seeds fixed in `tools/network_sim.cpp`. The full tables are in `results/summary_all.md`.

### Delivery vs bus load (BER 1e-5, deadline-monotonic IDs)

| Background | Bus utilisation | Delivered in deadline | p50 / p99 / max latency (ms) | Error frames |
|---:|---:|---:|---|---:|
| 0 % | 37.6 % | 100.000 % | 0.45 / 2.01 / 2.69 | 182 |
| 20 % | 57.5 % | 100.000 % | 0.52 / 2.16 / 2.65 | 305 |
| 40 % | 77.3 % | 100.000 % | 0.65 / 2.20 / 3.38 | 408 |
| 60 % | 95.1 % | 100.000 % | 0.69 / 2.18 / 3.43 | 492 |
| 70 % | 99.7 % | 100.000 % | 0.70 / 2.19 / 3.52 | 516 |

The 20 ECUs on their own load the bus to about 37%. With deadline-monotonic identifiers, all lower-priority background traffic can only delay an ECU frame by one non-preemptible frame time (~0.36 ms). For that reason every instance met its deadline even at 99.7% utilisation. The background frames and the diagnostic responses absorbed the overload instead.

### Delivery vs bit error rate (20% background)

| BER | Delivered in deadline | p99 / max latency (ms) | Error frames (share of attempts) | Bus-off events |
|---:|---:|---|---:|---:|
| 1e-6 | 100.000 % | 1.90 / 2.37 | 34 (0.06 %) | 0 |
| 1e-5 | 100.000 % | 1.96 / 3.21 | 330 (0.58 %) | 0 |
| 1e-4 | 100.000 % | 2.04 / 4.29 | 3084 (5.15 %) | 0 |
| 3e-4 | 99.882 % | 2.71 / 9.33 | 9473 (14.3 %) | 35 |
| 1e-3 | 97.842 % | 12.34 / 27.79 | 36006 (40.7 %) | 618 |

**On the 99.8% target:** with deadline-monotonic IDs, this error model only fell below 99.8% at a BER of about 1e-3. Losses start around 3e-4, when transmitters begin reaching bus-off; at 3e-4 delivery is still 99.88%. Up to a BER of 1e-4, automatic retransmission hides the errors; they show up only as extra latency.

### Identifier assignment (mean of 3 seeds)

| Background | Assignment | Bus utilisation | Delivered in deadline | 10 ms messages | 10 ms p99 latency |
|---:|---|---:|---:|---:|---:|
| 50 % | deadline-monotonic | 86.3 % | 100.000 % | 100.000 % | 1.62 ms |
| 50 % | arbitrary | 86.7 % | 98.643 % | 97.079 % | 16.95 ms |
| 70 % | deadline-monotonic | 99.2 % | 100.000 % | 100.000 % | 1.64 ms |
| 70 % | arbitrary | 99.7 % | 73.576 % | 65.745 % | 58.17 ms |

The "arbitrary" baseline draws sensor IDs at random from a range that overlaps the background traffic. This is the comparison where the 99.8% figure is actually at risk. It falls below 99.8% somewhere between 30% background (99.998%) and 50% background (98.6%).

### TX buffer policy (seed 11)

During development I found a priority inversion: at 70% background, diagnostic responses (low-priority IDs) sat in the controller's TX buffers and blocked an ECU's own sensor frames. With the FIFO `wait` policy, delivery dropped to 95.50% (1984 stale drops). Letting the TX task cancel a pending lower-priority frame restored 100.000%. At 80% background the tester's requests never won arbitration, so no responses were generated and both policies reached 100%.

### Watchdog recovery (3 seeds x 12 injected faults, 60 s each)

| Watchdog mode | Recovered | Detection mean / max | Recovery mean / p95 / max | Network delivery in deadline |
|---|---:|---|---|---:|
| off | 0 / 36 | - | - | 94.156 % |
| unconditional kick | 0 / 36 | - | - | 94.156 % |
| supervised (stop kicking, IWDG 100 ms) | 36 / 36 | 236 / 374 ms | 304 / 436 / 445 ms | 99.936 % |
| supervised + immediate reset | 36 / 36 | 154 / 294 ms | 222 / 356 / 365 ms | 99.955 % |

Recovery here means the time from the fault until the ECU's first sensor frame is delivered after its reset; it includes 20 ms of boot time. Detection time is dominated by the per-task heartbeat deadlines: 100 ms for event tasks, and 3x the period for periodic tasks, so up to 320 ms for a 100 ms task. An unconditionally refreshed watchdog never fired, because the refreshing task has the highest priority and keeps running while lower-priority tasks are hung. There were no resets on ECUs without an injected fault.

### Fault confinement

ECU 3 was given a defective transceiver for 10 s. It went bus-off 202 times (each time auto-recovering and failing again) and produced 6464 error frames. The other 19 ECUs still delivered 100.000% in deadline, with their p99 latency rising from 2.51 to 3.92 ms.

## Limitations

- These are simulation results. The kernel reproduces FreeRTOS scheduling semantics (fixed priority, preemption, queues with timeouts, `vTaskDelayUntil`), but not tick-interrupt granularity, ISR latency, critical sections, or the execution time of real code.
- The CRC is computed over the unstuffed bit sequence, whereas ISO 11898-1:2015 includes the dynamic stuff bits. The error model is a simplified per-bit BER rather than a physical-layer model.
- The FreeRTOS port, the STM32 HAL calls in it and the target build were not compiled or run.
- Message timestamps inside the payload exist only for measurement; a real system would need synchronised time for this.

## License

MIT, see `LICENSE`.
