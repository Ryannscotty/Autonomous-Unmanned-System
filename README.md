# Bare-metal STM32H7 Flight Controller

A from-scratch flight controller firmware stack for the STM32H743, built without an RTOS — a cooperative SysTick scheduler with hardware-backed fault detection, 9-DOF sensor fusion, and a graceful-degradation flight mode state machine. Designed as the on-board half of an SDR-linked drone command and telemetry system.

> **Project status: core firmware infrastructure complete, drivers in progress.** See [Implementation status](#implementation-status) below for exactly what runs today versus what's stubbed.

---

## Why this exists

This project was built to mirror the firmware architecture used in production UAS and avionics systems — the kind of concept-to-flight ownership that companies like Anduril, Shield AI, and Joby Aviation hire embedded engineers to do. It deliberately avoids reaching for off-the-shelf flight stacks (PX4, ArduPilot, Betaflight) in favor of writing the scheduler, sensor fusion, and control architecture from first principles, because that is where the actual engineering judgment lives.

The companion ground station (HTML/JS GCS with a counter-UAS RF detection panel and live mission planning) and the FPGA/SDR telemetry link this firmware is designed to pair with live in separate repos — see [Related work](#related-work).

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  STM32H743 — bare-metal, no RTOS                                 │
│                                                                   │
│  SysTick ISR (1 kHz)  ──sets pending flags──▶  superloop          │
│         │                                          │              │
│         │                                   watchdog_task_start() │
│         │                                          ▼              │
│         │                                    task.fn() runs       │
│         │                                          │              │
│         ▼                                   watchdog_task_end()   │
│  IWDG (hardware) ◀── kicked once per loop pass, never inside a    │
│  TIM7 deadman timer ── fires independently if a task hangs        │
│                                                                   │
│  ┌───────────────┐   ┌────────────────┐   ┌────────────────────┐ │
│  │ 1 kHz tier     │   │ 10 Hz tier     │   │ Flight mode FSM     │ │
│  │ IMU → AHRS     │──▶│ GPS/baro/batt  │──▶│ STABILIZE→ALT_HOLD  │ │
│  │ → rate PID     │   │ → position PID │   │ →LOITER→RTL→FAILSAFE│ │
│  └───────────────┘   └────────────────┘   └────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

Sensor fusion runs as two cooperating filters: a 9-DOF Madgwick quaternion filter (gyro + accelerometer + magnetometer) for attitude at 1 kHz, feeding a cascaded rate → attitude → position PID stack. See [`docs/sensor-fusion.md`](docs/sensor-fusion.md) *(coming soon)* for the full filter-per-sensor breakdown.

---

## Implementation status

| Module | Status | Notes |
|---|---|---|
| `scheduler.c` / `scheduler.h` | ✅ Complete | Cooperative SysTick dispatcher, per-task execution profiling via DWT cycle counter, configurable task table |
| `watchdog.c` | ✅ Complete | Two-layer fault detection: IWDG hardware reset + TIM7 deadman timer that identifies *which* task hung, surviving the reset via RTC backup register |
| `ahrs.c` / `ahrs.h` | ✅ Complete | 9-DOF Madgwick filter — gyro + accel + magnetometer, online gyro bias estimation, hard-iron mag calibration routine |
| `mag.c` / `mag.h` | ✅ Complete | IST8310 magnetometer SPI driver |
| `flight_mode.c` / `flight_mode.h` | ✅ Complete | Flight mode FSM — STABILIZE, ALT_HOLD, LOITER, AUTO, RTL, LAND, FAILSAFE, with automatic graceful degradation when a mode's required sensors become unavailable |
| `task_ahrs.c` | ✅ Complete | Scheduler glue connecting IMU/mag drivers to the AHRS update task |
| `main.c` | ✅ Complete | Boot sequence, clock config, peripheral init, watchdog diagnostic readback |
| `imu.c` | 🚧 Not yet written | ICM-42688-P SPI DMA driver — **verify your specific breakout exposes SPI**, not all do |
| `pid.c` | 🚧 Not yet written | Cascaded rate/attitude/position PID, motor mix matrix |
| `baro.c` | 🚧 Not yet written | BMP388 driver + altitude IIR filter |
| `gps.c` | 🚧 Not yet written | u-blox M10 UBX parser, NED conversion |
| `battery.c` | 🚧 Not yet written | ADC voltage/current monitoring, mAh integration |
| `mavlink.c` | 🚧 Not yet written | MAVLink 2.0 telemetry TX/RX, command parsing |
| `dshot.c` | 🚧 Not yet written | DShot600 ESC output via TIM1 DMA |
| `mission.c` | 🚧 Not yet written | Waypoint sequencing for AUTO mode |
| `param.c` | 🚧 Not yet written | Flash-backed parameter store |
| `failsafe.c` | 🚧 Not yet written | Pre-arm checks, link-loss detection, arm/disarm gating |
| `blackbox.c` | 🚧 Not yet written | SD card flight data logging |

`task_stubs.c` provides a harmless no-op for every task callback not yet implemented, so the scheduler core can be flashed and verified on real hardware today, independent of the rest of the stack.

---

## Design decisions worth knowing about

**Why a bare-metal scheduler instead of FreeRTOS.** A cooperative SysTick dispatcher gives full control over execution order and eliminates an entire class of priority-inversion and context-switch-jitter bugs that make PID tuning miserable to debug. Every task runs to completion, in a fixed table order, with execution time measured via the DWT cycle counter — you can put an oscilloscope on a single GPIO and see exactly what the CPU is doing at any microsecond. An RTOS earns its complexity when you have many independent, asynchronous workloads; a flight controller's control loops are not that — they are a small number of strictly periodic tasks with hard deadlines, which is exactly what a cooperative scheduler is built for.

**Why the watchdog has two layers, not one.** Every statistic the scheduler tracks (`max_exec_us`, `overruns`) is written *after* a task returns — a task stuck forever in a blocking call or an infinite loop never reaches that code, and the failure is invisible. The IWDG (hardware, LSI-clocked, independent of the CPU core) guarantees recovery no matter what. A second timer, TIM7, runs in one-pulse mode armed fresh before every task call; if it fires, it means that *specific* task exceeded its time budget, and the index is written to an RTC backup register that survives the reset. The result: the next boot can report "task `gps_parse` hung" instead of just resetting with zero diagnostic information.

**Why the flight mode FSM degrades instead of refusing.** If LOITER is requested but GPS lock drops mid-flight, the FSM doesn't get stuck or fault — it walks down a degradation ladder (`LOITER → ALT_HOLD → STABILIZE → FAILSAFE`) until it finds the most capable mode the *currently healthy* sensors can actually support, and sends a status message explaining why. This check runs every tick against the *active* mode, not just at the moment of a mode-change request, because sensors fail mid-flight, not just at the moment a button is pressed.

**Why the rate PID will read raw gyro, not the AHRS output.** (Implemented in the not-yet-written `pid.c`, documented here because it shapes the rest of the design.) The inner control loop needs the lowest possible latency; routing it through a filter — even a fast one — adds phase lag that directly degrades stability margin. Only the D-term gets a light PT1 low-pass, because differentiating raw gyro noise produces spikes a filter has no business being absent for.

---

## Build

### Prerequisites

```bash
# macOS
brew install arm-none-eabi-gcc cmake openocd

# Ubuntu / Debian
sudo apt install gcc-arm-none-eabi cmake openocd
```

This repo does not vendor the STM32H7 HAL/CMSIS sources. Generate a project in STM32CubeIDE for an STM32H743ZI target, copy the `Drivers/` folder it produces into the repo root, then build with the CMake config provided here.

### Build & flash

```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi.cmake
make -j$(nproc)
make flash    # requires ST-Link V3 + OpenOCD
```

Full build, debug, and timing-verification instructions are in [`README.md` inside `fc_firmware/`](fc_firmware/README.md) — including how to read scheduler statistics, interpret a watchdog-caused reset, and verify loop timing on an oscilloscope before connecting motors.

---

## Hardware target

| Component | Part |
|---|---|
| MCU | STM32H743ZIT6 (480 MHz Cortex-M7) |
| IMU | ICM-42688-P (gyro + accel, SPI) |
| Magnetometer | IST8310 (SPI) |
| Barometer | BMP388 *(driver pending)* |
| GPS | u-blox M10 *(driver pending)* |
| ESCs | BLHeli_32, DShot600 *(driver pending)* |

---

## Related work

- **Ground control station** — HTML/JS GCS with live MAVLink telemetry, mission planning with real tile-map waypoints, and a counter-UAS RF detection + QRF response panel. *(link to repo)*
- **SDR telemetry link** — GNU Radio flowgraph for the ADALM-PLUTO ground station: BPSK modulation, AES-128, Reed-Solomon FEC, bridging to MAVLink over a local TCP socket. *(link to repo)*

---

## Safety

This is firmware for a system that can spin propellers at lethal speed. Before connecting motors:

- Remove all propellers for every bench test until rate and attitude loops are verified stable.
- Confirm `scheduler_get_stats().missed_deadlines == 0` after a 60-second bench run.
- Verify the watchdog actually resets the MCU within the configured timeout by deliberately hanging a test task.
- Confirm failsafe triggers correctly with the radio link physically disconnected before any flight test.

## License

MIT — see [LICENSE](LICENSE).
