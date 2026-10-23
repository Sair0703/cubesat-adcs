# CubeSat Attitude Control Flight Software

Flight software that keeps a small satellite pointed where it's told, written in
fixed-point C++ for a Cortex-M3 with no floating-point unit. It runs as a task on
my [from-scratch RTOS](https://github.com/Sair0703/cortex-m-rtos), and the same
code compiles to WebAssembly for a live, interactive demo.

**[Try the live demo](https://sair0703.github.io/cubesat-adcs/)**: command slews, knock the satellite off course, or break its sun sensor, and watch the firmware recover.

## What it does

| Capability | How |
|---|---|
| **Detumble** | After release the satellite is spinning. Rate damping through the reaction wheel brakes it using the gyro alone. |
| **Attitude estimation** | A complementary filter fuses the gyro (accurate short-term) with the sun sensor (accurate long-term). |
| **Pointing and slews** | PID control with derivative-on-measurement, an anti-windup integrator, a torque limit, and a slew-rate limit for smooth large maneuvers. |
| **Fault detection** | Detects a **frozen** sun sensor (a real sensor is never noise-free) and a **lying** one (persistent disagreement with the gyro). Either fault drops the spacecraft into SAFE mode, which stops trusting the sun sensor until ground clears the fault. |
| **Momentum management** | Corrections accumulate spin in the reaction wheel. Above 3,000 rpm, the firmware commands the magnetorquers to bleed momentum off while holding pointing. |
| **Fixed-point math** | All control runs in saturating Q16.16 arithmetic, so there's no FPU dependency and overflow clips instead of wrapping. |

## Verified results

The scripted mission (`sim/mission.cpp`) runs identically in host tests and on the emulated Cortex-M3:

| Metric | Result |
|---|---|
| Detumble from 8 deg/s | 3.5 s |
| 90 deg slew, settled within 1 deg | 16.5 s |
| Overshoot | 0.14 deg |
| Frozen sun sensor detected and SAFE mode entered | 0.52 s |
| Back on target after the fault is cleared | 1.0 s |
| Final pointing error | 0.04 deg |

## Architecture

```
core/       flight software: fixed-point math, estimator, controller, modes, fault detection
sim/        spacecraft physics (rigid body, reaction wheel, magnetorquers, noisy sensors) and the test mission
targets/
  qemu/     Cortex-M3 firmware: runs the controller as a 50 Hz task on my RTOS, telemetry over UART
  wasm/     WebAssembly exports for the browser demo
tests/      host unit + closed-loop tests, and a QEMU test that checks the mission results
web/        demo page; the build embeds the .wasm so docs/index.html is self-contained
```

The flight software in `core/` has no hardware or OS dependencies. It takes sensor readings in and returns actuator commands, which is why the same code runs on the microcontroller, on a PC for testing, and in the browser.

## Build and test

Requires `gcc-arm-none-eabi`, `qemu-system-arm`, `clang`, and `lld`.

```
make test        # host unit tests + closed-loop mission (g++)
make qemu-test   # flight software on an emulated Cortex-M3, mission results checked
make run         # watch the telemetry stream from QEMU
make demo        # build the WebAssembly live demo into docs/index.html
```

GitHub Actions runs all three test stages on every push.

## Simplifications

Single-axis rotation; magnetorquer torque is modeled as the commanded value rather than computed from a magnetic field model; sensor noise is Gaussian. The control, estimation, and fault-handling structure is the same as on a three-axis system.
