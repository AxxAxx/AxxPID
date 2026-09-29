# Changelog

All notable changes to AxxPID are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-09-29

First release.

### The controller

- Parallel-form PID that integrates and differentiates against the real
  elapsed time, so `ki` and `kd` are independent of the sample rate and a
  jittery loop reaches the same steady state as a metronomic one.
- Derivative taken from the measurement rather than the error, so a setpoint
  step produces no derivative kick, with an optional low-pass filter set
  either by the Astrom-Hagglund divisor `N` or by an explicit time constant.
- Two-degree-of-freedom setpoint weighting, which subsumes
  proportional-on-measurement and derivative-on-error as special cases.
- Feed-forward as a first-class term, in four summed forms: a constant bias, a
  setpoint-proportional gain, a setpoint-rate (velocity) gain, and a user
  callback. Applied inside the saturation calculation, so the integrator
  cannot wind up behind it.
- Two anti-windup strategies - conditional integration and back-calculation
  with a configurable tracking time - plus an explicit clamp on the integral
  itself.
- Manual and automatic modes with bumpless transfer in both directions,
  bumpless retuning, and `axxpid_reset_to()` for resuming from a known
  actuator position.
- Output slew-rate limiting, which both anti-windup strategies account for,
  and a continuous error deadband.
- Support for one-way actuators: an integral engagement band that parks the
  integrator while the process is far from setpoint, an asymmetric integral
  gain that drains faster than it fills once the process has overshot, and an
  optional reset when the setpoint is exactly zero.
- Separate P, I, D and feed-forward contributions for telemetry and tuning
  graphs, plus the live integral for saving and restoring across a power
  cycle.
- Two ways to drive the loop: `axxpid_update()` with an elapsed time you
  supply, or `axxpid_update_at()` rate-limited against any millisecond
  counter, wrap-safe across the 49.7-day rollover of a 32-bit tick.

### The tuning module

- Ziegler-Nichols closed and open loop, Pessen Integral Rule, the two
  overshoot-limited variants, Tyreus-Luyben, Cohen-Coon and lambda/SIMC.
- A relay autotuner that measures the ultimate gain and period, with
  hysteresis for noisy measurements, and guards that reject a limit cycle
  driven by sensor noise rather than by the process.
- Conversion between parallel (`kp`, `ki`, `kd`) and standard (`kp`, `Ti`,
  `Td`) forms.

### Robustness

- No dynamic allocation, no globals, no vendor HAL, and no standard library
  dependency - not even `<math.h>`. The only external symbol on a freestanding
  build is compiler-emitted `memcpy`.
- Non-finite setpoints, measurements, timesteps, configuration values and
  feed-forward hook results are all rejected rather than allowed to reach the
  integrator.
- Both initialisation entry points always leave a usable controller, even when
  they reject an argument, so a caller that ignores the return value never
  runs on uninitialised memory.
- Bounds that cannot be switched off: no single update may move the integral
  by more than ten times the output range, and the default integral limits are
  derived from that range rather than left unbounded.

### Verification

- 808 assertions across four suites, in both `float` and `double`. Expected
  values are derived from the difference equations or from the published
  tuning tables, never recorded from a previous run.
- Mutation tested: twenty deliberately broken copies of the library are built
  and run against the suite, and nineteen are caught.
- Compiles clean and freestanding at `-Wall -Wextra -Wpedantic -Wshadow
  -Wconversion -Wdouble-promotion -Werror` for Cortex-M0+, Cortex-M4F,
  AArch64, RISC-V rv32imc and rv64, AVR, MSP430, MIPS, PowerPC, WebAssembly,
  and x86/x86-64 on Linux, macOS and Windows with both GCC and Clang. Usable
  from C++, and valid C99, C11 and C17.

[1.0.0]: https://github.com/AxxAxx/AxxPID/releases/tag/v1.0.0
