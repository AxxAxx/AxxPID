# Changelog

All notable changes to AxxPID are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses
[semantic versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-09-28

First release. The control law is the one that has been running in the
[AxxSolder](https://github.com/AxelJohanssonSWE/AxxSolder) soldering station
firmware, extracted into a standalone library and generalised.

### Carried over from AxxSolder

- Parallel-form PID integrating and differentiating against the real elapsed
  time, so `ki` and `kd` are independent of the sample rate.
- Derivative on measurement, so a setpoint step produces no derivative kick.
- Conditional-integration anti-windup.
- An explicit clamp on the integral term, separate from the output limits.
- The integral engagement band (`PID_SetIminError`), which parks the
  integrator while the process is far below setpoint.
- Asymmetric integral gain (`PID_SetNegativeErrorIgainMult`), which drains the
  integrator faster once the process has overshot.
- Zeroing the integrator when the setpoint is exactly zero.
- Separate P, I and D contributions for telemetry and tuning graphs
  (`PID_GetPpart` / `Ipart` / `Dpart`).
- Manual and automatic modes with a bumpless transition.
- Sample-time gating against a millisecond tick.

### Added

- **Feed-forward**, as four summed sources: a constant bias, a
  setpoint-proportional gain, a setpoint-rate (velocity) gain, and a user
  callback. Applied inside the saturation calculation so the integrator cannot
  wind up behind it.
- **Back-calculation anti-windup** as an alternative to conditional
  integration, with a configurable tracking time constant.
- **Derivative low-pass filtering**, by the Astrom-Hagglund divisor `N` or by
  an explicit time constant. Off by default, which reproduces the AxxSolder
  behaviour exactly.
- **Two-degree-of-freedom setpoint weighting** (`b` and `c`), which subsumes
  proportional-on-measurement and derivative-on-error as special cases.
- **Output slew-rate limiting**, fed back into back-calculation.
- **An error deadband**, applied as a continuous shift rather than a step.
- **Bumpless retuning**: the integrator absorbs a mid-flight change to `kp` or
  `kd` so the output does not step.
- **`axxpid_reset_to()`**, to resume from a known actuator position.
- **`axxpid_set_dt_max()`**, to bound the integral step after a stalled loop.
- **Standard (ISA) form tunings** via `axxpid_set_tunings_standard()`.
- **A tuning module** (`axxpid_tune.h`): Ziegler-Nichols closed and open loop,
  Pessen, the two overshoot-limited variants, Tyreus-Luyben, Cohen-Coon and
  lambda/SIMC, plus a relay autotuner that measures the ultimate gain and
  period.
- **NaN and infinity rejection** on the setpoint, measurement and timestep: a
  bad sample holds the previous output instead of poisoning the integrator.
- **Validation on every entry point**, returning `axxpid_status_t`. A rejected
  call changes nothing.
- **`axxpid_set_integral()` and `axxpid_get_integral()`**, to save and restore
  the integrator across a power cycle, hand over between gain schedules, or
  clear just the integral without disturbing the derivative history.
- A test suite of 538 assertions across four suites, including closed-loop
  simulations against a first-order-plus-dead-time plant, run in both
  `float` and `double`.

### Changed from AxxSolder

- **Values instead of pointers.** `axxpid_update(pid, setpoint, measurement,
  dt)` returns the output, rather than reading and writing through stored
  `volatile float *`. This removes the aliasing and lifetime hazards and makes
  the controller testable.
- **No HAL dependency.** The old `PID_Compute` called `HAL_GetTick()` itself.
  The caller now supplies either the elapsed time (`axxpid_update`) or a
  millisecond timestamp (`axxpid_update_at`).
- **Reverse action negates the error, not the gains.** The old code flipped
  the sign of `Kp`, `Ki` and `Kd` in place and kept a second `Disp*` copy for
  the getters; `PID_SetControllerDirection` could double-flip if called twice.
  Gains are now stored once, positive, and the getters always return what was
  set.
- **The `-1` asymmetry threshold is a parameter**, not a hard-coded constant.
- **Integral limits default to unbounded.** `PID_SetILimits` rejected
  `min >= max`, so `PID_SetILimits(&TPID, 0, 0)` silently did nothing and left
  whatever the struct was initialised with. Equal limits are now accepted and
  pin the integrator.

### Fixed

Two defects found while porting, both present in the AxxSolder original:

- **Conditional integration used the sign of the output** as a proxy for which
  limit was being exceeded (`error * output > 0`). That is only equivalent
  when the output limits straddle zero. On an actuator that lives between, say,
  20% and 80%, the test would allow the integrator to run straight through the
  lower limit. It now tests which limit the candidate output exceeds and
  whether the step deepens that specific violation.
- **`NegativeErrorIgainBias` was stored but never read** by `PID_Compute`.
  Removed rather than carried forward as dead configuration.

And six more found by review of the new code before release, each with a
regression test:

- **A rejected `axxpid_init` left the controller uninitialised.** Since the
  struct is caller-allocated, that meant the next update ran on stack litter -
  including calling a garbage feed-forward function pointer. Both init
  entry points now install a safe default controller unconditionally.
- **Non-finite configuration slipped through.** A NaN compares false against
  every range test, so `kp = NaN` passed a `kp < 0` check and reached the
  integrator. Configuration is now checked for finiteness explicitly.
- **Bumpless transfer preloaded the integrator from the raw manual output**
  rather than from the output the actuator could actually be given. With an
  out-of-range manual value it looked bumpless for one sample, then pinned the
  actuator at its limit while the integrator unwound the difference.
- **Conditional anti-windup ignored the slew-rate limit.** A rate limit holds
  the output away from the control law's demand just as firmly as a hard
  limit, so the default configuration wound up through every rate-limited
  ramp. It now treats the slew window as part of what the actuator can reach.
- **Back-calculation had an unbounded `dt/Tt` gain.** Because `dt` is measured
  rather than assumed, one late sample could push the gain past 1 (ringing) or
  past 2 (divergence). The gain is now capped at 1.
- **`dt_max` turned a stalled loop into a derivative spike.** The cap kept the
  integral step sane but the difference terms were still divided by the capped
  `dt`, inventing a rate of change orders of magnitude too large. The
  derivative and velocity feed-forward now restart across a capped sample, as
  they do across a rejected one.

Three smaller corrections in the same pass: a rejected sample no longer
corrupts the next derivative, `axxpid_update_at` no longer drops the interval
a rejected sample covered, and the integral engagement band and zero-setpoint
reset are no longer immediately nudged off zero by back-calculation.

And three in the autotuner:

- **An infinite measurement hung the control loop.** The square root's range
  reduction divides by four until the value drops below four, which never
  happens for infinity. The relay rejected NaN but not infinity, so one bad
  sensor sample could lock up the loop. Non-finite input is now rejected at
  both the relay and the square root, and the amplitude term is computed as
  `a·√(1−(h/a)²)` rather than `√(a²−h²)`, which cannot overflow.
- **Sensor noise was accepted as a limit cycle.** A relay chattering on noise
  produced a valid-looking result in a fraction of a second — in one measured
  case `Ku = 1808`, which Ziegler–Nichols turns into `kp = 1085`. Cycles
  shorter than `AXXPID_RELAY_MIN_SAMPLES_PER_CYCLE` are now discarded, so a
  noise-driven autotune times out instead.
- **The "oscillation too small" guard could never fire.** It tested `a > h`,
  but the relay only switches when the error exceeds `h`, so `a ≥ h` always
  held. Meanwhile the estimate runs away as `1/√(a²−h²)` as `a` approaches
  `h`. The guard now requires `a ≥ 2h`.

Documentation corrected to match measurement rather than intent: hysteresis
biases `Tu` upward by roughly `10·(h/a)` per cent and is not "compensated for"
in any meaningful sense, and `output_bias` has to be near the true holding
output or the autotune skews or never starts.

[1.0.0]: https://github.com/axeljohansson/AxxPID/releases/tag/v1.0.0
