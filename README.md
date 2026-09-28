# AxxPID

A PID controller for C that you can drop into any project and trust.

No dependencies — not even `<math.h>`; the one square root the autotuner needs
is in the box. No dynamic allocation, no globals, no vendor HAL. One `.c` file
and one header for the controller, one more pair if you want the autotuner.
Every controller lives in a struct you allocate yourself, so you can have as
many as you like, on the stack, in `.bss`, or inside another struct.

Compiled freestanding for a Cortex-M4F, the only external symbol either object
file references is `memcpy`, which the compiler emits for a struct copy and
every toolchain provides.

It is the control loop from the [AxxSolder](https://github.com/AxelJohanssonSWE/AxxSolder)
soldering station, pulled out and generalised. The unusual features — the
integral engagement band, the asymmetric integral gain, the explicit integral
clamp — are there because a soldering iron needed them, and they turn out to
be useful on anything with a one-way actuator.

```c
#include "axxpid/axxpid.h"

axxpid_t pid;

/*          kp    ki    kd    out_min  out_max */
axxpid_init(&pid, 8.0f, 2.0f, 0.5f,    0.0f, 100.0f);

for (;;) {
    float power = axxpid_update(&pid, setpoint, read_sensor(), 0.010f);
    drive_heater(power);
}
```

That is the whole integration. Everything below is optional.

---

## Contents

- [Why this one](#why-this-one)
- [Installing](#installing)
- [The 90% case](#the-90-case)
- [Units, and the mistake everyone makes](#units-and-the-mistake-everyone-makes)
- [Timing](#timing)
- [Feed-forward](#feed-forward)
- [Anti-windup](#anti-windup)
- [The derivative](#the-derivative)
- [Setpoint weighting](#setpoint-weighting)
- [Manual mode and bumpless transfer](#manual-mode-and-bumpless-transfer)
- [One-way actuators](#one-way-actuators)
- [Output shaping](#output-shaping)
- [Reverse-acting processes](#reverse-acting-processes)
- [Watching what it does](#watching-what-it-does)
- [Tuning](#tuning)
- [Autotuning](#autotuning)
- [The control law in full](#the-control-law-in-full)
- [API reference](#api-reference)
- [Configuration reference](#configuration-reference)
- [Limitations and gotchas](#limitations-and-gotchas)
- [Building and testing](#building-and-testing)
- [Porting from the AxxSolder PID](#porting-from-the-axxsolder-pid)
- [License](#license)

---

## Why this one

Most small PID libraries give you `kp`, `ki`, `kd` and an output clamp, and
stop. That is enough for a demo and not enough for a product. AxxPID adds the
things you end up writing yourself anyway:

| | |
|---|---|
| **Feed-forward, first class** | Constant, setpoint-proportional, setpoint-rate, or your own callback. Applied *inside* the saturation calculation, which is the part everyone gets wrong. |
| **Derivative on measurement, filtered** | No derivative kick on a setpoint step, and an Åström–Hägglund filter so the D term does not just amplify your ADC noise. |
| **Anti-windup you choose** | Conditional integration or back-calculation, plus an explicit clamp on the integral itself. |
| **Bumpless everything** | Manual↔automatic transfer, retuning mid-flight, and resuming from a known actuator position. |
| **Real elapsed time** | `ki` and `kd` mean the same thing whether your loop runs at 25 ms or jitters between 20 and 40. |
| **Introspection** | Read the P, I and D contributions separately — the single most useful thing you can do while tuning. |
| **Asymmetric integral** | Because a heater can heat but cannot cool, and a symmetric integrator will always overshoot on a one-way actuator. |
| **Autotuner included** | Relay feedback, plus the Ziegler–Nichols, Cohen–Coon, Tyreus–Luyben and lambda/SIMC tables. |
| **Actually tested** | 538 assertions across four suites, including closed-loop simulations that check settling time and overshoot, not just arithmetic. |

Measured with `arm-none-eabi-gcc 13.3 -Os`, **192 bytes of RAM** per controller
instance and no heap at all:

| Target | Controller | + tuning module |
|---|---|---|
| Cortex-M4F (hard float) | 3.9 kB | 1.8 kB |
| Cortex-M0+ (soft float) | 4.1 kB | 2.1 kB |

---

## Installing

**Copy the files.** Two of them, or four with the autotuner:

```
include/axxpid/axxpid.h       src/axxpid.c
include/axxpid/axxpid_tune.h  src/axxpid_tune.c
```

Add `include/` to your include path. That is it — there is no configuration
step and no build-time setup.

**CMake**, as a subdirectory or via `FetchContent`:

```cmake
add_subdirectory(AxxPID)
target_link_libraries(my_firmware PRIVATE axxpid::axxpid)
```

**PlatformIO**, in `platformio.ini`:

```ini
lib_deps = https://github.com/axeljohansson/AxxPID.git
```

**STM32CubeIDE**: copy `include/axxpid/` and `src/` into the project, add
`src/axxpid.c` to the build and `include/` to the include paths. There is no
library-wide init and no link-order requirement; each controller is
initialised on its own with `axxpid_init`.

**Arduino IDE**: copy `include/axxpid/` and `src/*.c` into
`Arduino/libraries/AxxPID/src/`, so the layout is `src/axxpid/axxpid.h` and
`src/axxpid.c`.

To run the controller in `double` instead of `float`, define
`AXXPID_USE_DOUBLE=1`. On any MCU with a single-precision FPU, don't.

---

## The 90% case

```c
axxpid_t pid;

axxpid_init(&pid, 2.0f, 0.5f, 0.1f, 0.0f, 100.0f);
```

Three gains and the actuator range. Everything else takes a sensible default:
direct acting, automatic mode, proportional on error, derivative on
measurement, conditional-integration anti-windup, no feed-forward.

Then, once per control period:

```c
float u = axxpid_update(&pid, setpoint, measurement, dt_seconds);
```

The output limits are arguments rather than a separate call because a
controller that does not know what its actuator can do cannot protect its
integrator, and forgetting them is the easiest way to get a badly behaved
loop. Pass `-AXXPID_UNLIMITED, AXXPID_UNLIMITED` if you genuinely want an
unbounded output.

`axxpid_init` also never leaves you holding an uninitialised controller. Even
when it rejects an argument it installs the defaults and zero gains, so code
that ignores the return value gets a controller that does nothing rather than
one running on whatever was on the stack.

---

## Units, and the mistake everyone makes

AxxPID uses the **parallel** (independent-gain) form:

```
u = kp·e  +  ki·∫e dt  +  kd·de/dt
```

so the gains carry units:

| Gain | Units | Meaning |
|---|---|---|
| `kp` | output / error | Output per unit of error, right now. |
| `ki` | output / (error·second) | Output added per second, per unit of error. |
| `kd` | output·second / error | Output per unit of error *rate*. |

`ki` and `kd` are **per second**, always, regardless of how fast your loop
runs. This is where most PID ports go wrong: they fold the sample period into
the gains, so doubling the loop rate silently doubles the integral action.
AxxPID integrates against the real elapsed time you pass in, so changing the
sample rate changes nothing about the tuning.

If you think in **standard** (ISA) form instead — proportional gain, integral
time, derivative time — use:

```c
axxpid_set_tunings_standard(&pid, kp, ti_seconds, td_seconds);
/* which is ki = kp/ti and kd = kp·td */
```

Pass `ti = 0` to turn integral action off.

---

## Timing

Two ways to drive the loop.

**You own the clock** — a timer interrupt or an RTOS periodic task. This is
the better option: a constant period is what every stability margin you tune
for assumes.

```c
void TIM6_IRQHandler(void) {
    float u = axxpid_update(&pid, setpoint, read_sensor(), 0.001f);  /* 1 kHz */
    set_actuator(u);
}
```

**AxxPID owns the clock** — call it as often as you like from a busy main
loop and let it rate-limit itself against any millisecond counter:

```c
axxpid_set_sample_time(&pid, 25, false);   /* recompute every 25 ms */

while (1) {
    if (axxpid_update_at(&pid, setpoint, read_sensor(), HAL_GetTick())) {
        set_actuator(axxpid_get_output(&pid));
    }
    update_display();
    handle_buttons();
}
```

`axxpid_update_at` returns `true` only on the calls where the control law
actually ran, and uses the *measured* interval rather than the nominal one, so
a late call costs you a little accuracy instead of correctness. The
subtraction is unsigned, so a 32-bit tick counter wrapping every 49.7 days is
handled.

If the loop can be stopped for a long time — a fault, a menu, a firmware
update — call `axxpid_reset()` when you resume, or set `axxpid_set_dt_max()`
so a single enormous `dt` cannot dump a huge step into the integrator.

---

## Feed-forward

Feedback control is reactive: it cannot do anything until an error exists.
Feed-forward is the opposite — if you already know roughly what output a given
setpoint needs, apply it immediately and leave the feedback loop to correct
only what your model got wrong.

This is usually the largest single improvement available to a temperature or
motion loop, and it is almost never in a PID library.

**A constant bias** — the output that holds the process at rest:

```c
axxpid_set_feedforward_bias(&pid, 40.0f);
```

**Proportional to the setpoint** — the steady-state inverse of the plant. If
holding the process at `sp` needs an output of `sp/G`, then:

```c
axxpid_set_feedforward_gains(&pid, 1.0f / G, 0.0f);
```

**Proportional to how fast the setpoint is moving** — what makes a ramp track
without lag. For a first-order plant with time constant τ, `rate_gain = τ/G`:

```c
axxpid_set_feedforward_gains(&pid, 1.0f / G, tau / G);
```

**Anything else** — a lookup table, a polynomial, a gain schedule, a measured
disturbance, a full inverse plant model:

```c
static float holding_power(float setpoint, float measurement, void *user) {
    (void)measurement; (void)user;
    return (setpoint - AMBIENT) / HEATER_GAIN;   /* losses grow with ΔT */
}

axxpid_set_feedforward_fn(&pid, holding_power, NULL);
```

All four sum. The result is added **before** the output is clamped and is
included in the anti-windup calculation, which is the part that matters: if
the feed-forward alone saturates the actuator, the integrator sees that and
stops accumulating instead of winding up behind a term it cannot observe.

Feed-forward does not replace the integrator — it is the integrator's head
start. Watch `axxpid_get_i_term()` shrink towards zero as your model gets
better. That is the number that tells you how good your feed-forward is.

See `examples/02_feedforward.c` for all three side by side.

---

## Anti-windup

When the actuator is at its limit the loop is open, and an integrator that
keeps accumulating is writing a cheque the process has to pay back as
overshoot. Three options:

```c
axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_CONDITIONAL, 0.0f);
```

**Conditional integration** (the default). Integration is frozen for a sample
whenever the step would push an output that is already beyond what the
actuator can reach further out. Cheap — two comparisons — and it never exceeds
the limit at all. This is what AxxSolder has run for years. "What the actuator
can reach" includes the slew rate, if you have set one.

```c
axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION, tt);
```

**Back-calculation.** The integrator is bled continuously towards whatever the
actuator actually got:

```
I += (dt/Tt)·(u_limited − u_wanted)
```

Smoother coming off the limit than a hard freeze, and it accounts for the
slew-rate limit as well as the output clamp because the *final* output is fed
back. Start with `Tt = Ti = kp/ki`, or `Tt = √(Ti·Td)` for a full PID.

`dt/Tt` is a feedback gain, so it is capped at 1 internally: a `Tt` that is
sane for your nominal period would otherwise ring, and then diverge, the first
time a sample arrived late.

```c
axxpid_set_integral_limits(&pid, -300.0f, 300.0f);
```

**An explicit clamp on the integral itself**, independent of both. This is the
most direct way to bound how much authority the integrator can ever hold, and
it composes with either strategy above. AxxSolder runs ±300 on a 0–500 output.

The limits default to unbounded, so integral action works out of the box.
(A library that defaults them to zero silently disables the I term and then
you spend an afternoon on it.)

---

## The derivative

The D term is differentiated from the **measurement**, not the error. A
setpoint step is a discontinuity; the measurement is a physical quantity and
cannot jump. Differentiating the error therefore produces an enormous spike on
every setpoint change — "derivative kick" — that slams the actuator into its
limit for a sample. Differentiating the measurement does not.

Derivatives amplify noise in direct proportion to frequency, so unless your
measurement is genuinely clean, filter it:

```c
axxpid_set_derivative_filter(&pid, 10.0f);   /* N, typically 8..16 */
```

This is the standard Åström–Hägglund construction: a first-order low-pass with
`Tf = Td/N = (kd/kp)/N`, which caps the high-frequency gain of the D term at
`kp·N` instead of letting it grow without bound. Lower `N` filters harder;
below about 2 there is no derivative action left, above about 20 there is no
filtering left.

If you retune at run time but the sensor's noise bandwidth does not change,
pin the filter to a fixed time constant instead:

```c
axxpid_set_derivative_filter_tau(&pid, 0.05f);   /* seconds */
```

Filtering is **off by default**, so that `kd` means exactly what the equation
says and AxxPID reproduces the AxxSolder behaviour bit for bit. Turn it on for
almost any real sensor.

The filter coefficient is recomputed from the measured `dt` on every sample,
so loop jitter cannot silently move the corner frequency.

---

## Setpoint weighting

Two-degree-of-freedom control, in one line:

```c
axxpid_set_setpoint_weights(&pid, b, c);
```

The P term acts on `b·sp − pv` and the D term on `c·sp − pv`.

| `b` | Effect |
|---|---|
| `1.0` | Proportional on error. Fastest setpoint tracking, most overshoot. **Default.** |
| `0.3`–`0.7` | Softer response to a setpoint step, identical disturbance rejection. |
| `0.0` | Proportional on measurement. No proportional step at all on a setpoint change. |

| `c` | Effect |
|---|---|
| `0.0` | Derivative on measurement. **Default, and almost always right.** |
| `1.0` | Derivative on error. Reintroduces derivative kick; occasionally wanted for aggressive tracking. |

Lowering `b` is the cleanest knob for "it overshoots when I change the
setpoint, but I don't want to detune the loop", because `b` does not touch the
disturbance response at all.

---

## Manual mode and bumpless transfer

```c
axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
axxpid_set_manual_output(&pid, 35.0f);
```

In manual mode `axxpid_update` returns your value and continuously
back-calculates the integrator to match it, so switching back is seamless:

```c
axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC);
/* The next output continues from 35.0, it does not leap to kp·error. */
```

Same idea when you are starting up and already know where the actuator is:

```c
axxpid_reset_to(&pid, 35.0f);
```

And when you want to retune a live loop without a step in the output:

```c
axxpid_set_bumpless_tuning(&pid, true);
axxpid_set_tunings(&pid, new_kp, new_ki, new_kd);
```

Changing `ki` never causes a step, with or without that flag, because the
integral is accumulated in output units rather than being stored as a raw
error sum and scaled at read time. Changing `kp` or `kd` does, unless you ask
for the compensation.

---

## One-way actuators

A heater can heat but cannot cool. A gravity-fed valve can open but cannot
suck. Once the process is above the setpoint your only actuator is patience —
and a controller tuned for how fast it can push will always overshoot, because
the authority it uses going up has no counterpart coming down.

Two features exist for this, both from AxxSolder.

**Asymmetric integral gain.** Drain the integrator faster than it filled:

```c
axxpid_set_integral_overshoot(&pid, 7.0f, -1.0f);
```

While the error is below `-1.0` — that is, the process has overshot by more
than 1 unit — the integral gain is multiplied by 7. The threshold is not zero
on purpose: a small steady-state error should not trip the fast path, or the
loop hunts around the setpoint.

**Integral engagement band.** Park the integrator while you are still a long
way below target:

```c
axxpid_set_integral_band(&pid, 75.0f);
```

While the process is more than 75 units below the setpoint the integrator is
held at zero. During a cold start the actuator is flat out regardless, so
anything the integrator accumulates there is pure debt.

The band is deliberately **one-sided**. A symmetric version would also dump
the integral when the process is far *above* setpoint, which is exactly when a
one-way actuator needs the negative integral it has built up.

Note that on a loop where `kp·error` already saturates the output during the
whole approach, conditional anti-windup blocks the integrator anyway and the
band changes nothing. It earns its place when `kp` is low enough that the
output is *not* saturated but the error is still far too large for the
integrator to have anything useful to say. Measure before reaching for it.

**And the off switch:**

```c
axxpid_set_integral_reset_on_zero_setpoint(&pid, true);
```

When a setpoint of exactly zero means "off" in your state machine, this makes
the controller let go completely so it restarts from a clean state.

`examples/04_soldering_iron.c` runs all of these on a two-node thermal model.

---

## Output shaping

**Slew-rate limiting**, to protect a mechanical actuator from a step command:

```c
axxpid_set_output_slew_rate(&pid, 50.0f);   /* output units per second */
```

Applied after the output clamp, and the slew-limited value is what
back-calculation feeds back — so a rate limit unwinds the integrator too.

**A deadband**, to stop sensor noise driving an actuator back and forth:

```c
axxpid_set_deadband(&pid, 0.5f);
```

Errors smaller than this produce no P or I action. Larger errors are *shifted*
towards zero by the deadband width rather than passed through unchanged, so
the control signal stays continuous across the band edge instead of stepping.
The D term keeps acting on the raw measurement, so real disturbances are still
caught.

---

## Reverse-acting processes

When raising the output *lowers* the measurement — a cooler, a drain valve, a
brake:

```c
axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE);
```

The control error is negated internally. The gains stay positive and
`axxpid_get_kp()` returns exactly what you set, rather than a sign-flipped
copy you then have to reason about. Changing the sense clears the integrator,
because the accumulated value belongs to the old wiring.

---

## Watching what it does

```c
axxpid_terms_t terms;
axxpid_get_terms(&pid, &terms);

printf("P %.1f  I %.1f  D %.1f  FF %.1f  ->  u %.1f\n",
       terms.p, terms.i, terms.d, terms.ff, terms.output);
```

Or one at a time — the direct equivalents of AxxSolder's `PID_GetPpart`,
`PID_GetIpart` and `PID_GetDpart`:

```c
axxpid_get_p_term(&pid);
axxpid_get_i_term(&pid);
axxpid_get_d_term(&pid);
axxpid_get_ff_term(&pid);
axxpid_get_error(&pid);
axxpid_get_output(&pid);
axxpid_is_saturated(&pid);
```

`axxpid_get_i_term()` is the snapshot that went into the last output, which is
what you want beside it on a graph. `axxpid_get_integral()` is the live
integrator — pair it with `axxpid_set_integral()` to save and restore the
controller across a power cycle.

The four terms always sum to the pre-clamp output. Graphing them separately is
the fastest way to diagnose a loop: an oscillation with a large swinging P is
too much gain, one with a slow-moving I is an integrator that cannot let go,
and a D term that looks like grass is the derivative amplifying noise.

`axxpid_is_saturated()` sitting true for long stretches means the actuator is
undersized for what you are asking, or `kp` is far too high.

---

## Tuning

**By hand**, the order that works:

1. Set `ki = 0`, `kd = 0`. Set the output limits.
2. Raise `kp` until the loop oscillates steadily, then halve it.
3. Raise `ki` until the steady-state error is gone within an acceptable time.
   Too much `ki` shows up as a slow, rolling overshoot.
4. Add `kd` only if you need it. Turn the derivative filter on before you do.
   Too much `kd` shows up as the actuator buzzing on sensor noise.
5. If it overshoots only on setpoint changes and is otherwise fine, lower `b`
   rather than detuning anything.

**From an oscillation test.** Raise `kp` until the loop sustains a constant
oscillation, note that gain as `Ku` and the period as `Tu`, then:

```c
axxpid_gains_t g = axxpid_tune_from_ultimate(AXXPID_RULE_TYREUS_LUYBEN_PID,
                                             ku, tu);
axxpid_tune_apply(&pid, &g);
```

| Rule | `kp` | `Ti` | `Td` | Character |
|---|---|---|---|---|
| `AXXPID_RULE_ZN_P` | 0.5·Ku | — | — | Proportional only |
| `AXXPID_RULE_ZN_PI` | 0.45·Ku | Tu/1.2 | — | No derivative needed |
| `AXXPID_RULE_ZN_PID` | 0.6·Ku | Tu/2 | Tu/8 | Classic, ~25% overshoot |
| `AXXPID_RULE_PESSEN` | 0.7·Ku | Tu/2.5 | 0.15·Tu | Fastest, most overshoot |
| `AXXPID_RULE_SOME_OVERSHOOT` | Ku/3 | Tu/2 | Tu/3 | Gentler |
| `AXXPID_RULE_NO_OVERSHOOT` | 0.2·Ku | Tu/2 | Tu/3 | When overshoot is unacceptable |
| `AXXPID_RULE_TYREUS_LUYBEN_PI` | Ku/3.2 | 2.2·Tu | — | Robust, noisy loops |
| `AXXPID_RULE_TYREUS_LUYBEN_PID` | Ku/2.2 | 2.2·Tu | Tu/6.3 | **A good default** |

Ziegler–Nichols is famous, not gentle. Start at Tyreus–Luyben and tighten only
if you need to.

**From a step test.** Step the output in open loop, read the process gain `K`,
the apparent dead time `L` and the time constant `T` off the response curve:

```c
axxpid_tune_ziegler_nichols_open(k, l, t);  /* aggressive */
axxpid_tune_cohen_coon(k, l, t);            /* better when L/T > 0.3 */
axxpid_tune_lambda(k, l, t, lambda);        /* you pick the closed-loop speed */
```

Lambda/SIMC is the one to reach for in production, because `λ` *is* the
closed-loop time constant: `λ = L` is aggressive, `λ = 3L` is a robust
default, larger is calmer. It is the only rule here where the knob means
something physical.

---

## Autotuning

Relay feedback: drive the process with a bang-bang output around the operating
point, measure the limit cycle it provokes, and recover `Ku` and `Tu` from the
describing function `Ku = 4d/(π√(a²−h²))`.

```c
#include "axxpid/axxpid_tune.h"

axxpid_relay_t relay;
axxpid_relay_config_t cfg;

axxpid_relay_config_default(&cfg);
cfg.setpoint    = 250.0f;   /* tune at the temperature you actually run at */
cfg.output_bias = 100.0f;   /* roughly the power that holds it there       */
cfg.output_step = 30.0f;    /* the relay swings ±30 around that            */
cfg.hysteresis  = 1.0f;     /* just above your measurement noise           */
cfg.cycles      = 4;
cfg.settle_cycles = 2;      /* discard the first two, they are transient   */
cfg.timeout     = 600.0f;

axxpid_relay_init(&relay, &cfg);

float u, ku, tu;
while (axxpid_relay_update(&relay, read_sensor(), dt, &u)
           == AXXPID_RELAY_RUNNING) {
    set_actuator(u);
    wait_one_period();
}

if (axxpid_relay_result(&relay, &ku, &tu) == AXXPID_OK) {
    axxpid_gains_t g = axxpid_relay_gains(&relay,
                                          AXXPID_RULE_TYREUS_LUYBEN_PID);
    axxpid_tune_apply(&pid, &g);
}
```

> **An autotune makes your process oscillate on purpose.** Only run it on a
> plant that can safely swing about the operating point, and size
> `output_step` for an excursion your hardware can live with.

Practical notes:

- **Tune where you run.** Plants are not linear; gains are local. A tip tuned
  at 200 °C is not tuned for 400 °C.
- **Get `output_bias` right.** It should be roughly the output that actually
  holds the process at your setpoint — read it off a manual-mode run. An
  off-centre bias makes the oscillation lopsided and stretches `Tu` (about
  +20% for a bias one relay-amplitude off). If the true holding output falls
  outside `bias ± output_step`, the relay never switches at all and you get
  `AXXPID_RELAY_TIMEOUT` with the actuator pinned.
- **Keep the hysteresis small** — around a tenth of the amplitude it provokes.
  It exists to stop the relay chattering on noise, and it is not free: the
  `√(a²−h²)` term corrects the shifted switching point, worth under one per
  cent, but what hysteresis mainly does is move the oscillation off the
  ultimate frequency, and *that* is not corrected. At `h/a ≈ 0.1` expect `Tu`
  about 10% high; at 0.2, about 20%.
- **Check the state; never assume it finished.** `AXXPID_RELAY_TIMEOUT` means
  no usable limit cycle appeared — either it never crossed the setpoint, or
  every apparent cycle was too short to be real. `AXXPID_RELAY_FAILED` means
  the oscillation was smaller than twice the hysteresis, where the estimate
  stops meaning anything. Neither produces gains.
- **Noise cannot fake a result.** A relay flipping every sample or two is
  following the sensor, not the process, and would otherwise report a tiny
  `Tu` and an enormous `Ku`. Cycles shorter than
  `AXXPID_RELAY_MIN_SAMPLES_PER_CYCLE` (8) are discarded, so a noisy autotune
  times out rather than handing you gains that would wreck the plant.
- The describing function is a first-harmonic approximation. Expect `Tu`
  within a few per cent and `Ku` perhaps 20% low on a lag-dominant process.
  That is fine — you are feeding it into rules with a safety factor built in,
  and `Ku` biased low gives gains biased gentle.

Full worked example in `examples/03_autotune.c`.

---

## The control law in full

Every sample, with `dir` = +1 direct or −1 reverse:

```
e       = dir·(sp − pv)                     clamped through the deadband
P       = kp·(e − dir·(1−b)·sp)             ≡ kp·dir·(b·sp − pv)
d_raw   = Δ[dir·(c·sp − pv)] / dt
d_filt += (dt/(Tf+dt))·(d_raw − d_filt)     Tf = (kd/kp)/N, or 0 for none
D       = kd·d_filt
FF      = bias + kf·sp + kf_rate·Δsp/dt + ff_fn(sp, pv)

ki_eff  = (e < threshold) ? ki·over_gain : ki
ΔI      = ki_eff·e·dt                       skipped if it deepens saturation
I       = clamp(I + ΔI, i_min, i_max)
I       = 0                                 if e > band, or sp == 0 and enabled

u_raw   = P + I + D + FF
u       = clamp(u_raw, out_min, out_max)
u       = clamp(u, u_prev − rate·dt, u_prev + rate·dt)
I      += (dt/Tt)·(u − u_raw)               back-calculation only
```

The first update after a reset produces `D = 0` and no velocity feed-forward,
because there is no history to differentiate against.

A non-finite setpoint, measurement or `dt`, or a `dt ≤ 0`, returns the
previous output and changes nothing — one bad ADC read cannot poison the
integrator.

---

## API reference

### Lifecycle

| Function | Purpose |
|---|---|
| `axxpid_init(pid, kp, ki, kd, out_min, out_max)` | Initialise with these gains and actuator range. |
| `axxpid_config_default(cfg)` | Fill a config struct with the defaults. |
| `axxpid_init_config(pid, cfg)` | Initialise from a fully specified config. |
| `axxpid_reset(pid)` | Clear all state, keep the configuration. |
| `axxpid_reset_to(pid, output)` | Clear state and resume from a known output. |

### Execution

| Function | Purpose |
|---|---|
| `axxpid_update(pid, sp, pv, dt)` | Run one update. Returns the output. |
| `axxpid_update_at(pid, sp, pv, now_ms)` | Rate-limited update on a ms clock. Returns whether it ran. |

### Tuning

| Function | Purpose |
|---|---|
| `axxpid_set_tunings(pid, kp, ki, kd)` | Parallel form. |
| `axxpid_set_tunings_standard(pid, kp, ti, td)` | Standard/ISA form. |
| `axxpid_set_kp/ki/kd(pid, value)` | One gain at a time. |

### Configuration

| Function | Purpose |
|---|---|
| `axxpid_set_output_limits(pid, min, max)` | Actuator range. |
| `axxpid_set_output_slew_rate(pid, rate)` | Maximum change per second. |
| `axxpid_set_integral(pid, value)` | Set the integral term directly. |
| `axxpid_set_integral_limits(pid, min, max)` | Clamp on the integral term. |
| `axxpid_set_antiwindup(pid, mode, tt)` | Conditional, back-calculation, or none. |
| `axxpid_set_integral_band(pid, band)` | Park the integrator when far below setpoint. |
| `axxpid_set_integral_overshoot(pid, gain, threshold)` | Extra integral gain after overshoot. |
| `axxpid_set_integral_reset_on_zero_setpoint(pid, on)` | Zero setpoint means off. |
| `axxpid_set_derivative_filter(pid, n)` | Derivative filter by divisor N. |
| `axxpid_set_derivative_filter_tau(pid, tau)` | Derivative filter by time constant. |
| `axxpid_set_setpoint_weights(pid, b, c)` | Two-degree-of-freedom weights. |
| `axxpid_set_deadband(pid, deadband)` | Ignore small errors. |
| `axxpid_set_feedforward_bias(pid, bias)` | Constant feed-forward. |
| `axxpid_set_feedforward_gains(pid, k_sp, k_rate)` | Setpoint and setpoint-rate feed-forward. |
| `axxpid_set_feedforward_fn(pid, fn, user)` | Arbitrary feed-forward hook. |
| `axxpid_set_acting(pid, acting)` | Direct or reverse. |
| `axxpid_set_mode(pid, mode)` | Manual or automatic. |
| `axxpid_set_manual_output(pid, u)` | The value manual mode returns. |
| `axxpid_set_sample_time(pid, ms, every_call)` | Rate limit for `axxpid_update_at`. |
| `axxpid_set_dt_max(pid, seconds)` | Cap `dt` after a stall. |
| `axxpid_set_bumpless_tuning(pid, on)` | Compensate the integrator when retuning. |

Every setter returns `AXXPID_OK`, `AXXPID_ERR_NULL` or `AXXPID_ERR_PARAM`, and
a rejected call changes nothing.

### Getters

| Function | Purpose |
|---|---|
| `axxpid_get_terms(pid, terms)` | P, I, D, FF, output, error, setpoint and measurement in one struct. |
| `axxpid_get_p_term/i_term/d_term/ff_term(pid)` | One contribution to the last output. |
| `axxpid_get_integral(pid)` | The live integrator, for persistence. |
| `axxpid_get_output(pid)` / `axxpid_get_error(pid)` | Latest output / error. |
| `axxpid_is_saturated(pid)` | Was the last output limited. |
| `axxpid_get_kp/ki/kd(pid)` | The gains, exactly as set. |
| `axxpid_get_mode/acting(pid)` | Current mode / sense. |
| `axxpid_version()` | Version string. |

### Tuning module (`axxpid_tune.h`)

| Function | Purpose |
|---|---|
| `axxpid_gains_from_standard/parallel(...)` | Build a gain set in either form. |
| `axxpid_tune_apply(pid, gains)` | Load a gain set into a controller. |
| `axxpid_tune_from_ultimate(rule, ku, tu)` | Apply a published rule table. |
| `axxpid_tune_ziegler_nichols_open(k, l, t)` | From a step test. |
| `axxpid_tune_cohen_coon(k, l, t)` | From a step test, dead-time dominant. |
| `axxpid_tune_lambda(k, l, t, lambda)` | Lambda / SIMC, you pick the speed. |
| `axxpid_relay_config_default(cfg)` | Relay defaults. |
| `axxpid_relay_init(relay, cfg)` | Start an autotune. |
| `axxpid_relay_update(relay, pv, dt, &u)` | Advance it one sample. |
| `axxpid_relay_result(relay, &ku, &tu)` | Read the measurement. |
| `axxpid_relay_gains(relay, rule)` | Straight from autotune to gains. |

---

## Configuration reference

Every field of `axxpid_config_t`, with its default.

| Field | Default | Notes |
|---|---|---|
| `kp`, `ki`, `kd` | `0` | Parallel form. Must be ≥ 0; use `acting` for reverse. |
| `acting` | `DIRECT` | Raising the output raises the measurement. |
| `mode` | `AUTOMATIC` | |
| `out_min`, `out_max` | ±`AXXPID_UNLIMITED` | Arguments to `axxpid_init`. |
| `out_slew_rate` | `0` | Disabled. Output units per second. |
| `integral_min`, `integral_max` | ±`AXXPID_UNLIMITED` | Clamp on the I term. |
| `antiwindup` | `CONDITIONAL` | |
| `tracking_time` | `1.0` | `Tt`, back-calculation only. |
| `integral_band` | `AXXPID_UNLIMITED` | Disabled. One-sided. |
| `integral_overshoot_gain` | `1.0` | Symmetric, i.e. disabled. |
| `integral_overshoot_threshold` | `0.0` | Error below which the extra gain applies. |
| `integral_reset_on_zero_setpoint` | `false` | |
| `derivative_filter_n` | `0` | Unfiltered. Set 8–16 for a real sensor. |
| `derivative_filter_tau` | `0` | Overrides `n` when > 0. |
| `setpoint_weight_b` | `1.0` | Proportional on error. |
| `setpoint_weight_c` | `0.0` | Derivative on measurement. |
| `deadband` | `0` | Disabled. |
| `ff_bias` | `0` | |
| `ff_setpoint_gain` | `0` | |
| `ff_setpoint_rate_gain` | `0` | |
| `ff_fn`, `ff_user` | `NULL` | |
| `sample_time_ms` | `10` | `axxpid_update_at` only. |
| `update_every_call` | `false` | |
| `dt_max` | `AXXPID_UNLIMITED` | Disabled. |
| `bumpless_tuning` | `false` | |

---

## Limitations and gotchas

- **`ki` and `kd` are per second, not per sample.** If you are porting gains
  from a library that folded the sample period in, they will need scaling.
- **The derivative filter is off by default.** `kd` therefore means exactly
  what the equation says, and on a noisy sensor that is not what you want.
  Turn it on.
- **Not interrupt-safe by itself.** A controller is a plain struct with no
  locking. Call `axxpid_update` from one context. If you read the terms from
  another, either accept a torn read or guard it — the library will not do it
  behind your back.
- **`-ffast-math` disables the NaN guard.** It tells the compiler NaNs cannot
  occur, so the check gets deleted. Do not use it if you rely on that
  protection.
- **A long stall still needs your help.** The controller cannot know the loop
  was paused. Call `axxpid_reset()` on resume, or set `axxpid_set_dt_max()`.
  When `dt_max` does cap a sample, the derivative and velocity feed-forward
  restart rather than differentiate across the gap — the integral still takes
  a capped step, so a stall is attenuated, not erased.
- **Manual→automatic transfer is bumpless unless something else outranks it.**
  The integral limits, the integral engagement band and the zero-setpoint
  reset all exist to stop the integrator holding a value, and all three beat
  the transfer. If you use them, expect a step.
- **A long-lived `float` integrator stops absorbing tiny steps.** Once the
  integral is large, a `ki·e·dt` smaller than its last bit rounds away and
  leaves a small permanent offset. On a slow loop with small gains, build with
  `AXXPID_USE_DOUBLE=1`.
- **There is no lower bound on `dt`.** `axxpid_update_at` cannot go below 1 ms,
  but a direct `axxpid_update` call with a microsecond `dt` will amplify the
  derivative accordingly. Pass the real elapsed time.
- **The autotuner deliberately oscillates the process.** See the warning
  above, and check the returned state rather than assuming it succeeded.
- **Gains are local.** Autotune and tune at the operating point you actually
  run at.
- **`float` by default.** Fine on a Cortex-M4F/M7/M33. On an FPU-less part at
  a high loop rate, budget for soft-float — or reduce the rate, which a
  thermal loop will not notice.

---

## Building and testing

```bash
cmake -B build -DAXXPID_STRICT=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Four suites:

| Suite | Covers |
|---|---|
| `test_core` | Lifecycle, argument validation, NULL safety on every entry point, NaN/Inf rejection, tick wraparound, scheduling. |
| `test_features` | One test per control-law feature, checked against hand-derived arithmetic, plus a regression for every defect found in review. |
| `test_closedloop` | Closed-loop runs against a first-order-plus-dead-time plant: settling, overshoot, disturbance rejection, jitter tolerance, the AxxSolder profile. |
| `test_tune` | Rule tables against the published coefficients; the relay autotuner against a plant whose true `Ku` and `Tu` are computed numerically in the test. |

538 assertions in total, in both `float` and `double`. The expected values
are derived from the difference equations above or from the published tuning
tables, not recorded from a previous run, so they catch a change in behaviour
rather than merely pinning it. Tolerances are set just wide enough to cover
the error that is genuinely inherent — the relay autotuner's `Ku` is checked
against the ~18% low that the describing-function approximation actually costs
on the test plant, not against a 30% band that would notice nothing.

CI builds with GCC and Clang on Linux, macOS and Windows, in both `float` and
`double`, against C99, C11 and C17, from C++, and cross-compiles for Cortex-M4F
and Cortex-M0+ with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
-Wdouble-promotion -Werror`.

Examples build with the tests and run standalone:

```bash
./build/01_minimal          # a PI loop on a tank
./build/02_feedforward      # three kinds of feed-forward, side by side
./build/03_autotune         # relay autotune, then control with the result
./build/04_soldering_iron   # the AxxSolder configuration, explained
```

---

## Porting from the AxxSolder PID

The algorithm is the same and your gains carry over unchanged. The plumbing is
not: the old controller held pointers to your variables and called
`HAL_GetTick()` itself, where AxxPID takes values and returns the output.

Full mapping table, behavioural differences and the two defects fixed along
the way: **[docs/PORTING_AXXSOLDER.md](docs/PORTING_AXXSOLDER.md)**.

---

## License

MIT. See [LICENSE](LICENSE).
