# AxxPID

A PID controller for embedded C. Two files, no dependencies.

```c
#include "axxpid/axxpid.h"

axxpid_t pid;

/*          kp    ki    kd     min    max   */
axxpid_init(&pid, 8.0f, 2.0f, 0.5f,  0.0f, 100.0f);

for (;;) {
    float power = axxpid_update(&pid, 250.0f, read_sensor(), 0.010f);
    set_heater(power);
}
```

That is a complete, working temperature controller. Everything below is
optional.

---

## Start here

**1. Copy two files into your project.**

```
include/axxpid/axxpid.h
src/axxpid.c
```

Add `include/` to your include path. There is no configuration step and nothing
to initialise at startup. For CMake, PlatformIO, Arduino and STM32CubeIDE, see
[Installing](#installing).

**2. Make one controller per thing you are controlling.**

```c
axxpid_t pid;
axxpid_init(&pid, kp, ki, kd, out_min, out_max);
```

`out_min` and `out_max` are the real range of your actuator — 0 to 255 for an
8-bit PWM, 0 to 100 for a percentage, −1000 to 1000 for a motor that runs both
ways. They are arguments rather than a later call because the controller needs
them to protect itself.

**3. Call it at a steady rate and drive your actuator with what it returns.**

```c
float u = axxpid_update(&pid, setpoint, measurement, dt);
```

- **setpoint** — the value you want.
- **measurement** — what your sensor reads right now.
- **dt** — seconds since you last called it. `0.010f` for a 100 Hz loop.

That is the whole integration. If you do not know what gains to use, go to
[Tuning](#tuning). You will not break anything by getting them wrong; the loop
will just be slow, or it will oscillate.

**Two things worth knowing on day one:**

- If a bigger output makes your reading go **down** — a cooler, a brake, a
  drain valve — add `axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE);`
- If your sensor is noisy and you are using `kd`, add
  `axxpid_set_derivative_filter(&pid, 10.0f);` — see
  [The derivative](#the-derivative).

Build and run `examples/01_minimal.c` on your desktop to watch a loop settle
before you put anything on hardware.

---

## Contents

- [Why this one](#why-this-one)
- [Installing](#installing)
- [Units: the mistake everyone makes](#units-the-mistake-everyone-makes)
- [Timing](#timing)
- [Tuning](#tuning)
- [Watching what it does](#watching-what-it-does)
- [Feed-forward](#feed-forward)
- [Anti-windup](#anti-windup)
- [The derivative](#the-derivative)
- [One-way actuators](#one-way-actuators)
- [Output shaping](#output-shaping)
- [Manual mode](#manual-mode)
- [Softening setpoint changes](#softening-setpoint-changes)
- [Autotuning](#autotuning)
- [Limitations and gotchas](#limitations-and-gotchas)
- [More documentation](#more-documentation)

---

## Why this one

Most small PID libraries give you `kp`, `ki`, `kd` and an output clamp, and
stop. That is enough for a demo and not enough for a product. AxxPID adds the
things you end up writing yourself anyway.

| | |
|---|---|
| **Feed-forward, first class** | Apply the output you already know a setpoint needs, instead of waiting for the loop to discover it. |
| **A derivative that survives a real sensor** | Taken from the measurement, so a setpoint change does not spike the output, and filtered so it does not amplify noise. |
| **Anti-windup you choose** | Two strategies, plus a hard limit on the integral. |
| **Bumpless everything** | Switch to manual and back, or retune mid-flight, without the output jumping. |
| **Real elapsed time** | `ki` and `kd` mean the same thing whether your loop runs at 25 ms or jitters between 20 and 40. |
| **You can see inside it** | Read the P, I and D contributions separately — the fastest way to work out why a loop misbehaves. |
| **Built for one-way actuators** | A heater can heat but not cool. AxxPID has the asymmetry that needs. |
| **Autotuner included** | Measures your process and picks gains for you. |

No dynamic allocation, no globals, no vendor HAL, not even `<math.h>`. Every
controller lives in a struct you allocate yourself, so you can have as many as
you like — on the stack, in `.bss`, or inside another struct.

Measured with `arm-none-eabi-gcc -Os`, **192 bytes of RAM** per controller and
no heap at all:

| Target | Controller | + autotuner |
|---|---|---|
| Cortex-M4F (hard float) | 4.4 kB | 2.3 kB |
| Cortex-M0+ (soft float) | 4.7 kB | 2.7 kB |

It is the control loop from the
[AxxSolder](https://github.com/AxelJohanssonSWE/AxxSolder) soldering station,
pulled out and generalised.

---

## Installing

**Copy the files.** Two of them, or four if you want the autotuner:

```
include/axxpid/axxpid.h       src/axxpid.c
include/axxpid/axxpid_tune.h  src/axxpid_tune.c
```

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
`src/axxpid.c` to the build and `include/` to the include paths.

**Arduino IDE**: copy `include/axxpid/` and `src/*.c` into
`Arduino/libraries/AxxPID/src/`, so the layout is `src/axxpid/axxpid.h` and
`src/axxpid.c`.

To run in `double` instead of `float`, define `AXXPID_USE_DOUBLE=1`. On any MCU
with a single-precision FPU, don't.

---

## Units: the mistake everyone makes

AxxPID uses the **parallel** form, where each gain acts on its own term:

```
u = kp·e  +  ki·∫e dt  +  kd·de/dt
```

so the gains carry units:

| Gain | Units | Meaning |
|---|---|---|
| `kp` | output / error | Output per unit of error, right now. |
| `ki` | output / (error·second) | Output added per second, per unit of error. |
| `kd` | output·second / error | Output per unit of error *rate*. |

**`ki` and `kd` are per second, always** — not per sample, whatever rate your
loop runs at. This is where most PID ports go wrong: they fold the sample
period into the gains, so doubling the loop rate silently doubles the integral
action. AxxPID integrates against the real elapsed time you pass in, so
changing the sample rate changes nothing about the tuning.

If you think in **integral time** and **derivative time** instead — the form
industrial controllers use — say so:

```c
axxpid_set_tunings_standard(&pid, kp, ti_seconds, td_seconds);
/* which is ki = kp/ti and kd = kp·td */
```

Pass `ti = 0` to turn integral action off.

---

## Timing

Two ways to drive the loop.

**You own the clock** — a timer interrupt or an RTOS task. This is the better
option: a constant period is what every stability margin you tune for assumes.

```c
void TIM6_IRQHandler(void) {
    float u = axxpid_update(&pid, setpoint, read_sensor(), 0.001f);  /* 1 kHz */
    set_actuator(u);
}
```

**AxxPID owns the clock** — call it as often as you like from a busy main loop
and let it rate-limit itself against any millisecond counter:

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
a late call costs a little accuracy instead of correctness. The subtraction is
unsigned, so a 32-bit tick counter wrapping every 49.7 days is handled.

If the loop can be stopped for a long time — a fault, a menu, a firmware
update — call `axxpid_reset()` when you resume.

---

## Tuning

If you have never tuned a loop before, do this, in this order:

1. Set `ki = 0` and `kd = 0`. Set the output limits to your real actuator
   range.
2. Raise `kp` until the loop oscillates steadily, then halve it.
3. Raise `ki` until the leftover error is gone within a time you are happy
   with. Too much `ki` looks like a slow, rolling overshoot.
4. Add `kd` only if you need it, and turn the
   [derivative filter](#the-derivative) on before you do. Too much `kd` looks
   like the actuator buzzing on sensor noise.
5. If it overshoots only when you *change* the setpoint and is otherwise fine,
   see [Softening setpoint changes](#softening-setpoint-changes) rather than
   detuning anything.

Watch the P, I and D terms separately while you do this — see
[Watching what it does](#watching-what-it-does). It turns tuning from guesswork
into reading a graph.

If you would rather measure the process than turn knobs, AxxPID can do that:
see [Autotuning](#autotuning).

**Ziegler–Nichols and the other published rules**, step tests, and the full
tables are in **[docs/TUNING.md](docs/TUNING.md)**.

---

## Watching what it does

```c
axxpid_terms_t terms;
axxpid_get_terms(&pid, &terms);

printf("pv %.1f  P %.1f  I %.1f  D %.1f  FF %.1f  ->  u %.1f\n",
       terms.measurement, terms.p, terms.i, terms.d, terms.ff, terms.output);
```

Or one at a time:

```c
axxpid_get_p_term(&pid);   axxpid_get_error(&pid);
axxpid_get_i_term(&pid);   axxpid_get_output(&pid);
axxpid_get_d_term(&pid);   axxpid_is_saturated(&pid);
axxpid_get_ff_term(&pid);
```

`P + I + D + FF` is always exactly the output the control law asked for, before
clamping. Graphing them separately is the fastest way to diagnose a loop:

- A large P swinging back and forth — too much gain.
- A slow-moving I that will not settle — the integrator cannot let go.
- A D term that looks like jagged noise — the derivative is amplifying your
  sensor. Filter it.
- `axxpid_is_saturated()` true for long stretches — the actuator is undersized,
  or `kp` is far too high.

---

## Feed-forward

Feedback only acts *after* an error appears. Feed-forward acts first: if you
already know roughly what output a given setpoint needs, apply it immediately
and leave the feedback loop to correct only what your guess got wrong.

This is usually the largest single improvement available to a temperature or
motion loop, and it is almost never in a PID library.

**A constant** — the output that holds the process at rest:

```c
axxpid_set_feedforward_bias(&pid, 40.0f);
```

**Proportional to the setpoint.** If holding the process at `sp` needs an
output of `sp/G`:

```c
axxpid_set_feedforward_gains(&pid, 1.0f / G, 0.0f);
```

**Proportional to how fast the setpoint is moving** — what makes a ramp track
without lagging behind:

```c
axxpid_set_feedforward_gains(&pid, 1.0f / G, tau / G);
```

**Anything else** — a lookup table, a gain schedule, a measured disturbance:

```c
static float holding_power(float setpoint, float measurement, void *user) {
    (void)measurement; (void)user;
    return (setpoint - AMBIENT) / HEATER_GAIN;   /* losses grow with ΔT */
}

axxpid_set_feedforward_fn(&pid, holding_power, NULL);
```

All four add together. The result goes in *before* the output is clamped and is
part of the anti-windup calculation, which is the part that matters: if the
feed-forward alone saturates the actuator, the integrator sees that and stops
accumulating instead of winding up behind a term it cannot observe.

Feed-forward does not replace the integrator — it is the integrator's head
start. Watch `axxpid_get_i_term()` shrink towards zero as your model gets
better. That number tells you how good your feed-forward is.

`examples/02_feedforward.c` runs three kinds side by side.

---

## Anti-windup

When the actuator is at its limit, the integral keeps growing but cannot do
anything about it. Release the limit and all that stored-up integral has to be
unwound before the output moves, so the process sails past the setpoint. That
is **windup**, and it is the most common reason a loop overshoots badly.

Three tools, which combine:

```c
axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_CONDITIONAL, 0.0f);
```

**Conditional integration** (the default). Integration is frozen for a sample
whenever the step would push an output that is already beyond what the actuator
can reach further out. Cheap, and it never exceeds the limit at all. This is
what AxxSolder has run for years.

```c
axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION, tt);
```

**Back-calculation.** Each sample the integral is pulled a little way towards
the value that would have produced the output the actuator actually got.
Smoother coming off a limit than a hard freeze. Start with `tt` equal to the
integral time, `kp/ki`.

```c
axxpid_set_integral_limits(&pid, -300.0f, 300.0f);
```

**A hard limit on the integral**, independent of both. The most direct way to
cap how much the integrator can ever contribute. AxxSolder runs ±300 on a
0–500 output.

If you do not set them, `axxpid_init` picks limits ten output spans either side
of your output range — wide enough never to interfere, but not unbounded.
Unbounded is dangerous: one wild sensor reading can push the integral so far
that ordinary steps round away to nothing, and the loop sits at a limit for
good.

---

## The derivative

The D term is taken from the **measurement**, not the error. A setpoint can
jump; a physical measurement cannot. Differentiating the error therefore
produces a huge spike every time you change the setpoint — "derivative kick" —
that slams the actuator into its limit for a sample. Differentiating the
measurement does not.

Derivatives amplify noise. Unless your measurement is genuinely clean, filter
it:

```c
axxpid_set_derivative_filter(&pid, 10.0f);   /* N, typically 8..16 */
```

`N` sets how hard: lower filters more. Below about 2 there is no derivative
action left; above about 20 there is no filtering left. **This is off by
default**, so `kd` means exactly what the equation says — but turn it on for
almost any real sensor.

The filter coefficient is recomputed from the measured `dt` every sample, so an
irregular loop period cannot quietly move the cut-off frequency.

---

## One-way actuators

A heater can heat but not cool. Once the process is above the setpoint, all the
controller can do is wait — and a loop tuned for how fast it can push will
always overshoot, because the authority it uses going up has no counterpart
coming down.

Two settings exist for this, both from AxxSolder.

**Drain the integrator faster than it filled:**

```c
axxpid_set_integral_overshoot(&pid, 7.0f, -1.0f);
```

While the error is below −1 — the process has overshot by more than one unit —
the integral gain is multiplied by 7. The threshold is not zero on purpose: a
small leftover error should not trip the fast path, or the loop ends up slowly
oscillating around the setpoint.

**Park the integrator while you are still far below target:**

```c
axxpid_set_integral_band(&pid, 75.0f);
```

While the process is more than 75 units below the setpoint the integral is held
at zero. During a cold start the actuator is flat out regardless, so anything
the integrator collects there only comes back as overshoot later.

This one is deliberately **one-sided**. A symmetric version would also dump the
integral when the process is far *above* setpoint, which is exactly when a
one-way actuator needs it.

**And the off switch:**

```c
axxpid_set_integral_reset_on_zero_setpoint(&pid, true);
```

When a setpoint of exactly zero means "off" in your state machine, this makes
the controller let go completely so it restarts clean.

`examples/04_soldering_iron.c` runs all of these on a two-node thermal model
and measures what each one is worth.

---

## Output shaping

**Slew-rate limiting**, to protect a mechanical actuator from a step command:

```c
axxpid_set_output_slew_rate(&pid, 50.0f);   /* output units per second */
```

Both anti-windup strategies account for it, so the integrator is held back
while the output is rate-limited.

**A deadband**, to stop sensor noise driving an actuator back and forth:

```c
axxpid_set_deadband(&pid, 0.5f);
```

Errors smaller than this leave the controller acting as though the measurement
were exactly on setpoint. Larger errors are *shifted* towards zero by the
deadband width rather than passed through unchanged, so the control signal
stays continuous across the edge instead of stepping.

---

## Manual mode

```c
axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);   /* holds the current output */
axxpid_set_manual_output(&pid, 35.0f);       /* or command a specific one */
```

In manual mode `axxpid_update` returns your value and keeps the integral
matched to it, so switching back is seamless:

```c
axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC);
/* continues from 35.0 — it does not leap to kp·error */
```

Same idea when starting up and you already know where the actuator is:

```c
axxpid_reset_to(&pid, 35.0f);
```

And to retune a running loop without a step in the output:

```c
axxpid_set_bumpless_tuning(&pid, true);
axxpid_set_tunings(&pid, new_kp, new_ki, new_kd);
```

Changing `ki` never causes a step, with or without that flag, because the
integral is accumulated in output units rather than stored as a raw error sum
and scaled at read time.

---

## Softening setpoint changes

If the loop is well behaved against disturbances but overshoots whenever you
*change* the setpoint, the cleanest fix is not to detune it:

```c
axxpid_set_setpoint_weights(&pid, 0.5f, 0.0f);
```

The first number scales how much of a setpoint change the proportional term
sees. `1.0` is the default and reacts fully; `0.0` ignores setpoint changes
entirely and responds only to the measurement; in between trades tracking speed
for overshoot. It does not touch how the loop rejects disturbances at all,
which is why it is the right knob for this job.

(The second number does the same for the derivative term. Leave it at `0`.)

---

## Autotuning

AxxPID can measure your process and pick gains for you. It drives the output
hard one way and then the other to make the process swing, measures how big and
how fast the swing is, and works the gains out from that.

> **This deliberately makes your process oscillate.** Only run it on a plant
> that can safely swing about the operating point, and size `output_step` for
> an excursion your hardware can live with.

```c
#include "axxpid/axxpid_tune.h"

axxpid_relay_t relay;
axxpid_relay_config_t cfg;

axxpid_relay_config_default(&cfg);
cfg.setpoint    = 250.0f;   /* tune at the temperature you actually run at */
cfg.output_bias = 100.0f;   /* roughly the power that holds it there       */
cfg.output_step = 30.0f;    /* it will swing ±30 around that               */
cfg.hysteresis  = 1.0f;     /* just above your measurement noise           */

axxpid_relay_init(&relay, &cfg);

float u;
while (axxpid_relay_update(&relay, read_sensor(), dt, &u)
           == AXXPID_RELAY_RUNNING) {
    set_actuator(u);
    wait_one_period();
}

if (axxpid_relay_result(&relay, NULL, NULL) == AXXPID_OK) {
    axxpid_gains_t g = axxpid_relay_gains(&relay,
                                          AXXPID_RULE_TYREUS_LUYBEN_PID);
    axxpid_tune_apply(&pid, &g);
}
```

**Always check the state it returns.** `AXXPID_RELAY_TIMEOUT` means no usable
oscillation appeared — usually `output_bias` is wrong, so the process never
crossed the setpoint. `AXXPID_RELAY_FAILED` means the swing was too small to
measure. Neither produces gains.

Gains are local: a tip tuned at 200 °C is not tuned for 400 °C. Tune where you
run.

Every rule, the practical notes and the maths are in
**[docs/TUNING.md](docs/TUNING.md)**. Worked example in
`examples/03_autotune.c`.

---

## Limitations and gotchas

- **`ki` and `kd` are per second, not per sample.** If you are porting gains
  from a library that folded the sample period in, they need scaling.
- **The derivative filter is off by default.** On a noisy sensor that is not
  what you want. Turn it on.
- **Not interrupt-safe by itself.** A controller is a plain struct with no
  locking. Call `axxpid_update` from one context. If you read the terms from
  another, guard it yourself.
- **`-ffast-math` disables the NaN guard.** It tells the compiler NaNs cannot
  occur, so the check is deleted. Do not use it if you rely on that protection.
- **A long stall still needs your help.** The controller cannot know the loop
  was paused. Call `axxpid_reset()` on resume, or set `axxpid_set_dt_max()`.
- **A wildly wrong sensor reading still costs you a transient.** NaN and
  infinity are rejected outright, and no single sample can move the integral by
  more than ten times the output range, so one bad reading can no longer leave
  the controller stuck for good. It can still leave the integral far enough out
  to take tens of seconds to unwind. Range-check your sensor; the controller
  cannot know what "plausible" means for your process.
- **Manual → automatic is bumpless unless something outranks it.** The integral
  limits, the integral band and the zero-setpoint reset all exist to stop the
  integrator holding a value, and all three win. If you use them, expect a step.
- **A long-lived `float` integrator stops absorbing tiny steps.** Once the
  integral is large, a `ki·e·dt` smaller than its last bit rounds away. On a
  slow loop with small gains, build with `AXXPID_USE_DOUBLE=1`.
- **The autotuner deliberately oscillates the process**, and you must check the
  state it returns rather than assume it succeeded.

---

## More documentation

| | |
|---|---|
| [docs/TUNING.md](docs/TUNING.md) | Every tuning rule, step tests, autotuner detail. |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | Every config field and every function, as tables. |
| [docs/CONTROL_LAW.md](docs/CONTROL_LAW.md) | The exact equations and the order they run in. |
| [docs/BUILDING.md](docs/BUILDING.md) | Building, the test suite, CI, and the invariants to keep if you change the code. |
| [docs/PORTING_AXXSOLDER.md](docs/PORTING_AXXSOLDER.md) | Moving from the original AxxSolder PID. |
| `include/axxpid/axxpid.h` | The authoritative reference — every function documented where it is declared. |

Examples, in the order worth reading them:

| | |
|---|---|
| `examples/01_minimal.c` | A PI loop on a simulated tank. Start here. |
| `examples/02_feedforward.c` | Three kinds of feed-forward, side by side. |
| `examples/03_autotune.c` | Autotune, then control with the result. |
| `examples/04_soldering_iron.c` | The AxxSolder configuration, explained and measured. |
| `examples/stm32_hal_snippet.c` | Both timing patterns in a CubeIDE project. |
| `examples/arduino/` | An Arduino thermostat sketch. |

---

## License

MIT. See [LICENSE](LICENSE).
