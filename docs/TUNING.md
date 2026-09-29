# Tuning AxxPID

How to choose gains by measuring the process, rather than by turning knobs.

For the by-hand recipe — raise `kp`, then `ki`, then `kd` — see
[Tuning](../README.md#tuning) in the README. This page covers the published
rules, the two tests that feed them, and the relay autotuner.

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
