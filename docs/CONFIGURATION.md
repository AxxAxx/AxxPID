# Configuration and API reference

Every field and every function. The authoritative version of this is the
header, `include/axxpid/axxpid.h`, which documents each item where it is
declared; this page is the same information laid out as tables.

## Every configuration field

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

## Every function

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
