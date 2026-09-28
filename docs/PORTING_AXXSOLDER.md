# Porting from the AxxSolder PID

The algorithm is the same; the plumbing is not. The old controller held
pointers to your variables and read `HAL_GetTick()` itself. AxxPID takes
values and returns the output, which removes the vendor dependency and makes
the whole thing testable.

| AxxSolder | AxxPID |
|---|---|
| `PID_TypeDef` | `axxpid_t` |
| `PID(&p, &in, &out, &sp, kp, ki, kd, dir)` | `axxpid_init(&p, kp, ki, kd)` |
| `PID_Compute(&p)` reads/writes through pointers | `u = axxpid_update(&p, sp, pv, dt)` |
| `PID_SetSampleTime(&p, ms, on_call)` | `axxpid_set_sample_time(&p, ms, on_call)` + `axxpid_update_at()` |
| `PID_SetOutputLimits` | `axxpid_set_output_limits` |
| `PID_SetILimits` | `axxpid_set_integral_limits` |
| `PID_SetIminError` | `axxpid_set_integral_band` |
| `PID_SetNegativeErrorIgainMult(&p, mult, bias)` | `axxpid_set_integral_asymmetry(&p, mult, -1.0f)` |
| `PID_SetTunings` | `axxpid_set_tunings` |
| `PID_SetMode(_PID_MODE_AUTOMATIC)` | `axxpid_set_mode(AXXPID_MODE_AUTOMATIC)` |
| `PID_SetControllerDirection(_PID_CD_DIRECT)` | `axxpid_set_acting(AXXPID_ACTING_DIRECT)` |
| `PID_GetPpart` / `Ipart` / `Dpart` | `axxpid_get_p_term` / `i_term` / `d_term` |
| `PID_GetKp` / `Ki` / `Kd` | `axxpid_get_kp` / `ki` / `kd` |

Your existing gains carry over unchanged — both use the parallel form and both
integrate against real elapsed time.

Five behavioural differences worth knowing:

1. **The `-1` threshold in the asymmetric integral is now a parameter.** It
   was hard-coded; pass `-1.0f` as the second argument to
   `axxpid_set_integral_asymmetry` to keep the old behaviour exactly.
2. **The `NegativeErrorIgainBias` argument is gone.** It was stored but never
   read by the old `PID_Compute`, so removing it changes nothing.
3. **Integral limits of `(0, 0)` are now honoured.** The old
   `PID_SetILimits` rejected `min >= max` and silently did nothing, which made
   `PID_SetILimits(&TPID, 0, 0)` a no-op. AxxPID accepts equal limits and pins
   the integrator there. If you were relying on the no-op, just don't make
   the call.
4. **Reverse action negates the error rather than the gains.** The old code
   flipped the sign of `Kp`, `Ki` and `Kd` in place and kept a second
   `DispKp`/`DispKi`/`DispKd` copy for the getters, and
   `PID_SetControllerDirection` could double-flip if called twice. AxxPID
   stores one positive copy and applies the sense to the error, so the getters
   are always right and the direction can be set as often as you like.

`examples/04_soldering_iron.c` reproduces the full AxxSolder configuration and
`examples/stm32_hal_snippet.c` shows both integration styles in a Cube project.

---
