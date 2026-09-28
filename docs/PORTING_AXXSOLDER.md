# Porting from the AxxSolder PID

The maths is the same. The way you call it is not.

The old controller kept pointers to your variables and read `HAL_GetTick()`
itself. AxxPID takes values and gives you back the output. That is what
removes the dependency on the ST HAL, and what makes the controller testable
on a desktop.

Your gains carry over unchanged. Both use the parallel form (`kp`, `ki`, `kd`)
and both integrate against the real elapsed time, so nothing needs rescaling.

## Before and after

```c
/* AxxSolder */
PID(&TPID, &sensor_values.thermocouple_temperature,
    &sensor_values.requested_power, &PID_setpoint,
    0, 0, 0, _PID_CD_DIRECT);
PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
PID_SetSampleTime(&TPID, 25, 0);
PID_SetOutputLimits(&TPID, 0, 500);
PID_SetILimits(&TPID, -300, 300);
PID_SetIminError(&TPID, 75);
PID_SetNegativeErrorIgainMult(&TPID, 7, 1);

while (1) {
    PID_Compute(&TPID);                    /* writes requested_power */
    heater_set(sensor_values.requested_power);
}
```

```c
/* AxxPID */
axxpid_init(&pid, 8.0f, 2.0f, 0.5f, 0.0f, 500.0f);
axxpid_set_integral_limits(&pid, -300.0f, 300.0f);
axxpid_set_integral_band(&pid, 75.0f);
axxpid_set_integral_overshoot(&pid, 7.0f, -1.0f);
axxpid_set_sample_time(&pid, 25, false);

while (1) {
    if (axxpid_update_at(&pid, setpoint, read_tip(), HAL_GetTick())) {
        heater_set(axxpid_get_output(&pid));
    }
}
```

## Function by function

| AxxSolder | AxxPID |
|---|---|
| `PID_TypeDef` | `axxpid_t` |
| `PID(&p, &in, &out, &sp, kp, ki, kd, dir)` | `axxpid_init(&p, kp, ki, kd, out_min, out_max)` |
| `PID_Compute(&p)`, writing through pointers | `u = axxpid_update(&p, sp, pv, dt)` |
| `PID_SetSampleTime(&p, ms, on_call)` | `axxpid_set_sample_time(&p, ms, on_call)`, then use `axxpid_update_at()` |
| `PID_SetOutputLimits(&p, lo, hi)` | now arguments to `axxpid_init`, or `axxpid_set_output_limits` |
| `PID_SetILimits` | `axxpid_set_integral_limits` |
| `PID_SetIminError` | `axxpid_set_integral_band` |
| `PID_SetNegativeErrorIgainMult(&p, mult, bias)` | `axxpid_set_integral_overshoot(&p, mult, -1.0f)` |
| `PID_SetTunings` | `axxpid_set_tunings` |
| `PID_SetMode(_PID_MODE_AUTOMATIC)` | `axxpid_set_mode(&p, AXXPID_MODE_AUTOMATIC)` |
| `PID_SetControllerDirection(_PID_CD_DIRECT)` | `axxpid_set_acting(&p, AXXPID_ACTING_DIRECT)` |
| `PID_GetPpart` / `Ipart` / `Dpart` | `axxpid_get_p_term` / `i_term` / `d_term` |
| `PID_GetKp` / `Ki` / `Kd` | `axxpid_get_kp` / `ki` / `kd` |
| `float_clamp`, `check_clamping` | internal; not part of the API |

Automatic mode is the default, so the `PID_SetMode` call is no longer needed
unless you want manual mode.

## Six differences in behaviour

1. **The `-1` threshold in the asymmetric integral is now a parameter.** It
   used to be hard-coded. Pass `-1.0f` as the second argument to
   `axxpid_set_integral_overshoot` to keep the old behaviour exactly.

2. **The `NegativeErrorIgainBias` argument is gone.** The old `PID_Compute`
   stored it but never used it, so dropping it changes nothing.

3. **Integral limits of `(0, 0)` now do what they say.** The old
   `PID_SetILimits` rejected `min >= max` and quietly did nothing, so
   `PID_SetILimits(&TPID, 0, 0)` was a no-op that left whatever the struct was
   initialised with. AxxPID accepts equal limits and pins the integral there.
   If you were relying on the no-op, just delete the call.

4. **Reverse action flips the error, not the gains.** The old code negated
   `Kp`, `Ki` and `Kd` in place and kept a second `DispKp`/`DispKi`/`DispKd`
   copy so the getters could still report the original values, and
   `PID_SetControllerDirection` could flip twice if you called it twice.
   AxxPID keeps one positive copy and applies the direction to the error
   instead. The getters are always right and you can set the direction as
   often as you like.

5. **Anti-windup checks which limit was hit, not the sign of the output.**
   The old test was `error * output > 0`, which only identifies the right
   limit when the output range straddles zero. On an actuator that runs
   between, say, 20% and 80%, it would let the integral run straight through
   the lower limit. AxxPID tests the limit that was actually exceeded. On
   AxxSolder's own 0..500 range the two agree, so your tuning is unaffected.

6. **The output is returned, not written through a pointer.** There is no
   `MyOutput` to read afterwards, though `axxpid_get_output()` will give you
   the last value if that suits your code better.

## Worked examples

- `examples/04_soldering_iron.c` — the full AxxSolder configuration, with each
  setting explained and measured on a two-node thermal model.
- `examples/stm32_hal_snippet.c` — both integration styles in a CubeIDE
  project: a fixed-rate timer interrupt, and a free-running main loop driven
  by `HAL_GetTick()`.
