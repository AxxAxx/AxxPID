# The control law in full

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

---

## The order of the steps, and why

The integrator is written twice in one update, which is the part that catches
people reading the source.

1. Reject a bad sample. Nothing after this may see a NaN or a bad `dt`.
2. Cap `dt` at `dt_max`. A capped sample restarts the derivative.
3. Error, then the deadband.
4. P, D and feed-forward. None of these need the integrator.
5. Manual mode returns here, holding the integral equal to the manual output
   minus P, D and FF.
6. Bumpless preload, once, after a mode switch or `axxpid_reset_to()`.
7. Integral step. Conditional anti-windup needs P + D + FF from step 4 to work
   out where the output would land.
8. Add the integral, clamp to the output limits, apply the slew rate.
9. **Record the term breakdown here**, so `P + I + D + FF` is exactly the
   output before clamping.
10. Back-calculation. It needs the final output from step 8, so it runs after
    the snapshot — hence the second write, and hence `axxpid_get_i_term()`
    (the snapshot) differing from `axxpid_get_integral()` (the live value).
11. Save this sample for the next one.

## Bounds that always apply

Two limits are not configuration and cannot be switched off:

- No single update may change the integral by more than **ten times the output
  range**. A step that large is never useful control, and it is how one
  implausible sensor reading throws the integrator somewhere it takes a long
  time to walk back from.
- If you leave the integral limits at their default, `axxpid_init` sets them
  to ten output spans either side of the output range. Unbounded is dangerous:
  once the stored integral is large enough, an ordinary step is smaller than
  its last floating-point bit and rounds away to nothing, and the loop sits at
  a limit for good. An explicit choice, however wide, is always respected.
