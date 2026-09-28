/**
 * @file  axxpid.c
 * @brief AxxPID controller implementation.
 *
 * No standard library dependency beyond the fixed-width integer and boolean
 * headers pulled in by axxpid.h: no <math.h>, no <stdlib.h>, no allocation,
 * no globals, no vendor HAL. Everything is plain arithmetic on
 * ::axxpid_real_t.
 *
 * Naming: every function in this file that is not declared in the public
 * header is `static`, which is the whole of what "private" means in C. They
 * keep the `axxpid_` prefix only so that amalgamating this file into a unity
 * build cannot collide with another translation unit's helpers - not as an
 * API claim. Note that a doubled underscore would be a poor marker here: C
 * reserves only leading underscores, but C++ reserves any identifier
 * containing `__` anywhere, and these sources are meant to survive being
 * compiled by a C++ toolchain.
 */

#include "axxpid/axxpid.h"

/* -------------------------------------------------------------------------- */
/* Small helpers                                                              */
/* -------------------------------------------------------------------------- */

/** @brief Clamp @p v into [@p lo, @p hi]. */
static axxpid_real_t axxpid_clamp(axxpid_real_t v,
                                   axxpid_real_t lo,
                                   axxpid_real_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/**
 * @brief Reject NaN and infinities without pulling in <math.h>.
 *
 * NaN fails every comparison including equality with itself; infinities fall
 * outside the ::AXXPID_UNLIMITED sentinel range, which is deliberately finite
 * for exactly this reason.
 *
 * @note Compiling with @c -ffast-math tells the compiler that NaNs cannot
 *       occur and lets it delete the @c v == v test. Do not use
 *       @c -ffast-math if you rely on this guard.
 */
static bool axxpid_is_finite(axxpid_real_t v)
{
    return (v == v) && (v <= AXXPID_UNLIMITED) && (v >= -AXXPID_UNLIMITED);
}

/**
 * @brief Validate a whole configuration before it is copied into a controller.
 *
 * Finiteness is checked first and separately, because a NaN compares false
 * against everything: a range test of the form @c x<0 waves NaN straight
 * through, and it would then poison the integrator on the first update.
 */
static bool axxpid_config_is_valid(const axxpid_config_t *cfg)
{
    const axxpid_real_t *const reals[] = {
        &cfg->kp,
        &cfg->ki,
        &cfg->kd,
        &cfg->out_min,
        &cfg->out_max,
        &cfg->out_slew_rate,
        &cfg->integral_min,
        &cfg->integral_max,
        &cfg->tracking_time,
        &cfg->integral_band,
        &cfg->integral_overshoot_gain,
        &cfg->integral_overshoot_threshold,
        &cfg->derivative_filter_n,
        &cfg->derivative_filter_tau,
        &cfg->setpoint_weight_b,
        &cfg->setpoint_weight_c,
        &cfg->deadband,
        &cfg->ff_bias,
        &cfg->ff_setpoint_gain,
        &cfg->ff_setpoint_rate_gain,
        &cfg->dt_max
    };
    size_t i;

    for (i = 0; i < (sizeof(reals) / sizeof(reals[0])); ++i) {
        if (!axxpid_is_finite(*reals[i])) {
            return false;
        }
    }

    if ((cfg->kp < 0) || (cfg->ki < 0) || (cfg->kd < 0)) {
        return false;
    }
    if ((cfg->acting != AXXPID_ACTING_DIRECT) &&
        (cfg->acting != AXXPID_ACTING_REVERSE)) {
        return false;
    }
    if ((cfg->mode != AXXPID_MODE_MANUAL) &&
        (cfg->mode != AXXPID_MODE_AUTOMATIC)) {
        return false;
    }
    if (!(cfg->out_min < cfg->out_max)) {
        return false;
    }
    if (cfg->out_slew_rate < 0) {
        return false;
    }
    if (cfg->integral_min > cfg->integral_max) {
        return false;
    }
    switch (cfg->antiwindup) {
        case AXXPID_ANTIWINDUP_NONE:
        case AXXPID_ANTIWINDUP_CONDITIONAL:
            break;
        case AXXPID_ANTIWINDUP_BACK_CALCULATION:
            if (!(cfg->tracking_time > 0)) {
                return false;
            }
            break;
        default:
            return false;
    }
    if (cfg->integral_band < 0) {
        return false;
    }
    if (cfg->integral_overshoot_gain < 0) {
        return false;
    }
    if (cfg->derivative_filter_n < 0) {
        return false;
    }
    if (cfg->derivative_filter_tau < 0) {
        return false;
    }
    if ((cfg->setpoint_weight_b < 0) || (cfg->setpoint_weight_b > 1)) {
        return false;
    }
    if ((cfg->setpoint_weight_c < 0) || (cfg->setpoint_weight_c > 1)) {
        return false;
    }
    if (cfg->deadband < 0) {
        return false;
    }
    if (cfg->sample_time_ms == 0u) {
        return false;
    }
    if (!(cfg->dt_max > 0)) {
        return false;
    }
    return true;
}

/** @brief Clear every piece of runtime state, leaving the configuration alone. */
static void axxpid_clear_state(axxpid_t *pid)
{
    pid->integral = 0;
    pid->d_filtered = 0;
    pid->prev_d_input = 0;
    pid->prev_setpoint = 0;
    pid->p_input = 0;
    pid->setpoint = 0;
    pid->measurement = 0;

    pid->output = 0;
    pid->manual_output = 0;
    pid->error = 0;
    pid->p_term = 0;
    pid->i_term = 0;
    pid->d_term = 0;
    pid->ff_term = 0;
    pid->saturated = false;

    pid->first_update = true;
    pid->bumpless_pending = false;
    pid->has_time = false;
    pid->last_time_ms = 0u;
}

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

axxpid_status_t axxpid_config_default(axxpid_config_t *cfg)
{
    if (cfg == (axxpid_config_t *)0) {
        return AXXPID_ERR_NULL;
    }

    cfg->kp = 0;
    cfg->ki = 0;
    cfg->kd = 0;

    cfg->acting = AXXPID_ACTING_DIRECT;
    cfg->mode = AXXPID_MODE_AUTOMATIC;

    cfg->out_min = -AXXPID_UNLIMITED;
    cfg->out_max = AXXPID_UNLIMITED;
    cfg->out_slew_rate = 0;

    cfg->integral_min = -AXXPID_UNLIMITED;
    cfg->integral_max = AXXPID_UNLIMITED;
    cfg->antiwindup = AXXPID_ANTIWINDUP_CONDITIONAL;
    cfg->tracking_time = 1;
    cfg->integral_band = AXXPID_UNLIMITED;
    cfg->integral_overshoot_gain = 1;
    cfg->integral_overshoot_threshold = 0;
    cfg->integral_reset_on_zero_setpoint = false;

    cfg->derivative_filter_n = 0;
    cfg->derivative_filter_tau = 0;

    cfg->setpoint_weight_b = 1;
    cfg->setpoint_weight_c = 0;

    cfg->deadband = 0;

    cfg->ff_bias = 0;
    cfg->ff_setpoint_gain = 0;
    cfg->ff_setpoint_rate_gain = 0;
    cfg->ff_fn = (axxpid_ff_fn_t)0;
    cfg->ff_user = (void *)0;

    cfg->sample_time_ms = 10u;
    cfg->update_every_call = false;
    cfg->dt_max = AXXPID_UNLIMITED;

    cfg->bumpless_tuning = false;

    return AXXPID_OK;
}

axxpid_status_t axxpid_init_config(axxpid_t *pid, const axxpid_config_t *cfg)
{
    if ((pid == (axxpid_t *)0) || (cfg == (const axxpid_config_t *)0)) {
        return AXXPID_ERR_NULL;
    }

    /* Install a safe controller unconditionally. Every other entry point
     * leaves the instance untouched when it rejects an argument, because
     * there is prior state worth protecting; here there is none, and an
     * uninitialised axxpid_t contains a garbage ff_fn that the next update
     * would happily call. A caller who ignores the return value must end up
     * with a controller that does nothing, not one running on stack litter. */
    (void)axxpid_config_default(&pid->cfg);
    axxpid_clear_state(pid);

    if (!axxpid_config_is_valid(cfg)) {
        return AXXPID_ERR_PARAM;
    }

    pid->cfg = *cfg;
    return AXXPID_OK;
}

axxpid_status_t axxpid_init(axxpid_t *pid,
                            axxpid_real_t kp,
                            axxpid_real_t ki,
                            axxpid_real_t kd,
                            axxpid_real_t out_min,
                            axxpid_real_t out_max)
{
    axxpid_config_t cfg;

    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    (void)axxpid_config_default(&cfg);
    cfg.kp = kp;
    cfg.ki = ki;
    cfg.kd = kd;
    cfg.out_min = out_min;
    cfg.out_max = out_max;

    return axxpid_init_config(pid, &cfg);
}

axxpid_status_t axxpid_reset(axxpid_t *pid)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    axxpid_clear_state(pid);
    return AXXPID_OK;
}

axxpid_status_t axxpid_reset_to(axxpid_t *pid, axxpid_real_t output)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    axxpid_clear_state(pid);

    if (axxpid_is_finite(output)) {
        output = axxpid_clamp(output, pid->cfg.out_min, pid->cfg.out_max);
        pid->manual_output = output;
        pid->output = output;
        pid->bumpless_pending = true;
    }
    return AXXPID_OK;
}

/* -------------------------------------------------------------------------- */
/* The control law                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief The control law. Returns false when the sample was rejected.
 *
 * ::axxpid_update is a thin wrapper; ::axxpid_update_at needs to know whether
 * the sample actually ran so that it does not advance its clock past an
 * interval the integrator never saw.
 */
static bool axxpid_compute(axxpid_t *pid,
                            axxpid_real_t setpoint,
                            axxpid_real_t measurement,
                            axxpid_real_t dt)
{
    const axxpid_config_t *cfg;
    bool dt_was_clamped = false;
    bool restart_history;
    bool integral_forced_zero = false;
    bool slew_limited;
    axxpid_real_t slew_step;
    axxpid_real_t direction;
    axxpid_real_t raw_error;
    axxpid_real_t error;
    axxpid_real_t p_input;
    axxpid_real_t p_term;
    axxpid_real_t d_input;
    axxpid_real_t d_raw;
    axxpid_real_t tau;
    axxpid_real_t d_term;
    axxpid_real_t ff_term;
    axxpid_real_t feedback_free;
    axxpid_real_t ki_effective;
    axxpid_real_t integral_step;
    axxpid_real_t output_unsaturated;
    axxpid_real_t output;

    cfg = &pid->cfg;

    /* A bad sample or a bad timestep must not be allowed to corrupt the
     * integrator, so hold the previous output and change nothing - except to
     * mark the derivative history stale. Carrying it across the gap would
     * make the next good sample divide a two-interval change by one
     * interval, and report a derivative twice the real one. */
    if (!axxpid_is_finite(setpoint) || !axxpid_is_finite(measurement) ||
        !axxpid_is_finite(dt) || !(dt > 0)) {
        pid->first_update = true;
        return false;
    }

    if (dt > cfg->dt_max) {
        /* The loop stalled. Bounding dt keeps the integral step sane, but the
         * measurement and setpoint have moved over the *whole* stall, so
         * dividing those changes by the bounded dt would manufacture a
         * derivative and a velocity feed-forward far larger than anything
         * that really happened. Treat the sample as a fresh start for both. */
        dt = cfg->dt_max;
        dt_was_clamped = true;
    }
    restart_history = pid->first_update || dt_was_clamped;

    direction = (cfg->acting == AXXPID_ACTING_REVERSE) ? AXXPID_C(-1)
                                                       : AXXPID_C(1);

    /* --- Error, with an optional continuous deadband ---------------------- */

    raw_error = direction * (setpoint - measurement);
    if (cfg->deadband > 0) {
        if (raw_error > cfg->deadband) {
            error = raw_error - cfg->deadband;
        } else if (raw_error < -cfg->deadband) {
            error = raw_error + cfg->deadband;
        } else {
            error = 0;
        }
    } else {
        error = raw_error;
    }

    /* --- Proportional, with setpoint weight b ----------------------------- */

    /* kp * dir * (b*sp - pv), rewritten around the (deadbanded) error so the
     * deadband applies to the proportional path as well. */
    p_input = error - (direction * (AXXPID_C(1) - cfg->setpoint_weight_b) *
                       setpoint);
    p_term = cfg->kp * p_input;

    /* --- Derivative on (c*sp - pv), optionally low-pass filtered ---------- */

    d_input = direction *
              ((cfg->setpoint_weight_c * setpoint) - measurement);

    if (restart_history) {
        /* No usable history: start the derivative at zero rather than
         * differentiating against a stale or zeroed sample. */
        d_raw = 0;
        pid->d_filtered = 0;
    } else {
        d_raw = (d_input - pid->prev_d_input) / dt;
    }

    tau = cfg->derivative_filter_tau;
    if ((tau <= 0) && (cfg->derivative_filter_n > 0) && (cfg->kp > 0) &&
        (cfg->kd > 0)) {
        /* Tf = Td / N, with Td = kd / kp. */
        tau = (cfg->kd / cfg->kp) / cfg->derivative_filter_n;
    }
    if (tau > 0) {
        /* Backward-Euler first-order low pass; alpha is recomputed from the
         * live dt so loop jitter cannot silently detune the corner. */
        const axxpid_real_t alpha = dt / (tau + dt);
        pid->d_filtered += alpha * (d_raw - pid->d_filtered);
    } else {
        pid->d_filtered = d_raw;
    }
    d_term = cfg->kd * pid->d_filtered;

    /* --- Feed-forward ----------------------------------------------------- */

    ff_term = cfg->ff_bias + (cfg->ff_setpoint_gain * setpoint);
    if ((cfg->ff_setpoint_rate_gain != 0) && !restart_history) {
        ff_term += cfg->ff_setpoint_rate_gain *
                   ((setpoint - pid->prev_setpoint) / dt);
    }
    if (cfg->ff_fn != (axxpid_ff_fn_t)0) {
        ff_term += cfg->ff_fn(setpoint, measurement, cfg->ff_user);
    }

    /* Everything that does not depend on the integrator. Feed-forward is part
     * of it on purpose: the integrator must see the same output the actuator
     * sees, or it will wind up fighting the feed-forward. */
    feedback_free = p_term + d_term + ff_term;

    /* --- Manual mode: hold the output and track the integrator ------------ */

    if (cfg->mode == AXXPID_MODE_MANUAL) {
        output = axxpid_clamp(pid->manual_output, cfg->out_min, cfg->out_max);

        /* Keep the integrator at the value that would reproduce the manual
         * output, so returning to automatic is bumpless. */
        pid->integral = axxpid_clamp(output - feedback_free,
                                      cfg->integral_min, cfg->integral_max);

        pid->error = error;
        pid->p_input = p_input;
        pid->setpoint = setpoint;
        pid->measurement = measurement;
        pid->p_term = p_term;
        pid->i_term = pid->integral;
        pid->d_term = d_term;
        pid->ff_term = ff_term;
        pid->saturated = (pid->manual_output > cfg->out_max) ||
                         (pid->manual_output < cfg->out_min);
        pid->prev_d_input = d_input;
        pid->prev_setpoint = setpoint;
        pid->output = output;
        pid->first_update = false;
        return true;
    }

    /* --- Bumpless entry into automatic ------------------------------------ */

    if (pid->bumpless_pending) {
        /* Preload from the output the actuator could actually have been
         * given, not from the raw manual figure. Preloading the integrator
         * with an unreachable value looks bumpless for one sample - the
         * output clamps to the same place either way - and then pins the
         * actuator at the limit for as long as it takes the integrator to
         * unwind the difference. */
        const axxpid_real_t resume_from =
            axxpid_clamp(pid->manual_output, cfg->out_min, cfg->out_max);

        pid->integral = axxpid_clamp(resume_from - feedback_free,
                                      cfg->integral_min, cfg->integral_max);
        pid->bumpless_pending = false;
    }

    /* --- Integrator ------------------------------------------------------- */

    /* Asymmetric integral authority: once the process has overshot, an
     * actuator that can only push in one direction needs to unwind faster
     * than it wound up. */
    slew_limited = (cfg->out_slew_rate > 0) &&
                   (cfg->out_slew_rate < AXXPID_UNLIMITED);
    slew_step = slew_limited ? (cfg->out_slew_rate * dt) : AXXPID_C(0);

    ki_effective = (error < cfg->integral_overshoot_threshold)
                       ? (cfg->ki * cfg->integral_overshoot_gain)
                       : cfg->ki;
    integral_step = ki_effective * error * dt;

    if (cfg->antiwindup == AXXPID_ANTIWINDUP_CONDITIONAL) {
        /* Conditional integration: refuse the step only when the output would
         * be outside what the actuator can reach AND the step pushes it
         * further out. A step that brings the output back towards the usable
         * range is always allowed, so the integrator can never latch.
         *
         * The test is on which bound is being exceeded, not on the sign of
         * the output. Those two agree only when the limits straddle zero; for
         * a valve that lives between 20% and 80% the sign tells you nothing.
         *
         * "What the actuator can reach" includes the slew rate. A rate limit
         * holds the output away from the control law's demand just as firmly
         * as a hard limit does, so ignoring it here would let the integrator
         * wind up freely through every rate-limited ramp. */
        axxpid_real_t reachable_min = cfg->out_min;
        axxpid_real_t reachable_max = cfg->out_max;
        axxpid_real_t candidate;
        bool pushing_over_max;
        bool pushing_under_min;

        if (slew_limited) {
            const axxpid_real_t slew_lo = pid->output - slew_step;
            const axxpid_real_t slew_hi = pid->output + slew_step;

            if (slew_lo > reachable_min) {
                reachable_min = slew_lo;
            }
            if (slew_hi < reachable_max) {
                reachable_max = slew_hi;
            }
        }

        candidate = feedback_free + pid->integral + integral_step;
        pushing_over_max = (candidate > reachable_max) && (integral_step > 0);
        pushing_under_min = (candidate < reachable_min) && (integral_step < 0);

        if (!pushing_over_max && !pushing_under_min) {
            pid->integral += integral_step;
        }
    } else {
        pid->integral += integral_step;
    }

    pid->integral = axxpid_clamp(pid->integral, cfg->integral_min,
                                  cfg->integral_max);

    if (cfg->integral_reset_on_zero_setpoint && (setpoint == 0)) {
        pid->integral = 0;
        integral_forced_zero = true;
    }

    /* Integral engagement band: park the integrator while the process is
     * still far below the setpoint and the actuator is saturated anyway. */
    if (error > cfg->integral_band) {
        pid->integral = 0;
        integral_forced_zero = true;
    }

    /* --- Sum, clamp, slew-limit ------------------------------------------- */

    output_unsaturated = feedback_free + pid->integral;
    output = axxpid_clamp(output_unsaturated, cfg->out_min, cfg->out_max);

    if (slew_limited) {
        output = axxpid_clamp(output, pid->output - slew_step,
                               pid->output + slew_step);
    }

    /* Record the split that produced this output before back-calculation
     * adjusts the integrator for the next sample, so that
     * p_term + i_term + d_term + ff_term is exactly the pre-clamp output. */
    pid->error = error;
    pid->p_input = p_input;
    pid->setpoint = setpoint;
    pid->measurement = measurement;
    pid->p_term = p_term;
    pid->i_term = pid->integral;
    pid->d_term = d_term;
    pid->ff_term = ff_term;
    pid->saturated = (output != output_unsaturated);

    /* --- Back-calculation ------------------------------------------------- */

    if ((cfg->antiwindup == AXXPID_ANTIWINDUP_BACK_CALCULATION) &&
        !integral_forced_zero) {
        /* Bleed the integrator towards whatever the actuator actually got.
         * Using the post-slew output means a rate limit unwinds the
         * integrator too, not just a hard output limit.
         *
         * The gain is capped at 1. dt is whatever the caller measured, so a
         * late sample can make dt/Tt exceed 1, at which point the correction
         * overshoots its own target: above 1 the integrator rings sample to
         * sample, and above 2 it diverges. A gain of exactly 1 is the
         * sensible limit - it lands on the tracking target in one step.
         *
         * Skipped entirely when the engagement band or the zero-setpoint rule
         * has just forced the integrator to zero, so that "held at zero"
         * means held at zero rather than "zero, then immediately nudged". */
        axxpid_real_t tracking_gain = dt / cfg->tracking_time;

        if (tracking_gain > 1) {
            tracking_gain = 1;
        }
        pid->integral += tracking_gain * (output - output_unsaturated);
        pid->integral = axxpid_clamp(pid->integral, cfg->integral_min,
                                      cfg->integral_max);
    }

    pid->prev_d_input = d_input;
    pid->prev_setpoint = setpoint;
    pid->output = output;
    pid->first_update = false;

    return true;
}

axxpid_real_t axxpid_update(axxpid_t *pid,
                            axxpid_real_t setpoint,
                            axxpid_real_t measurement,
                            axxpid_real_t dt)
{
    if (pid == (axxpid_t *)0) {
        return 0;
    }

    /* Any millisecond clock this instance was using is now stale, because
     * time has passed that axxpid_update_at did not account for. Dropping the
     * timestamp makes the next axxpid_update_at resynchronise from its
     * nominal period instead of integrating the whole gap a second time. */
    pid->has_time = false;

    (void)axxpid_compute(pid, setpoint, measurement, dt);
    return pid->output;
}

bool axxpid_update_at(axxpid_t *pid,
                      axxpid_real_t setpoint,
                      axxpid_real_t measurement,
                      uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (pid == (axxpid_t *)0) {
        return false;
    }

    if (!pid->has_time) {
        /* Backdate by one period so the very first call produces an update
         * with a sensible dt instead of silently skipping a sample. */
        pid->last_time_ms = now_ms - pid->cfg.sample_time_ms;
        pid->has_time = true;
    }

    /* Unsigned arithmetic: correct across the 49.7-day wrap of a 32-bit
     * millisecond counter. */
    elapsed_ms = now_ms - pid->last_time_ms;

    if (!pid->cfg.update_every_call && (elapsed_ms < pid->cfg.sample_time_ms)) {
        return false;
    }
    if (elapsed_ms == 0u) {
        return false;
    }

    if (!axxpid_compute(pid, setpoint, measurement,
                         (axxpid_real_t)elapsed_ms / AXXPID_C(1000))) {
        /* The sample was rejected. Leaving the clock where it is means the
         * skipped interval is rolled into the next good sample rather than
         * being quietly dropped from the integrator. */
        return false;
    }

    pid->last_time_ms = now_ms;
    pid->has_time = true;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Tuning                                                                     */
/* -------------------------------------------------------------------------- */

axxpid_status_t axxpid_set_tunings(axxpid_t *pid,
                                   axxpid_real_t kp,
                                   axxpid_real_t ki,
                                   axxpid_real_t kd)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((kp < 0) || (ki < 0) || (kd < 0) || !axxpid_is_finite(kp) ||
        !axxpid_is_finite(ki) || !axxpid_is_finite(kd)) {
        return AXXPID_ERR_PARAM;
    }

    if (pid->cfg.bumpless_tuning && !pid->first_update) {
        /* Shift the integrator by exactly as much as the P and D terms are
         * about to change, so the summed output does not step. The integral
         * gain needs no compensation: the integral is stored in output units,
         * so changing ki only affects future accumulation. */
        const axxpid_real_t new_p = kp * pid->p_input;
        const axxpid_real_t new_d = kd * pid->d_filtered;
        pid->integral = axxpid_clamp(pid->integral + pid->p_term +
                                          pid->d_term - new_p - new_d,
                                      pid->cfg.integral_min,
                                      pid->cfg.integral_max);
        pid->p_term = new_p;
        pid->d_term = new_d;
        pid->i_term = pid->integral;
    }

    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_tunings_standard(axxpid_t *pid,
                                            axxpid_real_t kp,
                                            axxpid_real_t ti,
                                            axxpid_real_t td)
{
    axxpid_real_t ki;

    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((kp < 0) || (ti < 0) || (td < 0)) {
        return AXXPID_ERR_PARAM;
    }

    /* ti == 0 is the documented way to ask for no integral action; dividing
     * by it would be undefined. */
    ki = (ti > 0) ? (kp / ti) : AXXPID_C(0);
    return axxpid_set_tunings(pid, kp, ki, kp * td);
}

axxpid_status_t axxpid_set_kp(axxpid_t *pid, axxpid_real_t kp)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    return axxpid_set_tunings(pid, kp, pid->cfg.ki, pid->cfg.kd);
}

axxpid_status_t axxpid_set_ki(axxpid_t *pid, axxpid_real_t ki)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    return axxpid_set_tunings(pid, pid->cfg.kp, ki, pid->cfg.kd);
}

axxpid_status_t axxpid_set_kd(axxpid_t *pid, axxpid_real_t kd)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    return axxpid_set_tunings(pid, pid->cfg.kp, pid->cfg.ki, kd);
}

/* -------------------------------------------------------------------------- */
/* Configuration setters                                                      */
/* -------------------------------------------------------------------------- */

axxpid_status_t axxpid_set_output_limits(axxpid_t *pid,
                                         axxpid_real_t min,
                                         axxpid_real_t max)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (!(min < max) || !axxpid_is_finite(min) || !axxpid_is_finite(max)) {
        return AXXPID_ERR_PARAM;
    }

    pid->cfg.out_min = min;
    pid->cfg.out_max = max;

    /* Bring the live state inside the new range immediately. */
    pid->output = axxpid_clamp(pid->output, min, max);
    pid->manual_output = axxpid_clamp(pid->manual_output, min, max);
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_output_slew_rate(axxpid_t *pid, axxpid_real_t rate)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((rate < 0) || !axxpid_is_finite(rate)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.out_slew_rate = rate;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_integral(axxpid_t *pid, axxpid_real_t value)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (!axxpid_is_finite(value)) {
        return AXXPID_ERR_PARAM;
    }
    pid->integral = axxpid_clamp(value, pid->cfg.integral_min,
                                  pid->cfg.integral_max);
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_integral_limits(axxpid_t *pid,
                                           axxpid_real_t min,
                                           axxpid_real_t max)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((min > max) || !axxpid_is_finite(min) || !axxpid_is_finite(max)) {
        return AXXPID_ERR_PARAM;
    }

    pid->cfg.integral_min = min;
    pid->cfg.integral_max = max;
    pid->integral = axxpid_clamp(pid->integral, min, max);
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_antiwindup(axxpid_t *pid,
                                      axxpid_antiwindup_t mode,
                                      axxpid_real_t tracking_time)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    switch (mode) {
        case AXXPID_ANTIWINDUP_NONE:
        case AXXPID_ANTIWINDUP_CONDITIONAL:
            break;
        case AXXPID_ANTIWINDUP_BACK_CALCULATION:
            if (!(tracking_time > 0) || !axxpid_is_finite(tracking_time)) {
                return AXXPID_ERR_PARAM;
            }
            break;
        default:
            return AXXPID_ERR_PARAM;
    }

    pid->cfg.antiwindup = mode;
    if (tracking_time > 0) {
        pid->cfg.tracking_time = tracking_time;
    }
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_integral_band(axxpid_t *pid, axxpid_real_t band)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((band < 0) || !axxpid_is_finite(band)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.integral_band = band;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_integral_overshoot(axxpid_t *pid,
                                              axxpid_real_t gain,
                                              axxpid_real_t threshold)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((gain < 0) || !axxpid_is_finite(gain) || !axxpid_is_finite(threshold)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.integral_overshoot_gain = gain;
    pid->cfg.integral_overshoot_threshold = threshold;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_integral_reset_on_zero_setpoint(axxpid_t *pid,
                                                           bool enable)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    pid->cfg.integral_reset_on_zero_setpoint = enable;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_derivative_filter(axxpid_t *pid, axxpid_real_t n)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((n < 0) || !axxpid_is_finite(n)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.derivative_filter_n = n;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_derivative_filter_tau(axxpid_t *pid,
                                                 axxpid_real_t tau)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((tau < 0) || !axxpid_is_finite(tau)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.derivative_filter_tau = tau;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_setpoint_weights(axxpid_t *pid,
                                            axxpid_real_t b,
                                            axxpid_real_t c)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((b < 0) || (b > 1) || (c < 0) || (c > 1)) {
        return AXXPID_ERR_PARAM;
    }

    if (c != pid->cfg.setpoint_weight_c) {
        /* The stored derivative history was measured with the old weight, so
         * differencing across the change would report a step in the setpoint
         * as a one-sample spike in the rate of the measurement. Start the
         * derivative again instead. */
        pid->first_update = true;
    }

    pid->cfg.setpoint_weight_b = b;
    pid->cfg.setpoint_weight_c = c;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_deadband(axxpid_t *pid, axxpid_real_t deadband)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((deadband < 0) || !axxpid_is_finite(deadband)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.deadband = deadband;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_feedforward_bias(axxpid_t *pid, axxpid_real_t bias)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (!axxpid_is_finite(bias)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.ff_bias = bias;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_feedforward_gains(axxpid_t *pid,
                                             axxpid_real_t setpoint_gain,
                                             axxpid_real_t rate_gain)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (!axxpid_is_finite(setpoint_gain) || !axxpid_is_finite(rate_gain)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.ff_setpoint_gain = setpoint_gain;
    pid->cfg.ff_setpoint_rate_gain = rate_gain;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_feedforward_fn(axxpid_t *pid,
                                          axxpid_ff_fn_t fn,
                                          void *user)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    pid->cfg.ff_fn = fn;
    pid->cfg.ff_user = user;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_acting(axxpid_t *pid, axxpid_acting_t acting)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((acting != AXXPID_ACTING_DIRECT) && (acting != AXXPID_ACTING_REVERSE)) {
        return AXXPID_ERR_PARAM;
    }

    if (acting != pid->cfg.acting) {
        /* The accumulated integral belongs to the old sense; keeping it would
         * drive the actuator hard the wrong way. */
        pid->integral = 0;
        pid->d_filtered = 0;
        pid->first_update = true;
    }
    pid->cfg.acting = acting;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_mode(axxpid_t *pid, axxpid_mode_t mode)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if ((mode != AXXPID_MODE_MANUAL) && (mode != AXXPID_MODE_AUTOMATIC)) {
        return AXXPID_ERR_PARAM;
    }

    if ((mode == AXXPID_MODE_AUTOMATIC) &&
        (pid->cfg.mode == AXXPID_MODE_MANUAL)) {
        /* Preload the integrator on the next update so the closed loop picks
         * up exactly where the manual output left off. */
        pid->bumpless_pending = true;
    }
    if (mode == AXXPID_MODE_MANUAL) {
        pid->manual_output = pid->output;
    }
    pid->cfg.mode = mode;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_manual_output(axxpid_t *pid, axxpid_real_t output)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (!axxpid_is_finite(output)) {
        return AXXPID_ERR_PARAM;
    }
    pid->manual_output = output;
    if (pid->cfg.mode == AXXPID_MODE_MANUAL) {
        pid->output = axxpid_clamp(output, pid->cfg.out_min, pid->cfg.out_max);
    }
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_sample_time(axxpid_t *pid,
                                       uint32_t sample_time_ms,
                                       bool update_every_call)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (sample_time_ms == 0u) {
        return AXXPID_ERR_PARAM;
    }

    /* No gain rescaling: axxpid_update integrates and differentiates against
     * the measured elapsed time, so ki and kd are already sample-rate
     * independent. */
    pid->cfg.sample_time_ms = sample_time_ms;
    pid->cfg.update_every_call = update_every_call;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_dt_max(axxpid_t *pid, axxpid_real_t dt_max)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (!(dt_max > 0) || !axxpid_is_finite(dt_max)) {
        return AXXPID_ERR_PARAM;
    }
    pid->cfg.dt_max = dt_max;
    return AXXPID_OK;
}

axxpid_status_t axxpid_set_bumpless_tuning(axxpid_t *pid, bool enable)
{
    if (pid == (axxpid_t *)0) {
        return AXXPID_ERR_NULL;
    }
    pid->cfg.bumpless_tuning = enable;
    return AXXPID_OK;
}

/* -------------------------------------------------------------------------- */
/* Getters                                                                    */
/* -------------------------------------------------------------------------- */

axxpid_real_t axxpid_get_output(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->output : AXXPID_C(0);
}

axxpid_real_t axxpid_get_error(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->error : AXXPID_C(0);
}

axxpid_real_t axxpid_get_p_term(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->p_term : AXXPID_C(0);
}

axxpid_real_t axxpid_get_i_term(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->i_term : AXXPID_C(0);
}

axxpid_real_t axxpid_get_integral(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->integral : AXXPID_C(0);
}

axxpid_real_t axxpid_get_d_term(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->d_term : AXXPID_C(0);
}

axxpid_real_t axxpid_get_ff_term(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->ff_term : AXXPID_C(0);
}

axxpid_status_t axxpid_get_terms(const axxpid_t *pid, axxpid_terms_t *terms)
{
    if ((pid == (const axxpid_t *)0) || (terms == (axxpid_terms_t *)0)) {
        return AXXPID_ERR_NULL;
    }
    terms->p = pid->p_term;
    terms->i = pid->i_term;
    terms->d = pid->d_term;
    terms->ff = pid->ff_term;
    terms->output = pid->output;
    terms->error = pid->error;
    terms->setpoint = pid->setpoint;
    terms->measurement = pid->measurement;
    return AXXPID_OK;
}

bool axxpid_is_saturated(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->saturated : false;
}

axxpid_real_t axxpid_get_kp(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->cfg.kp : AXXPID_C(0);
}

axxpid_real_t axxpid_get_ki(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->cfg.ki : AXXPID_C(0);
}

axxpid_real_t axxpid_get_kd(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->cfg.kd : AXXPID_C(0);
}

axxpid_mode_t axxpid_get_mode(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->cfg.mode : AXXPID_MODE_MANUAL;
}

axxpid_acting_t axxpid_get_acting(const axxpid_t *pid)
{
    return (pid != (const axxpid_t *)0) ? pid->cfg.acting
                                        : AXXPID_ACTING_DIRECT;
}

const char *axxpid_version(void)
{
    return AXXPID_VERSION_STRING;
}
