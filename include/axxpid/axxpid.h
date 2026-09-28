/**
 * @file    axxpid.h
 * @brief   AxxPID - a portable, dependency-free PID controller for C.
 * @version 1.0.0
 * @license MIT
 *
 * AxxPID is a single-translation-unit PID controller extracted from the
 * AxxSolder soldering station firmware and generalised for reuse. It has no
 * dependency on any vendor HAL, performs no dynamic allocation, and uses no
 * global state: every controller lives in a caller-allocated ::axxpid_t.
 *
 * Quick start:
 * @code
 *   axxpid_t pid;
 *
 *   //          kp    ki    kd    out_min  out_max
 *   axxpid_init(&pid, 8.0f, 2.0f, 0.5f,    0.0f, 100.0f);
 *
 *   for (;;) {
 *       // 0.010f is the seconds elapsed since the previous call.
 *       float u = axxpid_update(&pid, setpoint, read_sensor(), 0.010f);
 *       drive_actuator(u);
 *   }
 * @endcode
 *
 * Three names used throughout this file:
 *
 *   - **setpoint** (@c sp) - the value you want.
 *   - **measurement** (@c pv, "process variable") - the value your sensor
 *     reads.
 *   - **output** (@c u) - what you send to the actuator: the heater, motor or
 *     valve you are driving.
 *
 * The control law is the parallel form of PID - each gain acts on its own
 * term - with setpoint weighting and feed-forward added:
 *
 * @verbatim
 *   error = sp - pv
 *   P     = kp * (b*sp - pv)                          b weights the setpoint
 *   I     = running sum of ki * error * dt            stored in output units
 *   D     = kd * d/dt[ c*sp - pv ], low-pass filtered c weights the setpoint
 *   FF    = bias + kf*sp + kf_rate*d(sp)/dt + ff_fn()
 *   u     = clamp(P + I + D + FF, out_min, out_max)
 * @endverbatim
 *
 * The defaults are @c b = 1 and @c c = 0, which give the usual arrangement:
 * P acts on the error, D acts on the measurement. D on the measurement is
 * what stops the output spiking every time you change the setpoint.
 *
 * Where to look:
 *
 *   - Getting started: ::axxpid_init, ::axxpid_update, ::axxpid_update_at
 *   - Gains: ::axxpid_set_tunings, ::axxpid_set_tunings_standard
 *   - Stopping the integral running away: ::axxpid_set_antiwindup,
 *     ::axxpid_set_integral_limits
 *   - Noisy sensor: ::axxpid_set_derivative_filter
 *   - Getting ahead of the error: ::axxpid_set_feedforward_gains
 *   - Hand control: ::axxpid_set_mode, ::axxpid_set_manual_output
 *   - Logging and tuning: ::axxpid_get_terms
 */

#ifndef AXXPID_H
#define AXXPID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Version                                                                    */
/* -------------------------------------------------------------------------- */

#define AXXPID_VERSION_MAJOR  1
#define AXXPID_VERSION_MINOR  0
#define AXXPID_VERSION_PATCH  0
#define AXXPID_VERSION_STRING "1.0.0"

/* -------------------------------------------------------------------------- */
/* Scalar type                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compile with @c -DAXXPID_USE_DOUBLE=1 to run the controller in
 *        double precision. The default is @c float, which is the right choice
 *        on any MCU with a single-precision FPU (Cortex-M4F/M7/M33).
 */
#ifndef AXXPID_USE_DOUBLE
#define AXXPID_USE_DOUBLE 0
#endif

#if AXXPID_USE_DOUBLE
typedef double axxpid_real_t;
#else
typedef float axxpid_real_t;
#endif

/** @brief Cast an integer or floating literal to ::axxpid_real_t. */
#define AXXPID_C(x) ((axxpid_real_t)(x))

/**
 * @brief Sentinel used for "no limit".
 *
 * A finite value rather than @c INFINITY, so that no arithmetic inside the
 * controller can ever produce an infinity or a NaN from a limit alone.
 * It is comfortably larger than any physical actuator range.
 */
#ifndef AXXPID_UNLIMITED
#define AXXPID_UNLIMITED AXXPID_C(1e30)
#endif

/* -------------------------------------------------------------------------- */
/* Enumerations                                                               */
/* -------------------------------------------------------------------------- */

/** @brief Return code for the configuring entry points. */
typedef enum {
    AXXPID_OK = 0,      /**< Success. */
    AXXPID_ERR_NULL,    /**< A required pointer argument was NULL. */
    AXXPID_ERR_PARAM    /**< An argument was out of range; nothing was changed. */
} axxpid_status_t;

/**
 * @brief Which way the actuator moves the process.
 *
 * @c AXXPID_ACTING_DIRECT: raising the output raises the measurement
 * (a heater, a motor drive). @c AXXPID_ACTING_REVERSE: raising the output
 * lowers the measurement (a cooler, a drain valve).
 *
 * Reverse action negates the control error internally. The gains always stay
 * positive, so ::axxpid_get_kp and friends return exactly what you set.
 */
typedef enum {
    AXXPID_ACTING_DIRECT = 0,
    AXXPID_ACTING_REVERSE = 1
} axxpid_acting_t;

/**
 * @brief Automatic (closed-loop) or manual (open-loop) operation.
 *
 * In manual mode ::axxpid_update returns the value set with
 * ::axxpid_set_manual_output, and keeps the integral matched to it, so
 * switching back to automatic does not jump the output. (That is a direct
 * assignment, not the back-calculation anti-windup below; it happens in every
 * anti-windup mode.)
 */
typedef enum {
    AXXPID_MODE_MANUAL = 0,
    AXXPID_MODE_AUTOMATIC = 1
} axxpid_mode_t;

/**
 * @brief What to do about integrator windup.
 *
 * Windup: while the output is pinned at @c out_min or @c out_max - saturated
 * - the error stays large and the integral keeps growing, even though the
 * actuator cannot do any more about it. When the error finally reverses, all
 * of that surplus has to be unwound before the output moves, so the process
 * sails past the setpoint and stays there for a while.
 */
typedef enum {
    /** No protection beyond the explicit integral limits. */
    AXXPID_ANTIWINDUP_NONE = 0,

    /**
     * Conditional integration ("clamping"): integration is frozen for a
     * sample whenever integrating further would push an already-saturated
     * output deeper into its limit. This is the AxxSolder-proven default:
     * cheap, branch-only, and it never overshoots the limit at all.
     */
    AXXPID_ANTIWINDUP_CONDITIONAL = 1,

    /**
     * Back-calculation, also called tracking. Each sample the integral is
     * pulled a little way towards the value that would have produced the
     * output the actuator actually got:
     *
     *     I += (dt / tracking_time) * (limited_output - wanted_output)
     *
     * Smoother coming off a limit than freezing, and it accounts for the
     * feed-forward and the slew rate too, because the final output is what is
     * fed back. Start with @c tracking_time equal to the integral time
     * @f$T_i = k_p/k_i@f$, or @f$\sqrt{T_i T_d}@f$ for a full PID.
     */
    AXXPID_ANTIWINDUP_BACK_CALCULATION = 2
} axxpid_antiwindup_t;

/** @brief Signature of a user feed-forward function. */
typedef axxpid_real_t (*axxpid_ff_fn_t)(axxpid_real_t setpoint,
                                        axxpid_real_t measurement,
                                        void *user);

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Everything that describes a controller's behaviour.
 *
 * Fill one with ::axxpid_config_default, adjust the handful of fields you
 * care about, then hand it to ::axxpid_init_config. Every field also has a
 * dedicated setter that is safe to call at run time: think of the struct as
 * the initial state and the setters as the way to change it afterwards.
 *
 * Two conventions for switching a feature off. Where zero is not a meaningful
 * setting - @c deadband, @c out_slew_rate, @c derivative_filter_n,
 * @c derivative_filter_tau - zero disables the feature. Where zero *is*
 * meaningful, because it is a legitimate point on a scale of limits -
 * @c integral_band, @c dt_max, the output and integral limits -
 * ::AXXPID_UNLIMITED disables it.
 */
typedef struct {
    /* --- Gains, parallel form --------------------------------------------- */

    axxpid_real_t kp; /**< Proportional gain, output units per error unit. */
    axxpid_real_t ki; /**< Integral gain, output units per (error unit * second). */
    axxpid_real_t kd; /**< Derivative gain, output units per (error unit / second). */

    axxpid_acting_t acting; /**< Actuator sense. Default ::AXXPID_ACTING_DIRECT. */
    axxpid_mode_t mode;     /**< Start mode. Default ::AXXPID_MODE_AUTOMATIC. */

    /* --- Output ----------------------------------------------------------- */

    axxpid_real_t out_min; /**< Lower actuator limit. Default -::AXXPID_UNLIMITED. */
    axxpid_real_t out_max; /**< Upper actuator limit. Default +::AXXPID_UNLIMITED. */

    /**
     * Slew rate: the maximum the output may change per second, in output
     * units. Set to 0 to disable. Applied after the output clamp.
     *
     * Both anti-windup strategies account for it: a rate limit holds the
     * output away from what the control law asked for just as firmly as a
     * hard limit does, so the integrator is held back in conditional mode and
     * bled in back-calculation mode.
     */
    axxpid_real_t out_slew_rate;

    /* --- Integrator ------------------------------------------------------- */

    axxpid_real_t integral_min; /**< Lower clamp on the integral term. */
    axxpid_real_t integral_max; /**< Upper clamp on the integral term. */

    axxpid_antiwindup_t antiwindup; /**< Default ::AXXPID_ANTIWINDUP_CONDITIONAL. */

    /** Tracking time constant @f$T_t@f$ in seconds, back-calculation only. */
    axxpid_real_t tracking_time;

    /**
     * Integral engagement band, in error units. While @c error is greater
     * than @c integral_band the integrator is held at zero. For a
     * direct-acting loop that means "while the measurement is more than
     * @c integral_band below the setpoint"; for a reverse-acting one the
     * error is negated first, so it means the other way round.
     *
     * This is the AxxSolder "I min error". During a long ramp the actuator is
     * flat out anyway, so anything the integral collects there will only come
     * back as overshoot. Holding it at zero until the error is small avoids
     * that.
     *
     * Deliberately one-sided. A symmetric version would also dump the integral
     * when the process is far *above* setpoint, which is exactly when a heater
     * - which can heat but not cool - needs the negative integral it built
     * up.
     *
     * Default ::AXXPID_UNLIMITED (disabled).
     */
    axxpid_real_t integral_band;

    /**
     * Extra multiplier applied to @c ki while @c error < @c
     * integral_overshoot_threshold, i.e. while the measurement has overshot.
     *
     * A soldering iron heats fast but cools only as fast as it loses heat to
     * the air. Once it is above the setpoint the integral has to drain several
     * times faster than it filled, or the overshoot lasts. AxxSolder uses
     * 7.0.
     *
     * Default 1.0 (symmetric, i.e. disabled).
     */
    axxpid_real_t integral_overshoot_gain;

    /**
     * Error below which @c integral_overshoot_gain takes effect. AxxSolder uses
     * -1.0, so a small leftover error does not trip the fast-drain path and
     * set the loop hunting - slowly oscillating around the setpoint. Default
     * 0.0.
     */
    axxpid_real_t integral_overshoot_threshold;

    /**
     * When true, the integrator is forced to zero whenever the setpoint is
     * exactly zero. Handy when a setpoint of zero means "off" and you want
     * the controller to come back up from a clean state. Default false.
     */
    bool integral_reset_on_zero_setpoint;

    /* --- Derivative ------------------------------------------------------- */

    /**
     * Derivative filter divisor @f$N@f$ (Astrom & Hagglund). The derivative
     * is low-pass filtered with @f$T_f = T_d/N = (k_d/k_p)/N@f$, which bounds
     * the high-frequency gain of the D term at @f$k_p N@f$ instead of letting
     * it grow without limit. Sensible values are 2..20; 8..16 is typical.
     *
     * Default 0.0 (no filtering), which reproduces the raw derivative used by
     * AxxSolder. Enable it if your measurement is noisy - see the README.
     */
    axxpid_real_t derivative_filter_n;

    /**
     * Explicit derivative filter time constant in seconds. When greater than
     * zero this overrides @c derivative_filter_n, which is what you want if
     * you retune @c kp and @c kd at run time but the sensor noise bandwidth
     * stays put. Default 0.0.
     */
    axxpid_real_t derivative_filter_tau;

    /* --- Setpoint weighting (2-DOF) --------------------------------------- */

    /**
     * Proportional setpoint weight @f$b@f$. The P term acts on a scaled copy
     * of the setpoint, @f$b \cdot sp - pv@f$, which lets you soften the
     * response to a setpoint change without touching how the loop rejects
     * disturbances. (That independence is what "two degrees of freedom"
     * means.) @c b = 1 is proportional-on-error (default);
     * @c b = 0 is proportional-on-measurement, which removes the proportional
     * step on a setpoint change; values in between trade tracking speed for
     * overshoot without touching disturbance rejection.
     */
    axxpid_real_t setpoint_weight_b;

    /**
     * Derivative setpoint weight @f$c@f$: the D term acts on
     * @f$c \cdot sp - pv@f$. @c c = 0 is derivative-on-measurement (default,
     * and almost always what you want); @c c = 1 is derivative-on-error and
     * reintroduces derivative kick.
     */
    axxpid_real_t setpoint_weight_c;

    /* --- Error shaping ---------------------------------------------------- */

    /**
     * Error deadband, in error units. Errors smaller in magnitude than this
     * leave the controller acting as though the measurement were exactly on
     * setpoint; larger errors are shifted towards zero by the deadband width
     * so that the control signal stays continuous. Useful to
     * stop sensor noise driving a mechanical actuator back and forth.
     * The D term keeps acting on the raw measurement so that real
     * disturbances are still caught. Default 0.0 (disabled).
     */
    axxpid_real_t deadband;

    /* --- Feed-forward ----------------------------------------------------- */

    /** Constant feed-forward, in output units. Default 0.0. */
    axxpid_real_t ff_bias;

    /** Feed-forward proportional to the setpoint: @c ff += ff_setpoint_gain * sp. */
    axxpid_real_t ff_setpoint_gain;

    /**
     * Velocity feed-forward: @c ff += ff_setpoint_rate_gain * d(sp)/dt.
     * Pushes the actuator during a setpoint ramp instead of waiting for an
     * error to build up. Skipped on the first update after a reset so that a
     * cold start cannot produce a spike.
     */
    axxpid_real_t ff_setpoint_rate_gain;

    /**
     * Arbitrary feed-forward hook, called once per control update with the
     * current setpoint and measurement. Use it for a gain-scheduled term, a
     * lookup table, a measured-disturbance term, or a plant inverse.
     * Default NULL.
     */
    axxpid_ff_fn_t ff_fn;

    /** Opaque pointer handed to ::axxpid_config_t::ff_fn. Default NULL. */
    void *ff_user;

    /* --- Sampling --------------------------------------------------------- */

    /**
     * Nominal control period in milliseconds, used only by
     * ::axxpid_update_at. Default 10.
     */
    uint32_t sample_time_ms;

    /**
     * When true, ::axxpid_update_at recomputes on every call using the real
     * elapsed time instead of waiting for @c sample_time_ms. Default false.
     */
    bool update_every_call;

    /**
     * Upper bound on the @c dt accepted by ::axxpid_update, in seconds.
     * Protects the integrator if the control loop is stalled by a long
     * blocking operation. Default ::AXXPID_UNLIMITED (disabled); prefer
     * calling ::axxpid_reset after a deliberate pause.
     *
     * When a sample is capped, the derivative and the velocity feed-forward
     * are restarted rather than computed: the measurement and setpoint moved
     * over the whole stall, and dividing that change by the capped @c dt
     * would invent a rate of change that never happened.
     */
    axxpid_real_t dt_max;

    /* --- Behaviour flags --------------------------------------------------- */

    /**
     * When true, ::axxpid_set_tunings and friends shift the integrator so the
     * output does not step when @c kp or @c kd change mid-flight. Default
     * false. (Changing @c ki never causes a step: the integral is accumulated
     * in output units, not as a raw error sum scaled at read time.)
     */
    bool bumpless_tuning;
} axxpid_config_t;

/* -------------------------------------------------------------------------- */
/* Controller state                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief A PID controller instance.
 *
 * The struct is transparent so that you can allocate one statically, on the
 * stack, or inside another struct - there is no hidden allocation anywhere in
 * AxxPID. Treat the fields as read-only: use the setters to change
 * configuration and the getters to inspect state.
 */
typedef struct {
    /**
     * Active configuration. Reading it is supported and is the intended way
     * to query settings - @c pid.cfg.out_max and friends - so there is no
     * getter for every field. Write to it only through the setters, which
     * validate and carry the state side effects that matter (clearing the
     * integrator when the actuator sense changes, clamping the live output
     * when the limits tighten, and so on).
     */
    axxpid_config_t cfg;

    /* Integrator and filter state. */
    axxpid_real_t integral;      /**< Integral term, in output units. */
    axxpid_real_t d_filtered;    /**< Filtered derivative of the D input. */
    axxpid_real_t prev_d_input;  /**< Previous value of (c*sp - pv), sense-corrected. */
    axxpid_real_t prev_setpoint; /**< Previous setpoint, for velocity feed-forward. */
    axxpid_real_t p_input;       /**< Latest (b*sp - pv), sense-corrected. */
    axxpid_real_t setpoint;      /**< Setpoint of the last update. */
    axxpid_real_t measurement;   /**< Measurement of the last update. */

    /* Latest results, for logging and introspection. */
    axxpid_real_t output;        /**< Latest output actually returned. */
    axxpid_real_t manual_output; /**< Output used while in manual mode. */
    axxpid_real_t error;         /**< Latest control error, after deadband. */
    axxpid_real_t p_term;        /**< Proportional contribution to ::output. */
    axxpid_real_t i_term;        /**< Integral contribution to ::output. */
    axxpid_real_t d_term;        /**< Derivative contribution to ::output. */
    axxpid_real_t ff_term;       /**< Feed-forward contribution to ::output. */
    bool saturated;              /**< True if the last output was limited. */

    /* Housekeeping. */
    bool first_update;     /**< True until the first update after a reset. */
    bool bumpless_pending; /**< Preload the integrator on the next update. */
    bool has_time;         /**< ::axxpid_update_at has seen a timestamp. */
    uint32_t last_time_ms; /**< Timestamp of the last ::axxpid_update_at run. */
} axxpid_t;

/**
 * @brief Everything about the most recent update, for one telemetry frame.
 *
 * @c p + @c i + @c d + @c ff is the output the control law asked for, before
 * the output limits and the slew rate had their say.
 */
typedef struct {
    axxpid_real_t p;           /**< Proportional contribution. */
    axxpid_real_t i;           /**< Integral contribution. */
    axxpid_real_t d;           /**< Derivative contribution. */
    axxpid_real_t ff;          /**< Feed-forward contribution. */
    axxpid_real_t output;      /**< The value ::axxpid_update returned. */
    axxpid_real_t error;       /**< Control error, after the deadband. */
    axxpid_real_t setpoint;    /**< Setpoint of the last update. */
    axxpid_real_t measurement; /**< Measurement of the last update. */
} axxpid_terms_t;

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Fill @p cfg with the default configuration.
 *
 * Defaults: zero gains, direct acting, automatic mode, no output or integral
 * limits, conditional-integration anti-windup, unfiltered derivative on
 * measurement, proportional on error, no deadband, no feed-forward, 10 ms
 * sample time.
 *
 * @param cfg Configuration to initialise. Must not be NULL.
 * @return ::AXXPID_OK, or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_config_default(axxpid_config_t *cfg);

/**
 * @brief Initialise a controller: three gains and the actuator range.
 *
 * This is the one call most loops ever need. Everything else takes a sensible
 * default: direct acting, automatic mode, proportional on error, derivative
 * on measurement, conditional-integration anti-windup, no feed-forward.
 *
 * The output limits are arguments rather than a later setter because a
 * controller that does not know what its actuator can do cannot protect its
 * integrator, and forgetting to set them is the single easiest way to get a
 * badly behaved loop. Pass -::AXXPID_UNLIMITED and ::AXXPID_UNLIMITED if you
 * genuinely want an unbounded output.
 *
 * @param pid     Controller to initialise. Must not be NULL.
 * @param kp      Proportional gain, >= 0.
 * @param ki      Integral gain, >= 0.
 * @param kd      Derivative gain, >= 0.
 * @param out_min Lower actuator limit.
 * @param out_max Upper actuator limit, strictly greater than @p out_min.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM for a
 *         negative or non-finite gain (use ::AXXPID_ACTING_REVERSE for a
 *         reverse-acting process rather than negating the gains) or an
 *         inverted output range.
 *
 * @note Unlike the setters, this call never leaves the controller untouched.
 *       Even when it rejects an argument, @p pid is left fully initialised
 *       with the default configuration and zero gains, so a caller that
 *       ignores the return value gets a controller that does nothing rather
 *       than one running on whatever happened to be in memory.
 */
axxpid_status_t axxpid_init(axxpid_t *pid,
                            axxpid_real_t kp,
                            axxpid_real_t ki,
                            axxpid_real_t kd,
                            axxpid_real_t out_min,
                            axxpid_real_t out_max);

/**
 * @brief Initialise a controller from a fully specified configuration.
 *
 * All controller state is cleared. The configuration is validated as a whole,
 * so a single bad field rejects the lot.
 *
 * @param pid Controller to initialise. Must not be NULL.
 * @param cfg Configuration to copy in. Must not be NULL.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 *
 * @note As with ::axxpid_init, a rejected configuration still leaves @p pid
 *       fully initialised - with the defaults - rather than untouched.
 */
axxpid_status_t axxpid_init_config(axxpid_t *pid, const axxpid_config_t *cfg);

/**
 * @brief Clear all controller state, keeping the configuration.
 *
 * Zeroes the integrator, forgets the derivative history and sets the output
 * to zero. Call this whenever the loop has been stopped for a while, the
 * plant has been disturbed out of band, or the actuator has been switched off
 * behind the controller's back.
 *
 * @param pid Controller. Must not be NULL.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_reset(axxpid_t *pid);

/**
 * @brief Clear state and arrange for the controller to resume at @p output.
 *
 * The integrator is preloaded on the next ::axxpid_update so that the first
 * computed output matches @p output, giving a bumpless start from a known
 * actuator position.
 *
 * @param pid    Controller. Must not be NULL.
 * @param output Output value to resume from, clamped to the output limits.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM if @p output
 *         is not finite - in which case the controller is still cleared, it
 *         just does not resume from anything.
 */
axxpid_status_t axxpid_reset_to(axxpid_t *pid, axxpid_real_t output);

/* -------------------------------------------------------------------------- */
/* Execution                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Run one control update.
 *
 * This is the primary entry point: you own the timing, AxxPID owns the maths.
 * Call it from a timer interrupt or a periodic task with the real elapsed
 * time in seconds.
 *
 * Non-finite inputs and a non-positive @p dt are ignored: the previous output
 * is returned and no state is changed, so a single bad sensor reading cannot
 * poison the integrator. The derivative history is restarted across the gap,
 * because differentiating across a skipped interval would otherwise report
 * roughly twice the real rate of change.
 *
 * Mixing this with ::axxpid_update_at on the same controller is safe. This
 * call drops the millisecond clock, so the next ::axxpid_update_at
 * resynchronises from its nominal period instead of integrating the same
 * interval a second time.
 *
 * @param pid         Controller. Must not be NULL.
 * @param setpoint    Desired process value.
 * @param measurement Current process value.
 * @param dt          Elapsed time since the previous update, in seconds.
 * @return The new output, clamped to the configured limits. Returns 0 if
 *         @p pid is NULL.
 */
axxpid_real_t axxpid_update(axxpid_t *pid,
                            axxpid_real_t setpoint,
                            axxpid_real_t measurement,
                            axxpid_real_t dt);

/**
 * @brief Run one control update on a millisecond clock, rate-limited.
 *
 * Call this as often as you like from a free-running main loop and pass any
 * monotonically increasing millisecond counter (@c HAL_GetTick,
 * @c xTaskGetTickCount, @c millis, ...). The controller recomputes only once
 * @c sample_time_ms has elapsed, using the true measured interval, and skips
 * the work otherwise. The subtraction is unsigned, so a 32-bit counter
 * wrapping at 49.7 days is handled correctly.
 *
 * Set axxpid_config_t::update_every_call to recompute on every call instead.
 *
 * A rejected sample - a non-finite setpoint or measurement - returns false
 * and leaves the clock alone, so the skipped interval is folded into the next
 * good sample rather than being dropped from the integrator.
 *
 * @param pid         Controller. Must not be NULL.
 * @param setpoint    Desired process value.
 * @param measurement Current process value.
 * @param now_ms      Current time in milliseconds.
 * @retval true  The controller ran; ::axxpid_get_output has a fresh value.
 * @retval false Not enough time had elapsed; nothing changed.
 */
bool axxpid_update_at(axxpid_t *pid,
                      axxpid_real_t setpoint,
                      axxpid_real_t measurement,
                      uint32_t now_ms);

/* -------------------------------------------------------------------------- */
/* Tuning                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Set all three gains, parallel form.
 * @param pid Controller. Must not be NULL.
 * @param kp  Proportional gain, >= 0.
 * @param ki  Integral gain, >= 0.
 * @param kd  Derivative gain, >= 0.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_tunings(axxpid_t *pid,
                                   axxpid_real_t kp,
                                   axxpid_real_t ki,
                                   axxpid_real_t kd);

/**
 * @brief Set the gains in standard (ISA / "ideal") form.
 *
 * @f$ u = k_p \left( e + \frac{1}{T_i}\int e\,dt + T_d \frac{de}{dt} \right) @f$,
 * i.e. @c ki = kp/ti and @c kd = kp*td. This is the form used by industrial
 * process controllers and by most textbook tuning tables.
 *
 * @param pid Controller. Must not be NULL.
 * @param kp  Proportional gain, >= 0.
 * @param ti  Integral time in seconds, > 0, or 0 to disable integral action.
 * @param td  Derivative time in seconds, >= 0.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_tunings_standard(axxpid_t *pid,
                                            axxpid_real_t kp,
                                            axxpid_real_t ti,
                                            axxpid_real_t td);

/** @brief Set the proportional gain only. @see axxpid_set_tunings */
axxpid_status_t axxpid_set_kp(axxpid_t *pid, axxpid_real_t kp);
/** @brief Set the integral gain only. @see axxpid_set_tunings */
axxpid_status_t axxpid_set_ki(axxpid_t *pid, axxpid_real_t ki);
/** @brief Set the derivative gain only. @see axxpid_set_tunings */
axxpid_status_t axxpid_set_kd(axxpid_t *pid, axxpid_real_t kd);

/* -------------------------------------------------------------------------- */
/* Configuration setters                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Set the actuator range.
 * @param pid Controller. Must not be NULL.
 * @param min Lower limit.
 * @param max Upper limit, strictly greater than @p min.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_output_limits(axxpid_t *pid,
                                         axxpid_real_t min,
                                         axxpid_real_t max);

/**
 * @brief Limit how fast the output may change.
 * @param pid  Controller. Must not be NULL.
 * @param rate Output units per second; 0 or ::AXXPID_UNLIMITED disables it.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM if negative.
 */
axxpid_status_t axxpid_set_output_slew_rate(axxpid_t *pid, axxpid_real_t rate);

/**
 * @brief Set the integral term directly, in output units.
 *
 * The low-level escape hatch, for what the rest of the API does not cover:
 * clearing only the integral, without touching the derivative history or the
 * last output (pass 0); restoring a value saved to flash across a power
 * cycle; or handing over between gain schedules.
 *
 * Prefer ::axxpid_reset_to for "resume from this actuator position" - it
 * works out the integral for you and accounts for the P, D and feed-forward
 * terms.
 *
 * @param pid   Controller. Must not be NULL.
 * @param value New integral term, clamped to the configured integral limits.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM if @p value
 *         is not finite.
 */
axxpid_status_t axxpid_set_integral(axxpid_t *pid, axxpid_real_t value);

/**
 * @brief Clamp the integral term to an explicit range.
 *
 * Independent of the output limits, and the most direct way to cap how much
 * authority the integrator can ever accumulate. AxxSolder runs its heater at
 * +/- 300 out of a 0..500 output range.
 *
 * @param pid Controller. Must not be NULL.
 * @param min Lower limit.
 * @param max Upper limit, greater than or equal to @p min. Equal limits are
 *            legal and pin the integrator to that value, which is how you
 *            disable integral action without touching @c ki.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_integral_limits(axxpid_t *pid,
                                           axxpid_real_t min,
                                           axxpid_real_t max);

/**
 * @brief Choose the anti-windup strategy.
 * @param pid           Controller. Must not be NULL.
 * @param mode          Strategy to use.
 * @param tracking_time @f$T_t@f$ in seconds. Stored whenever it is greater
 *                      than zero, but only used - and only required to be
 *                      valid - in ::AXXPID_ANTIWINDUP_BACK_CALCULATION mode,
 *                      so you can set it once and switch modes later.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_antiwindup(axxpid_t *pid,
                                      axxpid_antiwindup_t mode,
                                      axxpid_real_t tracking_time);

/**
 * @brief Set the integral engagement band.
 * @param pid  Controller. Must not be NULL.
 * @param band Error width, >= 0; ::AXXPID_UNLIMITED disables it.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 * @see axxpid_config_t::integral_band
 */
axxpid_status_t axxpid_set_integral_band(axxpid_t *pid, axxpid_real_t band);

/**
 * @brief Give the integrator extra gain once the process has overshot.
 * @param pid       Controller. Must not be NULL.
 * @param gain      Multiplier applied to @c ki below @p threshold, >= 0.
 *                  Use 1.0 for symmetric behaviour.
 * @param threshold Error below which @p gain applies, typically 0 or slightly
 *                  negative.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 * @see axxpid_config_t::integral_overshoot_gain
 */
axxpid_status_t axxpid_set_integral_overshoot(axxpid_t *pid,
                                              axxpid_real_t gain,
                                              axxpid_real_t threshold);

/**
 * @brief Force the integrator to zero whenever the setpoint is exactly zero.
 * @param pid    Controller. Must not be NULL.
 * @param enable Whether to enable the behaviour.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_set_integral_reset_on_zero_setpoint(axxpid_t *pid,
                                                           bool enable);

/**
 * @brief Enable the derivative low-pass filter via the divisor @f$N@f$.
 * @param pid Controller. Must not be NULL.
 * @param n   Filter divisor, typically 8..16; 0 disables filtering.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM if negative.
 * @see axxpid_config_t::derivative_filter_n
 */
axxpid_status_t axxpid_set_derivative_filter(axxpid_t *pid, axxpid_real_t n);

/**
 * @brief Enable the derivative low-pass filter with an explicit time constant.
 * @param pid Controller. Must not be NULL.
 * @param tau Filter time constant in seconds; 0 falls back to the @f$N@f$
 *            setting.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM if negative.
 */
axxpid_status_t axxpid_set_derivative_filter_tau(axxpid_t *pid,
                                                 axxpid_real_t tau);

/**
 * @brief Set the two-degree-of-freedom setpoint weights.
 * @param pid Controller. Must not be NULL.
 * @param b   Proportional weight, 0..1. 1 = proportional on error.
 * @param c   Derivative weight, 0..1. 0 = derivative on measurement.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_setpoint_weights(axxpid_t *pid,
                                            axxpid_real_t b,
                                            axxpid_real_t c);

/**
 * @brief Set the error deadband.
 * @param pid      Controller. Must not be NULL.
 * @param deadband Half-width in error units, >= 0. 0 disables it.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_deadband(axxpid_t *pid, axxpid_real_t deadband);

/**
 * @brief Set the constant feed-forward term.
 * @param pid  Controller. Must not be NULL.
 * @param bias Output units added to every update.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_set_feedforward_bias(axxpid_t *pid, axxpid_real_t bias);

/**
 * @brief Set the setpoint-proportional and setpoint-rate feed-forward gains.
 *
 * @c setpoint_gain is the steady-state inverse plant gain: if holding the
 * process at @c sp needs an output of @c sp/G, then @c setpoint_gain = 1/G
 * puts the actuator roughly where it belongs before the loop has to correct
 * anything. @c rate_gain adds a push proportional to how fast the setpoint is
 * moving, which is what makes a ramp track without lag.
 *
 * @param pid           Controller. Must not be NULL.
 * @param setpoint_gain Output units per setpoint unit.
 * @param rate_gain     Output units per (setpoint unit / second).
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_set_feedforward_gains(axxpid_t *pid,
                                             axxpid_real_t setpoint_gain,
                                             axxpid_real_t rate_gain);

/**
 * @brief Install an arbitrary feed-forward function.
 * @param pid  Controller. Must not be NULL.
 * @param fn   Callback, or NULL to remove it.
 * @param user Opaque pointer passed straight back to @p fn.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_set_feedforward_fn(axxpid_t *pid,
                                          axxpid_ff_fn_t fn,
                                          void *user);

/**
 * @brief Set the actuator sense.
 * @param pid    Controller. Must not be NULL.
 * @param acting Direct or reverse.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_acting(axxpid_t *pid, axxpid_acting_t acting);

/**
 * @brief Switch between manual and automatic operation.
 *
 * Switching to manual captures the current output as the manual output, so
 * that automatic -> manual is seamless without you having to supply a value.
 * If you want to go manual at a *specific* output, call this first and
 * ::axxpid_set_manual_output second - the other order has its value
 * overwritten by the capture.
 *
 * Switching to automatic preloads the integral on the next update, so the
 * first automatic output is the manual output plus one normal integral step -
 * that step is control action, not a bump.
 *
 * Three settings legitimately override the preload and will produce a real
 * step: the integral limits, if the value needed falls outside them; the
 * integral engagement band, if the error is outside it; and the zero-setpoint
 * reset, if it is on and the setpoint is zero. Each of those exists to stop
 * the integral holding a value, so they take priority.
 *
 * @param pid  Controller. Must not be NULL.
 * @param mode Mode to switch to.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_mode(axxpid_t *pid, axxpid_mode_t mode);

/**
 * @brief Set the output used while in manual mode.
 *
 * Stored as given and clamped to the output limits when it is applied, so
 * widening the limits later restores what you asked for.
 *
 * @note ::axxpid_set_mode overwrites this when it switches to manual. Set the
 *       mode first, then the output.
 *
 * @param pid    Controller. Must not be NULL.
 * @param output Output value, clamped to the output limits when applied.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM if @p output
 *         is not finite.
 */
axxpid_status_t axxpid_set_manual_output(axxpid_t *pid, axxpid_real_t output);

/**
 * @brief Configure the rate limiting used by ::axxpid_update_at.
 * @param pid               Controller. Must not be NULL.
 * @param sample_time_ms    Nominal control period in milliseconds, > 0.
 * @param update_every_call Recompute on every call using the true elapsed
 *                          time, ignoring @p sample_time_ms.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_sample_time(axxpid_t *pid,
                                       uint32_t sample_time_ms,
                                       bool update_every_call);

/**
 * @brief Cap the @c dt that ::axxpid_update will act on.
 * @param pid    Controller. Must not be NULL.
 * @param dt_max Seconds, > 0; ::AXXPID_UNLIMITED disables the cap.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_set_dt_max(axxpid_t *pid, axxpid_real_t dt_max);

/**
 * @brief Compensate the integral when @c kp or @c kd change at run time.
 *
 * Not applied on the first update after a reset, or after a rejected sample:
 * there is no valid P or D history to compensate against.
 *
 * @param pid    Controller. Must not be NULL.
 * @param enable Whether to enable bumpless retuning.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_set_bumpless_tuning(axxpid_t *pid, bool enable);

/* -------------------------------------------------------------------------- */
/* Getters                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief Latest output. Returns 0 if @p pid is NULL. */
axxpid_real_t axxpid_get_output(const axxpid_t *pid);
/** @brief Latest control error, after the deadband. */
axxpid_real_t axxpid_get_error(const axxpid_t *pid);
/** @brief Proportional contribution to the latest output. */
axxpid_real_t axxpid_get_p_term(const axxpid_t *pid);
/**
 * @brief Integral contribution to the latest output.
 *
 * This is a snapshot taken when the output was computed, not the live
 * integrator. It is what you want for a tuning graph, because it is the
 * number that actually went into the output shown next to it. For the current
 * state - to save it, or after ::axxpid_set_integral - use
 * ::axxpid_get_integral.
 */
axxpid_real_t axxpid_get_i_term(const axxpid_t *pid);

/**
 * @brief The live integral term, in output units.
 *
 * The counterpart to ::axxpid_set_integral, so a controller's integrator can
 * be saved to flash and restored across a power cycle. Unlike
 * ::axxpid_get_i_term this reflects every change since the last update,
 * including back-calculation and the setters.
 */
axxpid_real_t axxpid_get_integral(const axxpid_t *pid);
/** @brief Derivative contribution to the latest output. */
axxpid_real_t axxpid_get_d_term(const axxpid_t *pid);
/** @brief Feed-forward contribution to the latest output. */
axxpid_real_t axxpid_get_ff_term(const axxpid_t *pid);

/**
 * @brief Fetch the whole P/I/D/FF breakdown in one call.
 *
 * Handy for a tuning graph or a telemetry frame: P + I + D + FF is always
 * exactly the output the control law asked for, before clamping.
 *
 * @param pid   Controller. Must not be NULL.
 * @param terms Destination. Must not be NULL.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_get_terms(const axxpid_t *pid, axxpid_terms_t *terms);

/**
 * @brief True if the control law asked for an output the actuator cannot give.
 *
 * Set whenever the returned output differs from P + I + D + FF, which covers
 * both the output limits and the slew rate. A loop that sits saturated for
 * long stretches is telling you the actuator is undersized, or @c kp is far
 * too high.
 */
bool axxpid_is_saturated(const axxpid_t *pid);

/** @brief Proportional gain, exactly as it was set. */
axxpid_real_t axxpid_get_kp(const axxpid_t *pid);
/** @brief Integral gain, exactly as it was set. */
axxpid_real_t axxpid_get_ki(const axxpid_t *pid);
/** @brief Derivative gain, exactly as it was set. */
axxpid_real_t axxpid_get_kd(const axxpid_t *pid);

/** @brief Current operating mode. */
axxpid_mode_t axxpid_get_mode(const axxpid_t *pid);
/** @brief Current actuator sense. */
axxpid_acting_t axxpid_get_acting(const axxpid_t *pid);

/** @brief Library version string, e.g. "1.0.0". */
const char *axxpid_version(void);

#ifdef __cplusplus
}
#endif

#endif /* AXXPID_H */
