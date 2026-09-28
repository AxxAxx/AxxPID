/**
 * @file    axxpid_tune.h
 * @brief   AxxPID tuning helpers: classical rules and a relay autotuner.
 * @version 1.0.0
 * @license MIT
 *
 * This module is optional. It does not touch a running controller unless you
 * ask it to via ::axxpid_tune_apply, and it can be left out of a build
 * entirely by simply not compiling @c axxpid_tune.c.
 *
 * Two things live here:
 *
 * 1. **Tuning rules** - the published tables that turn a process
 *    characterisation into gains. Give them either an ultimate gain and
 *    period @f$(K_u, T_u)@f$ from a sustained-oscillation test, or a
 *    first-order-plus-dead-time model @f$(K, L, T)@f$ from a step test.
 *
 * 2. **A relay autotuner** (Astrom & Hagglund) - drives the process with a
 *    bang-bang output around the operating point, measures the limit cycle it
 *    provokes, and reports @f$(K_u, T_u)@f$ so you can feed the rules above.
 *
 * @warning An autotune deliberately makes the process oscillate. Only run it
 *          on a plant that can safely swing about the operating point, and
 *          always pick a relay amplitude your actuator and your hardware can
 *          live with.
 */

#ifndef AXXPID_TUNE_H
#define AXXPID_TUNE_H

#include "axxpid/axxpid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Gain set                                                                   */
/* -------------------------------------------------------------------------- */

/** @brief A set of gains in both the parallel and the standard form. */
typedef struct {
    axxpid_real_t kp; /**< Proportional gain. */
    axxpid_real_t ki; /**< Integral gain, = kp/ti. */
    axxpid_real_t kd; /**< Derivative gain, = kp*td. */

    /**
     * Integral time in seconds. Zero means "no integral action", which is
     * also what @c ki == 0 means; the two always agree, in both directions,
     * for any gain set this module returns.
     */
    axxpid_real_t ti;
    axxpid_real_t td; /**< Derivative time in seconds. 0 means no D action. */
} axxpid_gains_t;

/**
 * @brief Build a gain set from standard-form parameters.
 * @param kp Proportional gain.
 * @param ti Integral time in seconds, or 0 for no integral action.
 * @param td Derivative time in seconds.
 * @return The gain set, with @c ki and @c kd filled in.
 */
axxpid_gains_t axxpid_gains_from_standard(axxpid_real_t kp,
                                          axxpid_real_t ti,
                                          axxpid_real_t td);

/**
 * @brief Build a gain set from parallel-form parameters.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @return The gain set, with @c ti and @c td filled in.
 */
axxpid_gains_t axxpid_gains_from_parallel(axxpid_real_t kp,
                                          axxpid_real_t ki,
                                          axxpid_real_t kd);

/**
 * @brief Load a gain set into a controller.
 * @param pid   Controller. Must not be NULL.
 * @param gains Gains to apply. Must not be NULL.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_tune_apply(axxpid_t *pid, const axxpid_gains_t *gains);

/* -------------------------------------------------------------------------- */
/* Rules based on the ultimate gain and period                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Published tuning rules that take an ultimate gain and period.
 *
 * @f$K_u@f$ is the proportional gain at which a P-only loop oscillates with
 * constant amplitude, and @f$T_u@f$ is the period of that oscillation. Get
 * them by hand (raise @c kp until the loop just sustains an oscillation) or
 * from the relay autotuner below.
 *
 * Ordered roughly from most aggressive to most conservative.
 */
typedef enum {
    AXXPID_RULE_ZN_P = 0,    /**< Ziegler-Nichols P: kp = 0.5 Ku. */
    AXXPID_RULE_ZN_PI,       /**< Ziegler-Nichols PI: 0.45 Ku, Tu/1.2. */
    AXXPID_RULE_ZN_PID,      /**< Ziegler-Nichols PID: 0.6 Ku, Tu/2, Tu/8. Fast, ~25% overshoot. */
    AXXPID_RULE_PESSEN,      /**< Pessen Integral Rule: 0.7 Ku, Tu/2.5, 3Tu/20. Faster still. */
    AXXPID_RULE_SOME_OVERSHOOT, /**< 0.33 Ku, Tu/2, Tu/3. Gentler than ZN. */
    AXXPID_RULE_NO_OVERSHOOT,   /**< 0.2 Ku, Tu/2, Tu/3. Use when overshoot is unacceptable. */
    AXXPID_RULE_TYREUS_LUYBEN_PI,  /**< Ku/3.2, 2.2 Tu. Robust, for noisy lag-dominant loops. */
    AXXPID_RULE_TYREUS_LUYBEN_PID  /**< Ku/2.2, 2.2 Tu, Tu/6.3. Robust PID. */
} axxpid_rule_t;

/**
 * @brief Apply a tuning rule to an ultimate gain and period.
 * @param rule Rule to use.
 * @param ku   Ultimate gain, > 0.
 * @param tu   Ultimate period in seconds, > 0.
 * @return The resulting gain set, or an all-zero set if an argument is
 *         invalid or the rule is unknown.
 */
axxpid_gains_t axxpid_tune_from_ultimate(axxpid_rule_t rule,
                                         axxpid_real_t ku,
                                         axxpid_real_t tu);

/* -------------------------------------------------------------------------- */
/* Rules based on a first-order-plus-dead-time step test                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Ziegler-Nichols open-loop (reaction curve) PID rule.
 *
 * Run a step test: move the output by @f$\Delta u@f$ in open loop and watch
 * the measurement settle. Then @f$K = \Delta pv / \Delta u@f$ is the process
 * gain, @f$L@f$ the apparent dead time and @f$T@f$ the apparent time
 * constant, both read off the steepest-tangent construction.
 *
 * @param k Process gain, > 0 (measurement units per output unit).
 * @param l Dead time in seconds, > 0.
 * @param t Process time constant in seconds, > 0.
 * @return The gain set, or an all-zero set if an argument is invalid.
 */
axxpid_gains_t axxpid_tune_ziegler_nichols_open(axxpid_real_t k,
                                                axxpid_real_t l,
                                                axxpid_real_t t);

/**
 * @brief Cohen-Coon PID rule.
 *
 * Designed for a quarter-amplitude-damping response and noticeably better
 * than open-loop Ziegler-Nichols on dead-time-dominant processes
 * (@f$L/T > 0.3@f$). Aggressive; expect overshoot.
 *
 * @param k Process gain, > 0.
 * @param l Dead time in seconds, > 0.
 * @param t Process time constant in seconds, > 0.
 * @return The gain set, or an all-zero set if an argument is invalid.
 */
axxpid_gains_t axxpid_tune_cohen_coon(axxpid_real_t k,
                                      axxpid_real_t l,
                                      axxpid_real_t t);

/**
 * @brief Lambda / SIMC PI rule (Skogestad).
 *
 * You pick the closed-loop speed directly through @f$\lambda@f$, which makes
 * this the easiest rule to reason about and by far the most forgiving in
 * production. @f$\lambda = L@f$ is aggressive, @f$\lambda = 3L@f$ is a good
 * robust default, larger is slower and calmer.
 *
 * @f$ k_p = \frac{T}{K(\lambda + L)},\quad T_i = \min(T,\ 4(\lambda + L)),
 *     \quad T_d = 0 @f$
 *
 * @param k      Process gain, > 0.
 * @param l      Dead time in seconds, >= 0.
 * @param t      Process time constant in seconds, > 0.
 * @param lambda Desired closed-loop time constant in seconds, > 0.
 * @return The gain set, or an all-zero set if an argument is invalid.
 */
axxpid_gains_t axxpid_tune_lambda(axxpid_real_t k,
                                  axxpid_real_t l,
                                  axxpid_real_t t,
                                  axxpid_real_t lambda);

/* -------------------------------------------------------------------------- */
/* Relay autotuner                                                            */
/* -------------------------------------------------------------------------- */

/** @brief Relay autotuner settings. */
typedef struct {
    /** Operating point to oscillate about, in measurement units. */
    axxpid_real_t setpoint;

    /**
     * Nominal output, in output units. The relay swings either side of it.
     *
     * Put it near the output that actually holds the process at
     * ::axxpid_relay_config_t::setpoint, which you can read off a manual-mode
     * run. It matters more than it looks: an off-centre bias makes the
     * oscillation spend longer on one side than the other, which biases
     * @f$T_u@f$ - around +20% for a bias one relay amplitude away from the
     * equilibrium on a first-order process. Worse, if the equilibrium output
     * falls outside @c output_bias +/- @c output_step the relay never
     * switches at all, and the autotune runs to its timeout with the actuator
     * pinned to one end.
     */
    axxpid_real_t output_bias;

    /**
     * Relay half-amplitude @f$d@f$, in output units, > 0. The output
     * alternates between @c output_bias + d and @c output_bias - d. Big
     * enough to move the process clearly out of the noise, small enough to
     * keep the excursion safe.
     */
    axxpid_real_t output_step;

    /**
     * Switching hysteresis @f$h@f$, in measurement units, >= 0. Set it just
     * above the peak-to-peak measurement noise, or the relay will chatter on
     * the noise instead of on the process.
     *
     * Keep it small, and understand what the correction does and does not
     * do. The square-root term in the @f$K_u@f$ formula adjusts for the
     * shifted switching point: it is worth @f$1/\sqrt{1-(h/a)^2}@f$, so
     * about 0.5% at @f$h/a = 0.1@f$ and 2% at 0.2. What it does *not* correct
     * is the larger effect - hysteresis moves the oscillation off the
     * ultimate frequency altogether. Measured on a first-order-plus-dead-time
     * process, @f$h@f$ at a tenth of the amplitude biases @f$T_u@f$ by about
     * +10% and a fifth by about +20%, with @f$K_u@f$ drifting low to match.
     * Treat hysteresis as the price of measuring through noise, not as a free
     * knob.
     *
     * The autotune fails outright if the oscillation it provokes is less than
     * twice @f$h@f$, because below that the estimate means nothing.
     */
    axxpid_real_t hysteresis;

    /**
     * Number of complete oscillation cycles to average, >= 1. Three or four
     * is plenty.
     */
    uint8_t cycles;

    /**
     * Cycles to discard before averaging starts, so the initial transient
     * does not pollute the result. One or two.
     */
    uint8_t settle_cycles;

    /** Give up after this many seconds, > 0. Use it as a safety net. */
    axxpid_real_t timeout;

    /** Actuator sense, matching the controller you are tuning. */
    axxpid_acting_t acting;
} axxpid_relay_config_t;

/**
 * @brief Minimum samples in one oscillation for it to be believed.
 *
 * A relay switching every sample or two is following sensor noise, not the
 * process, and would otherwise be reported as a very fast, very large limit
 * cycle - which the tuning rules turn into an enormous @c kp. Periods shorter
 * than this are discarded, so a noise-driven autotune reaches its timeout
 * instead of returning confident nonsense. Override it with @c -D at build
 * time if you genuinely run a relay at a handful of samples per cycle.
 */
#ifndef AXXPID_RELAY_MIN_SAMPLES_PER_CYCLE
#define AXXPID_RELAY_MIN_SAMPLES_PER_CYCLE 8u
#endif

/** @brief How a relay autotune ended. */
typedef enum {
    AXXPID_RELAY_RUNNING = 0, /**< Still oscillating, keep calling update. */
    AXXPID_RELAY_DONE,        /**< Finished; ::axxpid_relay_result is valid. */

    /**
     * No usable limit cycle before the timeout. Either the process never
     * crossed the setpoint - check that @c output_bias +/- @c output_step
     * straddles the output that holds it there - or every apparent cycle was
     * too short to be real. See ::AXXPID_RELAY_MIN_SAMPLES_PER_CYCLE.
     */
    AXXPID_RELAY_TIMEOUT,

    /** The oscillation was too small to measure against the hysteresis. */
    AXXPID_RELAY_FAILED
} axxpid_relay_state_t;

/**
 * @brief Relay autotuner state. Allocate one, initialise it, then drive it.
 *
 * Treat the fields as private.
 */
typedef struct {
    axxpid_relay_config_t cfg;
    axxpid_relay_state_t state;

    bool relay_high;  /**< Current relay position. */
    bool have_period; /**< A full-period measurement window is open. */

    /**
     * Elapsed time, split into whole seconds and a fraction below one.
     *
     * A single accumulator cannot do this job in float. At 100 kHz, `elapsed
     * += dt` stops advancing once elapsed reaches 256 s, because 1e-5 is
     * below half of that number's last bit - so the timeout never arrives and
     * the autotune runs forever. Keeping every addition near 1.0 avoids it.
     */
    uint32_t elapsed_seconds;
    axxpid_real_t elapsed_fraction;

    axxpid_real_t period_time;  /**< Time inside the open window, seconds. */
    uint16_t period_samples;    /**< Samples taken inside the open window. */
    axxpid_real_t period_max;   /**< Highest signal seen in the open window. */
    axxpid_real_t period_min;   /**< Lowest signal seen in the open window. */

    uint16_t cycles_seen;         /**< Complete periods observed. */
    uint16_t period_count;        /**< Periods accepted into the average. */
    axxpid_real_t period_sum;     /**< Sum of the accepted periods. */
    axxpid_real_t amplitude_sum;  /**< Sum of the accepted half-amplitudes. */

    axxpid_real_t output;    /**< Output to apply right now. */
    axxpid_real_t ku;        /**< Estimated ultimate gain, once done. */
    axxpid_real_t tu;        /**< Estimated ultimate period, once done. */
    axxpid_real_t amplitude; /**< Measured half-amplitude, once done. */
} axxpid_relay_t;

/**
 * @brief Fill @p cfg with sensible relay defaults.
 *
 * Amplitude 0, setpoint 0, hysteresis 0, 4 cycles averaged after 1 settling
 * cycle, 600 s timeout, direct acting. You must at least set
 * ::axxpid_relay_config_t::setpoint and ::axxpid_relay_config_t::output_step.
 *
 * @param cfg Configuration to initialise. Must not be NULL.
 * @return ::AXXPID_OK or ::AXXPID_ERR_NULL.
 */
axxpid_status_t axxpid_relay_config_default(axxpid_relay_config_t *cfg);

/**
 * @brief Start a relay autotune.
 * @param relay Autotuner to initialise. Must not be NULL.
 * @param cfg   Settings to copy in. Must not be NULL.
 * @return ::AXXPID_OK, ::AXXPID_ERR_NULL, or ::AXXPID_ERR_PARAM.
 */
axxpid_status_t axxpid_relay_init(axxpid_relay_t *relay,
                                  const axxpid_relay_config_t *cfg);

/**
 * @brief Advance the autotune by one sample.
 *
 * Call it at your normal control rate, in place of ::axxpid_update, and drive
 * the actuator with the value it writes to @p output.
 *
 * @code
 *   axxpid_real_t u;
 *   while (axxpid_relay_update(&relay, read_sensor(), 0.1f, &u) ==
 *          AXXPID_RELAY_RUNNING) {
 *       drive_actuator(u);
 *   }
 * @endcode
 *
 * @param relay       Autotuner. Must not be NULL.
 * @param measurement Current process value.
 * @param dt          Elapsed time since the previous call, in seconds, > 0.
 * @param output      Receives the value to drive the actuator with. May be
 *                    NULL. Once the autotune has finished - however it
 *                    finished - this is ::axxpid_relay_config_t::output_bias,
 *                    so a caller that keeps driving the actuator from it
 *                    parks the process at its nominal output. On a rejected
 *                    sample, a @p dt of zero or less or a measurement that is
 *                    not finite, the relay holds its current output.
 * @return The current autotuner state.
 */
axxpid_relay_state_t axxpid_relay_update(axxpid_relay_t *relay,
                                         axxpid_real_t measurement,
                                         axxpid_real_t dt,
                                         axxpid_real_t *output);

/**
 * @brief Read the ultimate gain and period a finished autotune measured.
 *
 * The estimate comes from the describing function of an ideal relay:
 * @f$ K_u = \frac{4d}{\pi \sqrt{a^2 - h^2}} @f$, where @f$d@f$ is the relay
 * half-amplitude, @f$a@f$ the measured half-amplitude of the limit cycle and
 * @f$h@f$ the switching hysteresis.
 *
 * @param relay Autotuner. Must not be NULL.
 * @param ku    Receives the ultimate gain. May be NULL.
 * @param tu    Receives the ultimate period in seconds. May be NULL.
 * @return ::AXXPID_OK when the autotune succeeded, ::AXXPID_ERR_NULL, or
 *         ::AXXPID_ERR_PARAM if it has not finished successfully.
 */
axxpid_status_t axxpid_relay_result(const axxpid_relay_t *relay,
                                    axxpid_real_t *ku,
                                    axxpid_real_t *tu);

/**
 * @brief Turn a finished autotune straight into gains.
 * @param relay Autotuner. Must not be NULL.
 * @param rule  Tuning rule to apply.
 * @return The gain set, or an all-zero set if the autotune did not succeed.
 */
axxpid_gains_t axxpid_relay_gains(const axxpid_relay_t *relay,
                                  axxpid_rule_t rule);

#ifdef __cplusplus
}
#endif

#endif /* AXXPID_TUNE_H */
