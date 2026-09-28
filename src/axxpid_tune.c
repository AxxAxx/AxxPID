/**
 * @file  axxpid_tune.c
 * @brief AxxPID tuning rules and relay autotuner implementation.
 *
 * Like the core, this file has no standard library dependency - not even
 * <math.h>. The one transcendental it needs is a square root, provided here
 * by range reduction plus Newton-Raphson so that the library links on a
 * bare-metal target with no libm.
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

#include "axxpid/axxpid_tune.h"

/** @brief Pi, to more digits than double precision can hold. */
#define AXXPID_PI AXXPID_C(3.14159265358979323846)

/**
 * @brief Reject NaN and infinities without pulling in <math.h>.
 *
 * NaN fails every comparison including equality with itself; infinities fall
 * outside the ::AXXPID_UNLIMITED sentinel range, which is finite for exactly
 * this reason.
 */
static bool axxpid_is_finite(axxpid_real_t v)
{
    return (v == v) && (v <= AXXPID_UNLIMITED) && (v >= -AXXPID_UNLIMITED);
}

/* -------------------------------------------------------------------------- */
/* Local helpers                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Square root without <math.h>.
 *
 * The argument is first scaled by powers of four into [1, 4), where six
 * Newton-Raphson iterations from a seed of 1.5 are already accurate to well
 * beyond double precision, then the result is scaled back by the matching
 * power of two. Ten iterations are used for margin; this runs once per
 * autotune, not once per sample.
 */
static axxpid_real_t axxpid_sqrt(axxpid_real_t v)
{
    axxpid_real_t scale = 1;
    axxpid_real_t x;
    int i;

    /* Reject non-finite input before the range reduction. Infinity divided by
     * four is still infinity, so the loop below would never terminate - and
     * it runs inside a control loop. */
    if (!(v > 0) || !axxpid_is_finite(v)) {
        return 0;
    }
    while (v >= AXXPID_C(4)) {
        v /= AXXPID_C(4);
        scale *= AXXPID_C(2);
    }
    while (v < AXXPID_C(1)) {
        v *= AXXPID_C(4);
        scale /= AXXPID_C(2);
    }

    x = AXXPID_C(1.5);
    for (i = 0; i < 10; ++i) {
        x = AXXPID_C(0.5) * (x + (v / x));
    }
    return x * scale;
}

/** @brief Build an all-zero gain set, used for every rejected argument. */
static axxpid_gains_t axxpid_zero_gains(void)
{
    axxpid_gains_t g;
    g.kp = 0;
    g.ki = 0;
    g.kd = 0;
    g.ti = 0;
    g.td = 0;
    return g;
}

/* -------------------------------------------------------------------------- */
/* Gain sets                                                                  */
/* -------------------------------------------------------------------------- */

axxpid_gains_t axxpid_gains_from_standard(axxpid_real_t kp,
                                          axxpid_real_t ti,
                                          axxpid_real_t td)
{
    axxpid_gains_t g;

    if ((kp < 0) || (ti < 0) || (td < 0)) {
        return axxpid_zero_gains();
    }
    g.kp = kp;
    g.ti = ti;
    g.td = td;
    g.ki = (ti > 0) ? (kp / ti) : AXXPID_C(0);
    g.kd = kp * td;
    return g;
}

axxpid_gains_t axxpid_gains_from_parallel(axxpid_real_t kp,
                                          axxpid_real_t ki,
                                          axxpid_real_t kd)
{
    axxpid_gains_t g;

    if ((kp < 0) || (ki < 0) || (kd < 0)) {
        return axxpid_zero_gains();
    }
    g.kp = kp;
    g.ki = ki;
    g.kd = kd;
    g.ti = (ki > 0) ? (kp / ki) : AXXPID_C(0);
    g.td = (kp > 0) ? (kd / kp) : AXXPID_C(0);
    return g;
}

axxpid_status_t axxpid_tune_apply(axxpid_t *pid, const axxpid_gains_t *gains)
{
    if ((pid == (axxpid_t *)0) || (gains == (const axxpid_gains_t *)0)) {
        return AXXPID_ERR_NULL;
    }
    return axxpid_set_tunings(pid, gains->kp, gains->ki, gains->kd);
}

/* -------------------------------------------------------------------------- */
/* Rules from the ultimate gain and period                                    */
/* -------------------------------------------------------------------------- */

axxpid_gains_t axxpid_tune_from_ultimate(axxpid_rule_t rule,
                                         axxpid_real_t ku,
                                         axxpid_real_t tu)
{
    if (!(ku > 0) || !(tu > 0)) {
        return axxpid_zero_gains();
    }

    switch (rule) {
        case AXXPID_RULE_ZN_P:
            return axxpid_gains_from_standard(AXXPID_C(0.5) * ku, 0, 0);

        case AXXPID_RULE_ZN_PI:
            return axxpid_gains_from_standard(AXXPID_C(0.45) * ku,
                                              tu / AXXPID_C(1.2), 0);

        case AXXPID_RULE_ZN_PID:
            return axxpid_gains_from_standard(AXXPID_C(0.6) * ku,
                                              tu / AXXPID_C(2),
                                              tu / AXXPID_C(8));

        case AXXPID_RULE_PESSEN:
            return axxpid_gains_from_standard(AXXPID_C(0.7) * ku,
                                              tu / AXXPID_C(2.5),
                                              AXXPID_C(0.15) * tu);

        case AXXPID_RULE_SOME_OVERSHOOT:
            return axxpid_gains_from_standard(ku / AXXPID_C(3),
                                              tu / AXXPID_C(2),
                                              tu / AXXPID_C(3));

        case AXXPID_RULE_NO_OVERSHOOT:
            return axxpid_gains_from_standard(AXXPID_C(0.2) * ku,
                                              tu / AXXPID_C(2),
                                              tu / AXXPID_C(3));

        case AXXPID_RULE_TYREUS_LUYBEN_PI:
            return axxpid_gains_from_standard(ku / AXXPID_C(3.2),
                                              AXXPID_C(2.2) * tu, 0);

        case AXXPID_RULE_TYREUS_LUYBEN_PID:
            return axxpid_gains_from_standard(ku / AXXPID_C(2.2),
                                              AXXPID_C(2.2) * tu,
                                              tu / AXXPID_C(6.3));

        default:
            return axxpid_zero_gains();
    }
}

/* -------------------------------------------------------------------------- */
/* Rules from a step test                                                     */
/* -------------------------------------------------------------------------- */

axxpid_gains_t axxpid_tune_ziegler_nichols_open(axxpid_real_t k,
                                                axxpid_real_t l,
                                                axxpid_real_t t)
{
    if (!(k > 0) || !(l > 0) || !(t > 0)) {
        return axxpid_zero_gains();
    }
    /* kp = 1.2 T / (K L), Ti = 2L, Td = 0.5L. */
    return axxpid_gains_from_standard((AXXPID_C(1.2) * t) / (k * l),
                                      AXXPID_C(2) * l, AXXPID_C(0.5) * l);
}

axxpid_gains_t axxpid_tune_cohen_coon(axxpid_real_t k,
                                      axxpid_real_t l,
                                      axxpid_real_t t)
{
    axxpid_real_t r;
    axxpid_real_t kp;
    axxpid_real_t ti;
    axxpid_real_t td;

    if (!(k > 0) || !(l > 0) || !(t > 0)) {
        return axxpid_zero_gains();
    }

    r = l / t;
    kp = (AXXPID_C(4.0) / AXXPID_C(3.0) + (r / AXXPID_C(4))) / (k * r);
    ti = l * ((AXXPID_C(32) + (AXXPID_C(6) * r)) /
              (AXXPID_C(13) + (AXXPID_C(8) * r)));
    td = (AXXPID_C(4) * l) / (AXXPID_C(11) + (AXXPID_C(2) * r));

    return axxpid_gains_from_standard(kp, ti, td);
}

axxpid_gains_t axxpid_tune_lambda(axxpid_real_t k,
                                  axxpid_real_t l,
                                  axxpid_real_t t,
                                  axxpid_real_t lambda)
{
    axxpid_real_t kp;
    axxpid_real_t ti;
    axxpid_real_t four_sum;

    if (!(k > 0) || (l < 0) || !(t > 0) || !(lambda > 0)) {
        return axxpid_zero_gains();
    }

    kp = t / (k * (lambda + l));
    four_sum = AXXPID_C(4) * (lambda + l);
    ti = (t < four_sum) ? t : four_sum;

    return axxpid_gains_from_standard(kp, ti, 0);
}

/* -------------------------------------------------------------------------- */
/* Relay autotuner                                                            */
/* -------------------------------------------------------------------------- */

axxpid_status_t axxpid_relay_config_default(axxpid_relay_config_t *cfg)
{
    if (cfg == (axxpid_relay_config_t *)0) {
        return AXXPID_ERR_NULL;
    }
    cfg->setpoint = 0;
    cfg->output_bias = 0;
    cfg->output_step = 0;
    cfg->hysteresis = 0;
    cfg->cycles = 4u;
    cfg->settle_cycles = 1u;
    cfg->timeout = AXXPID_C(600);
    cfg->acting = AXXPID_ACTING_DIRECT;
    return AXXPID_OK;
}

axxpid_status_t axxpid_relay_init(axxpid_relay_t *relay,
                                  const axxpid_relay_config_t *cfg)
{
    if ((relay == (axxpid_relay_t *)0) ||
        (cfg == (const axxpid_relay_config_t *)0)) {
        return AXXPID_ERR_NULL;
    }
    if (!(cfg->output_step > 0) || (cfg->hysteresis < 0) ||
        (cfg->cycles == 0u) || !(cfg->timeout > 0)) {
        return AXXPID_ERR_PARAM;
    }
    if ((cfg->acting != AXXPID_ACTING_DIRECT) &&
        (cfg->acting != AXXPID_ACTING_REVERSE)) {
        return AXXPID_ERR_PARAM;
    }

    relay->cfg = *cfg;
    relay->state = AXXPID_RELAY_RUNNING;

    /* Start by driving the process towards the setpoint; the first update
     * flips the relay immediately if it is already on the far side. */
    relay->relay_high = true;
    relay->have_period = false;

    relay->elapsed = 0;
    relay->period_start = 0;
    relay->period_samples = 0u;
    relay->period_max = 0;
    relay->period_min = 0;

    relay->cycles_seen = 0u;
    relay->period_count = 0u;
    relay->period_sum = 0;
    relay->amplitude_sum = 0;

    /* The actuator sense is already folded into the error below, so the relay
     * output itself is always "high means more output". */
    relay->output = cfg->output_bias + cfg->output_step;
    relay->ku = 0;
    relay->tu = 0;
    relay->amplitude = 0;

    return AXXPID_OK;
}

/** @brief Compute Ku and Tu once enough periods have been collected. */
static void axxpid_relay_estimate(axxpid_relay_t *relay)
{
    const axxpid_real_t h = relay->cfg.hysteresis;
    axxpid_real_t a;
    axxpid_real_t denom;

    relay->tu = relay->period_sum / (axxpid_real_t)relay->period_count;
    a = relay->amplitude_sum / (axxpid_real_t)relay->period_count;
    relay->amplitude = a;

    /* The describing function of an ideal relay: the process has gain
     * pi*a/(4d) at the oscillation frequency, so Ku is its reciprocal.
     *
     * Require the oscillation to be at least twice the hysteresis. The relay
     * only switches when |error| exceeds h, so a >= h holds trivially
     * whenever it switched at all - testing a > h would be a guard that can
     * never fire. It is not a rounding concern either: as a approaches h the
     * estimate runs away as 1/sqrt(a^2-h^2), and the whole first-harmonic
     * approximation has stopped meaning anything well before that. */
    if (!axxpid_is_finite(a) || !(a > 0) || !(a >= (AXXPID_C(2) * h))) {
        relay->state = AXXPID_RELAY_FAILED;
        return;
    }

    /* a*sqrt(1-(h/a)^2) rather than sqrt(a*a-h*h): the squares overflow to
     * infinity for a large amplitude in single precision, and this form
     * cannot. */
    {
        const axxpid_real_t ratio = h / a;

        denom = AXXPID_PI * a * axxpid_sqrt(AXXPID_C(1) - (ratio * ratio));
    }
    if (!(denom > 0)) {
        relay->state = AXXPID_RELAY_FAILED;
        return;
    }

    relay->ku = (AXXPID_C(4) * relay->cfg.output_step) / denom;
    relay->state = (relay->tu > 0) ? AXXPID_RELAY_DONE : AXXPID_RELAY_FAILED;
}

axxpid_relay_state_t axxpid_relay_update(axxpid_relay_t *relay,
                                         axxpid_real_t measurement,
                                         axxpid_real_t dt,
                                         axxpid_real_t *output)
{
    axxpid_real_t direction;
    axxpid_real_t signal;
    axxpid_real_t error;
    bool want_high;

    if (relay == (axxpid_relay_t *)0) {
        return AXXPID_RELAY_FAILED;
    }
    if (relay->state != AXXPID_RELAY_RUNNING) {
        if (output != (axxpid_real_t *)0) {
            *output = relay->cfg.output_bias;
        }
        return relay->state;
    }
    if (!(dt > 0) || !axxpid_is_finite(dt) || !axxpid_is_finite(measurement)) {
        /* Bad sample or timestep: hold the relay where it is. */
        if (output != (axxpid_real_t *)0) {
            *output = relay->output;
        }
        return relay->state;
    }

    relay->elapsed += dt;

    direction = (relay->cfg.acting == AXXPID_ACTING_REVERSE) ? AXXPID_C(-1)
                                                             : AXXPID_C(1);

    /* Work in the actuating sense so that "rising" always means "the relay is
     * winning", whichever way round the process is wired. */
    signal = direction * measurement;
    error = (direction * relay->cfg.setpoint) - signal;

    if (relay->have_period) {
        if (signal > relay->period_max) {
            relay->period_max = signal;
        }
        if (signal < relay->period_min) {
            relay->period_min = signal;
        }
        if (relay->period_samples < 0xFFFFu) {
            relay->period_samples++;
        }
    }

    /* Relay law with symmetric hysteresis. */
    want_high = relay->relay_high;
    if (relay->relay_high) {
        if (error < -relay->cfg.hysteresis) {
            want_high = false;
        }
    } else {
        if (error > relay->cfg.hysteresis) {
            want_high = true;
        }
    }

    if (want_high != relay->relay_high) {
        if (want_high) {
            /* A low-to-high switch closes one full period. Measuring between
             * identical switch edges makes the result immune to the phase lag
             * between the relay and the peak it produces. */
            if (relay->have_period) {
                const axxpid_real_t period =
                    relay->elapsed - relay->period_start;

                /* A "cycle" spanning a couple of samples is the relay
                 * chattering on measurement noise, not the process
                 * responding. Accepting it would report a tiny Tu and a huge
                 * Ku, and the tuning rules would turn that into gains that
                 * tear the plant apart. Drop it, and let the autotune time
                 * out rather than answer confidently with nonsense. */
                if (relay->period_samples >=
                    (uint16_t)AXXPID_RELAY_MIN_SAMPLES_PER_CYCLE) {
                    relay->cycles_seen++;

                    if (relay->cycles_seen > relay->cfg.settle_cycles) {
                        relay->period_sum += period;
                        relay->amplitude_sum +=
                            (relay->period_max - relay->period_min) /
                            AXXPID_C(2);
                        relay->period_count++;
                    }
                }
            }

            relay->period_start = relay->elapsed;
            relay->period_samples = 0u;
            relay->period_max = signal;
            relay->period_min = signal;
            relay->have_period = true;
        }
        relay->relay_high = want_high;
    }

    /* `error` is already sense-corrected, so a positive error always means
     * "more output" whichever way the plant is wired. Applying `direction`
     * again here would invert a reverse-acting tune and it would never
     * oscillate. */
    relay->output = relay->cfg.output_bias +
                    (relay->relay_high ? relay->cfg.output_step
                                       : -relay->cfg.output_step);

    if (relay->period_count >= relay->cfg.cycles) {
        axxpid_relay_estimate(relay);
        if (output != (axxpid_real_t *)0) {
            *output = relay->cfg.output_bias;
        }
        return relay->state;
    }

    if (relay->elapsed >= relay->cfg.timeout) {
        relay->state = AXXPID_RELAY_TIMEOUT;
        if (output != (axxpid_real_t *)0) {
            *output = relay->cfg.output_bias;
        }
        return relay->state;
    }

    if (output != (axxpid_real_t *)0) {
        *output = relay->output;
    }
    return AXXPID_RELAY_RUNNING;
}

axxpid_status_t axxpid_relay_result(const axxpid_relay_t *relay,
                                    axxpid_real_t *ku,
                                    axxpid_real_t *tu)
{
    if (relay == (const axxpid_relay_t *)0) {
        return AXXPID_ERR_NULL;
    }
    if (relay->state != AXXPID_RELAY_DONE) {
        return AXXPID_ERR_PARAM;
    }
    if (ku != (axxpid_real_t *)0) {
        *ku = relay->ku;
    }
    if (tu != (axxpid_real_t *)0) {
        *tu = relay->tu;
    }
    return AXXPID_OK;
}

axxpid_gains_t axxpid_relay_gains(const axxpid_relay_t *relay,
                                  axxpid_rule_t rule)
{
    if ((relay == (const axxpid_relay_t *)0) ||
        (relay->state != AXXPID_RELAY_DONE)) {
        return axxpid_zero_gains();
    }
    return axxpid_tune_from_ultimate(rule, relay->ku, relay->tu);
}
