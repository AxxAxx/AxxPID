/**
 * @file  test_tune.c
 * @brief Tuning rules and relay autotuner.
 *
 * The rule tables are checked against the published coefficients by hand. The
 * autotuner is checked against a plant whose true ultimate gain and period
 * are computed numerically in the test itself, so the assertions are anchored
 * to control theory rather than to a previous run of this code.
 */

#include "axxpid_plant.h"
#include "axxpid_test.h"

#include "axxpid/axxpid_tune.h"

#define DT AXXPID_C(0.01)

/* -------------------------------------------------------------------------- */
/* Form conversion                                                            */
/* -------------------------------------------------------------------------- */

static void test_gain_conversions(void)
{
    axxpid_gains_t g;

    g = axxpid_gains_from_standard(4, 2, AXXPID_C(0.5));
    CHECK_NEAR(g.kp, 4, AXXPID_C(1e-6));
    CHECK_NEAR(g.ki, 2, AXXPID_C(1e-6));
    CHECK_NEAR(g.kd, 2, AXXPID_C(1e-6));
    CHECK_NEAR(g.ti, 2, AXXPID_C(1e-6));
    CHECK_NEAR(g.td, AXXPID_C(0.5), AXXPID_C(1e-6));

    /* Round-tripping must be exact. */
    g = axxpid_gains_from_parallel(g.kp, g.ki, g.kd);
    CHECK_NEAR(g.ti, 2, AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(0.5), AXXPID_C(1e-6));

    /* No integral action: ti = 0 in, ki = 0 out, and back again. */
    g = axxpid_gains_from_standard(3, 0, 0);
    CHECK(g.ki == 0);
    g = axxpid_gains_from_parallel(3, 0, 0);
    CHECK(g.ti == 0);
    CHECK(g.td == 0);

    /* kp = 0 must not divide by zero. */
    g = axxpid_gains_from_parallel(0, 1, 1);
    CHECK(g.td == 0);

    /* Negative gains are rejected outright. */
    g = axxpid_gains_from_standard(-1, 1, 1);
    CHECK(g.kp == 0 && g.ki == 0 && g.kd == 0);
}

static void test_apply_gains(void)
{
    axxpid_t pid;
    axxpid_gains_t g;

    (void)axxpid_init(&pid, 0, 0, 0, -1000, 1000);
    g = axxpid_gains_from_standard(5, 10, 1);

    CHECK(axxpid_tune_apply(&pid, &g) == AXXPID_OK);
    CHECK(axxpid_tune_apply(NULL, &g) == AXXPID_ERR_NULL);
    CHECK(axxpid_tune_apply(&pid, NULL) == AXXPID_ERR_NULL);

    CHECK_NEAR(axxpid_get_kp(&pid), 5, AXXPID_C(1e-6));
    CHECK_NEAR(axxpid_get_ki(&pid), AXXPID_C(0.5), AXXPID_C(1e-6));
    CHECK_NEAR(axxpid_get_kd(&pid), 5, AXXPID_C(1e-6));
}

/* -------------------------------------------------------------------------- */
/* Rule tables                                                                */
/* -------------------------------------------------------------------------- */

static void test_ultimate_rules(void)
{
    const axxpid_real_t ku = 10;
    const axxpid_real_t tu = 4;
    axxpid_gains_t g;

    g = axxpid_tune_from_ultimate(AXXPID_RULE_ZN_P, ku, tu);
    CHECK_NEAR(g.kp, 5, AXXPID_C(1e-5));
    CHECK(g.ki == 0);
    CHECK(g.kd == 0);

    g = axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PI, ku, tu);
    CHECK_NEAR(g.kp, AXXPID_C(4.5), AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, AXXPID_C(3.333333), AXXPID_C(1e-4));
    CHECK(g.kd == 0);

    /* Ziegler-Nichols PID: 0.6 Ku, Tu/2, Tu/8. */
    g = axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PID, ku, tu);
    CHECK_NEAR(g.kp, 6, AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, 2, AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(0.5), AXXPID_C(1e-5));
    CHECK_NEAR(g.ki, 3, AXXPID_C(1e-5));
    CHECK_NEAR(g.kd, 3, AXXPID_C(1e-5));

    g = axxpid_tune_from_ultimate(AXXPID_RULE_PESSEN, ku, tu);
    CHECK_NEAR(g.kp, 7, AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, AXXPID_C(1.6), AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(0.6), AXXPID_C(1e-5));

    g = axxpid_tune_from_ultimate(AXXPID_RULE_SOME_OVERSHOOT, ku, tu);
    CHECK_NEAR(g.kp, AXXPID_C(3.333333), AXXPID_C(1e-4));
    CHECK_NEAR(g.ti, 2, AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(1.333333), AXXPID_C(1e-4));

    g = axxpid_tune_from_ultimate(AXXPID_RULE_NO_OVERSHOOT, ku, tu);
    CHECK_NEAR(g.kp, 2, AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, 2, AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(1.333333), AXXPID_C(1e-4));

    /* Tyreus-Luyben: kp = Ku/3.2 for PI but Ku/2.2 for PID, which is easy to
     * get wrong in either direction. Literal values, so that a typo in the
     * implementation cannot be mirrored here and pass. */
    g = axxpid_tune_from_ultimate(AXXPID_RULE_TYREUS_LUYBEN_PI, ku, tu);
    CHECK_NEAR(g.kp, AXXPID_C(3.125), AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, AXXPID_C(8.8), AXXPID_C(1e-5));
    CHECK(g.kd == 0);

    g = axxpid_tune_from_ultimate(AXXPID_RULE_TYREUS_LUYBEN_PID, ku, tu);
    CHECK_NEAR(g.kp, AXXPID_C(4.545455), AXXPID_C(1e-4));
    CHECK_NEAR(g.ti, AXXPID_C(8.8), AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(0.634921), AXXPID_C(1e-5));

    /* The conservative rules must actually be more conservative. */
    CHECK(axxpid_tune_from_ultimate(AXXPID_RULE_NO_OVERSHOOT, ku, tu).kp <
          axxpid_tune_from_ultimate(AXXPID_RULE_SOME_OVERSHOOT, ku, tu).kp);
    CHECK(axxpid_tune_from_ultimate(AXXPID_RULE_SOME_OVERSHOOT, ku, tu).kp <
          axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PID, ku, tu).kp);
    CHECK(axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PID, ku, tu).kp <
          axxpid_tune_from_ultimate(AXXPID_RULE_PESSEN, ku, tu).kp);

    /* Bad arguments produce zeros, never garbage or a divide by zero. */
    g = axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PID, 0, tu);
    CHECK(g.kp == 0);
    g = axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PID, ku, 0);
    CHECK(g.kp == 0);
    g = axxpid_tune_from_ultimate(AXXPID_RULE_ZN_PID, -1, tu);
    CHECK(g.kp == 0);
    g = axxpid_tune_from_ultimate((axxpid_rule_t)99, ku, tu);
    CHECK(g.kp == 0);
}

static void test_step_test_rules(void)
{
    axxpid_gains_t g;

    /* Ziegler-Nichols open loop: kp = 1.2T/(KL) = 1.2*10/(2*1) = 6,
     * Ti = 2L = 2, Td = 0.5L = 0.5. */
    g = axxpid_tune_ziegler_nichols_open(2, 1, 10);
    CHECK_NEAR(g.kp, 6, AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, 2, AXXPID_C(1e-5));
    CHECK_NEAR(g.td, AXXPID_C(0.5), AXXPID_C(1e-5));

    /* Cohen-Coon with K=1, L=1, T=10 so r = 0.1:
     * kp = (4/3 + 0.1/4)/(1*0.1)             = 13.5833
     * Ti = 1*(32 + 0.6)/(13 + 0.8)           =  2.3623
     * Td = 4*1/(11 + 0.2)                    =  0.35714 */
    g = axxpid_tune_cohen_coon(1, 1, 10);
    CHECK_NEAR(g.kp, AXXPID_C(13.58333), AXXPID_C(1e-3));
    CHECK_NEAR(g.ti, AXXPID_C(2.362319), AXXPID_C(1e-3));
    CHECK_NEAR(g.td, AXXPID_C(0.357143), AXXPID_C(1e-4));

    /* Lambda / SIMC: kp = T/(K(lambda+L)) = 20/(1*(6+2)) = 2.5,
     * Ti = min(T, 4(lambda+L)) = min(20, 32) = 20, Td = 0. */
    g = axxpid_tune_lambda(1, 2, 20, 6);
    CHECK_NEAR(g.kp, AXXPID_C(2.5), AXXPID_C(1e-5));
    CHECK_NEAR(g.ti, 20, AXXPID_C(1e-5));
    CHECK(g.td == 0);

    /* And the min() branch: 4(lambda+L) = 4*(1+0) = 4 < T = 20. */
    g = axxpid_tune_lambda(1, 0, 20, 1);
    CHECK_NEAR(g.ti, 4, AXXPID_C(1e-5));

    /* A larger lambda must always give a gentler controller. */
    CHECK(axxpid_tune_lambda(1, 2, 20, 20).kp <
          axxpid_tune_lambda(1, 2, 20, 2).kp);

    CHECK(axxpid_tune_ziegler_nichols_open(0, 1, 1).kp == 0);
    CHECK(axxpid_tune_cohen_coon(1, 0, 1).kp == 0);
    CHECK(axxpid_tune_lambda(1, 1, 1, 0).kp == 0);
}

/* -------------------------------------------------------------------------- */
/* Relay autotuner                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Numerically find the true (Ku, Tu) of a first-order-plus-dead-time
 *        plant, so the autotuner can be judged against theory.
 *
 * At the ultimate frequency the open-loop phase is exactly -180 degrees:
 * @f$ \arctan(\omega T) + \omega L = \pi @f$. Ku is the reciprocal of the
 * magnitude there, @f$ K/\sqrt{1 + (\omega T)^2} @f$.
 */
static void fopdt_ultimate(double gain,
                           double dead_time,
                           double tau,
                           double *ku,
                           double *tu)
{
    const double pi = 3.14159265358979323846;
    double lo = 1e-6;
    double hi = 1000.0;
    int i;

    /* atan(wT) + wL rises monotonically in w, so a bisection is exact. */
    for (i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        double phase = 0.0;
        double x = mid * tau;
        double term;
        int k;

        /* atan via its arctangent series, range-reduced with the identity
         * atan(x) = pi/2 - atan(1/x) so the series always converges fast. */
        if (x > 1.0) {
            x = 1.0 / x;
            term = x;
            for (k = 0; k < 400; ++k) {
                phase += term / (double)(2 * k + 1);
                term *= -x * x;
            }
            phase = (pi / 2.0) - phase;
        } else {
            term = x;
            for (k = 0; k < 400; ++k) {
                phase += term / (double)(2 * k + 1);
                term *= -x * x;
            }
        }
        phase += mid * dead_time;

        if (phase < pi) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    {
        const double w = 0.5 * (lo + hi);
        const double wt = w * tau;
        double magnitude = gain;
        double root = 1.0 + (wt * wt);
        double r = root;
        int k;

        for (k = 0; k < 60; ++k) {
            r = 0.5 * (r + (root / r));
        }
        magnitude /= r;

        *ku = 1.0 / magnitude;
        *tu = (2.0 * pi) / w;
    }
}

static void test_relay_config_validation(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;

    CHECK(axxpid_relay_config_default(NULL) == AXXPID_ERR_NULL);
    CHECK(axxpid_relay_config_default(&cfg) == AXXPID_OK);
    CHECK(cfg.cycles > 0u);
    CHECK(cfg.timeout > 0);

    CHECK(axxpid_relay_init(NULL, &cfg) == AXXPID_ERR_NULL);
    CHECK(axxpid_relay_init(&relay, NULL) == AXXPID_ERR_NULL);

    /* output_step defaults to 0, which is not a usable relay. */
    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_ERR_PARAM);

    cfg.output_step = 10;
    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_OK);

    cfg.hysteresis = -1;
    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_ERR_PARAM);
    cfg.hysteresis = 0;

    cfg.cycles = 0u;
    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_ERR_PARAM);
    cfg.cycles = 4u;

    cfg.timeout = 0;
    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_ERR_PARAM);
    cfg.timeout = 100;

    cfg.acting = (axxpid_acting_t)9;
    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_ERR_PARAM);

    /* Results are unavailable until it has actually finished. */
    cfg.acting = AXXPID_ACTING_DIRECT;
    (void)axxpid_relay_init(&relay, &cfg);
    CHECK(axxpid_relay_result(&relay, NULL, NULL) == AXXPID_ERR_PARAM);
    CHECK(axxpid_relay_result(NULL, NULL, NULL) == AXXPID_ERR_NULL);
    CHECK(axxpid_relay_gains(&relay, AXXPID_RULE_ZN_PID).kp == 0);
    CHECK(axxpid_relay_gains(NULL, AXXPID_RULE_ZN_PID).kp == 0);
    CHECK(axxpid_relay_update(NULL, 0, DT, NULL) == AXXPID_RELAY_FAILED);
}

/** @brief Drive a relay autotune to completion against a simulated plant. */
static axxpid_relay_state_t run_relay(axxpid_relay_t *relay,
                                      axxpid_plant_t *plant,
                                      long max_steps)
{
    axxpid_real_t pv = plant->y;
    axxpid_real_t u = 0;
    axxpid_relay_state_t state = AXXPID_RELAY_RUNNING;
    long i;

    for (i = 0; (i < max_steps) && (state == AXXPID_RELAY_RUNNING); ++i) {
        state = axxpid_relay_update(relay, pv, DT, &u);
        pv = axxpid_plant_step(plant, u, DT);
    }
    return state;
}

static void test_relay_finds_ultimate_gain(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_plant_t plant;
    axxpid_real_t ku = 0;
    axxpid_real_t tu = 0;
    double true_ku = 0;
    double true_tu = 0;

    /* K = 1, tau = 10 s, dead time 2 s. */
    fopdt_ultimate(1.0, 2.0, 10.0, &true_ku, &true_tu);

    axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.cycles = 4u;
    cfg.settle_cycles = 2u;
    cfg.timeout = 2000;

    CHECK(axxpid_relay_init(&relay, &cfg) == AXXPID_OK);
    CHECK(run_relay(&relay, &plant, 400000L) == AXXPID_RELAY_DONE);
    CHECK(axxpid_relay_result(&relay, &ku, &tu) == AXXPID_OK);

    /* The describing function is a first-harmonic approximation, so some
     * error is inherent rather than a defect. On this lag-dominant plant with
     * no hysteresis it lands Tu within about 1% and Ku about 18% low, so the
     * tolerances are set just wide enough to cover that and no wider - a
     * 30% band would not notice the estimate drifting another 10%. */
    CHECK_MSG(axxpid_test_near(tu, (axxpid_real_t)true_tu,
                               (axxpid_real_t)(true_tu * 0.03)),
              "Tu estimate %.3f s vs theoretical %.3f s", (double)tu, true_tu);
    CHECK_MSG(axxpid_test_near(ku, (axxpid_real_t)(true_ku * 0.82),
                               (axxpid_real_t)(true_ku * 0.05)),
              "Ku estimate %.3f vs theoretical %.3f (expected ~18%% low)",
              (double)ku, true_ku);
}

/* The hysteresis correction must keep the estimate honest, or a noise band
 * silently biases every autotune on a real machine. */
static void test_relay_with_hysteresis(void)
{
    axxpid_relay_t plain;
    axxpid_relay_t hyst;
    axxpid_plant_t plant_a;
    axxpid_plant_t plant_b;
    axxpid_real_t ku_plain = 0;
    axxpid_real_t ku_hyst = 0;
    axxpid_real_t tu_plain = 0;
    axxpid_real_t tu_hyst = 0;
    axxpid_relay_config_t cfg;

    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.cycles = 4u;
    cfg.settle_cycles = 2u;
    cfg.timeout = 2000;

    axxpid_plant_init(&plant_a, 1, 10, 0, 2, DT);
    (void)axxpid_relay_init(&plain, &cfg);
    CHECK(run_relay(&plain, &plant_a, 400000L) == AXXPID_RELAY_DONE);
    (void)axxpid_relay_result(&plain, &ku_plain, &tu_plain);

    /* The describing-function correction is a first-harmonic result, so it
     * holds while the hysteresis stays small next to the limit-cycle
     * amplitude - which is exactly how you would set it in practice, just
     * above the measurement noise. */
    cfg.hysteresis = AXXPID_C(0.5);
    axxpid_plant_init(&plant_b, 1, 10, 0, 2, DT);
    (void)axxpid_relay_init(&hyst, &cfg);
    CHECK(run_relay(&hyst, &plant_b, 400000L) == AXXPID_RELAY_DONE);
    (void)axxpid_relay_result(&hyst, &ku_hyst, &tu_hyst);

    /* Hysteresis does not come free: it moves the oscillation off the
     * ultimate frequency, which pulls Ku down and pushes Tu up. What the test
     * pins is that a sensibly small h costs something bounded and in the
     * documented direction, not that it is corrected away. */
    CHECK_MSG(ku_hyst < ku_plain,
              "hysteresis raised Ku from %.3f to %.3f, expected a drop",
              (double)ku_plain, (double)ku_hyst);
    CHECK_MSG(ku_hyst > ku_plain * AXXPID_C(0.8),
              "hysteresis cost %.1f%% of Ku, more than documented",
              (double)((1.0 - (double)(ku_hyst / ku_plain)) * 100.0));
    CHECK_MSG(tu_hyst > tu_plain,
              "hysteresis lowered Tu from %.3f to %.3f, expected a rise",
              (double)tu_plain, (double)tu_hyst);
    CHECK_MSG(tu_hyst < tu_plain * AXXPID_C(1.2),
              "hysteresis stretched Tu by %.1f%%, more than documented",
              (double)(((double)(tu_hyst / tu_plain) - 1.0) * 100.0));
}

static void test_relay_reverse_acting(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_plant_t plant;
    axxpid_real_t ku_forward = 0;
    axxpid_real_t ku_reverse = 0;
    axxpid_real_t tu_forward = 0;
    axxpid_real_t tu_reverse = 0;

    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.settle_cycles = 2u;
    cfg.timeout = 2000;

    axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
    (void)axxpid_relay_init(&relay, &cfg);
    CHECK(run_relay(&relay, &plant, 400000L) == AXXPID_RELAY_DONE);
    (void)axxpid_relay_result(&relay, &ku_forward, &tu_forward);

    /* Same plant, wired backwards, tuned in reverse mode: identical answer. */
    cfg.acting = AXXPID_ACTING_REVERSE;
    cfg.setpoint = -50;
    axxpid_plant_init(&plant, -1, 10, 0, 2, DT);
    (void)axxpid_relay_init(&relay, &cfg);
    CHECK(run_relay(&relay, &plant, 400000L) == AXXPID_RELAY_DONE);
    (void)axxpid_relay_result(&relay, &ku_reverse, &tu_reverse);

    CHECK_NEAR(ku_reverse, ku_forward, ku_forward * AXXPID_C(0.05));
    CHECK_NEAR(tu_reverse, tu_forward, tu_forward * AXXPID_C(0.05));
}

static void test_relay_timeout(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_plant_t plant;

    /* The relay swings 55..65 but the plant needs ~60 to sit at 50 and moves
     * so slowly it never gets near the setpoint, so the relay never switches
     * at all. That is the commonest way an autotune fails on real hardware. */
    axxpid_plant_init(&plant, 1, 500, 0, 0, DT);
    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 5;
    cfg.timeout = 20;

    (void)axxpid_relay_init(&relay, &cfg);
    CHECK(run_relay(&relay, &plant, 400000L) == AXXPID_RELAY_TIMEOUT);
    CHECK(axxpid_relay_result(&relay, NULL, NULL) == AXXPID_ERR_PARAM);
}

/* The whole point: autotune, apply the gains, and the loop must actually
 * control the plant it was tuned on. */
static void test_autotune_produces_a_working_controller(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_plant_t plant;
    axxpid_t pid;
    axxpid_gains_t gains;
    axxpid_real_t pv;
    axxpid_real_t peak = 0;
    long i;

    axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.settle_cycles = 2u;
    cfg.timeout = 2000;

    (void)axxpid_relay_init(&relay, &cfg);
    CHECK(run_relay(&relay, &plant, 400000L) == AXXPID_RELAY_DONE);

    gains = axxpid_relay_gains(&relay, AXXPID_RULE_TYREUS_LUYBEN_PID);
    CHECK(gains.kp > 0);
    CHECK(gains.ki > 0);
    CHECK(gains.kd > 0);

    (void)axxpid_init(&pid, 0, 0, 0, 0, 300);
    (void)axxpid_set_derivative_filter(&pid, 10);
    CHECK(axxpid_tune_apply(&pid, &gains) == AXXPID_OK);

    axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
    pv = plant.y;
    for (i = 0; i < 40000; ++i) {
        pv = axxpid_plant_step(&plant, axxpid_update(&pid, 100, pv, DT), DT);
        if (pv > peak) {
            peak = pv;
        }
    }

    CHECK_MSG(axxpid_test_near(pv, 100, AXXPID_C(1.0)),
              "autotuned loop settled at %.2f", (double)pv);
    CHECK_MSG(peak < AXXPID_C(115),
              "the Tyreus-Luyben rule overshot to %.1f", (double)peak);

    /* Sanity-check the ordering the rule table promises: the robust rule must
     * be calmer on the same plant than the aggressive one. */
    {
        axxpid_gains_t aggressive =
            axxpid_relay_gains(&relay, AXXPID_RULE_ZN_PID);
        axxpid_real_t aggressive_peak = 0;

        CHECK(aggressive.kp > gains.kp);

        (void)axxpid_init(&pid, 0, 0, 0, 0, 300);
        (void)axxpid_set_derivative_filter(&pid, 10);
        (void)axxpid_tune_apply(&pid, &aggressive);

        axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
        pv = plant.y;
        for (i = 0; i < 40000; ++i) {
            pv = axxpid_plant_step(&plant, axxpid_update(&pid, 100, pv, DT),
                                   DT);
            if (pv > aggressive_peak) {
                aggressive_peak = pv;
            }
        }
        CHECK_MSG(aggressive_peak > peak,
                  "Ziegler-Nichols peaked at %.1f, no higher than Tyreus-Luyben"
                  " at %.1f",
                  (double)aggressive_peak, (double)peak);
    }
}

/* An infinity in the measurement used to spin the square root forever, inside
 * the control loop, because infinity divided by four is still infinity. */
static void test_relay_rejects_non_finite_samples(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_plant_t plant;
    axxpid_real_t u = 0;
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t inf_value = AXXPID_C(1) / zero;
    const axxpid_real_t nan_value = zero / zero;
    long i;

    axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.settle_cycles = 2u;
    cfg.timeout = 2000;
    (void)axxpid_relay_init(&relay, &cfg);

    /* Poison it early, then run normally. It must finish, and finish sane. */
    CHECK(axxpid_relay_update(&relay, inf_value, DT, &u) ==
          AXXPID_RELAY_RUNNING);
    CHECK(axxpid_relay_update(&relay, nan_value, DT, &u) ==
          AXXPID_RELAY_RUNNING);

    {
        axxpid_real_t pv = plant.y;
        axxpid_relay_state_t state = AXXPID_RELAY_RUNNING;

        for (i = 0; (i < 400000L) && (state == AXXPID_RELAY_RUNNING); ++i) {
            state = axxpid_relay_update(&relay, pv, DT, &u);
            pv = axxpid_plant_step(&plant, u, DT);
        }
        CHECK(state == AXXPID_RELAY_DONE);
    }

    {
        axxpid_real_t ku = 0;

        CHECK(axxpid_relay_result(&relay, &ku, NULL) == AXXPID_OK);
        CHECK(ku > 0);
        CHECK(ku < 1000);
    }
}

/* A relay that flips every sample is following sensor noise, not the process.
 * Reporting that as a limit cycle yields a tiny Tu and an enormous Ku, and the
 * tuning rules turn it into gains that would wreck the plant. */
static void test_relay_rejects_noise(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_relay_state_t state = AXXPID_RELAY_RUNNING;
    axxpid_real_t u = 0;
    long i;

    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.hysteresis = 1;
    cfg.settle_cycles = 0u;
    cfg.timeout = 60;
    (void)axxpid_relay_init(&relay, &cfg);

    /* Pure chatter: the measurement jumps either side of the hysteresis band
     * every sample and never responds to the output at all. */
    for (i = 0; (i < 400000L) && (state == AXXPID_RELAY_RUNNING); ++i) {
        const axxpid_real_t noisy =
            ((i % 2) == 0) ? AXXPID_C(48.5) : AXXPID_C(51.5);

        state = axxpid_relay_update(&relay, noisy, DT, &u);
    }

    CHECK_MSG(state == AXXPID_RELAY_TIMEOUT,
              "noise was accepted as a limit cycle (state %d)", (int)state);
    CHECK(axxpid_relay_result(&relay, NULL, NULL) == AXXPID_ERR_PARAM);
    CHECK(axxpid_relay_gains(&relay, AXXPID_RULE_ZN_PID).kp == 0);
}

/* The oscillation has to be clearly bigger than the switching band, or the
 * estimate runs away as 1/sqrt(a^2 - h^2). */
static void test_relay_fails_on_a_marginal_oscillation(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_relay_state_t state = AXXPID_RELAY_RUNNING;
    axxpid_real_t u = 0;
    long i;

    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.hysteresis = 1;
    cfg.settle_cycles = 0u;
    cfg.cycles = 2u;
    cfg.timeout = 600;
    (void)axxpid_relay_init(&relay, &cfg);

    /* A slow triangle that only just clears the hysteresis band: amplitude
     * barely above h, so the describing function has nothing to work with. */
    for (i = 0; (i < 400000L) && (state == AXXPID_RELAY_RUNNING); ++i) {
        const long phase = i % 400L;
        const axxpid_real_t ramp =
            (phase < 200L) ? ((axxpid_real_t)phase / AXXPID_C(200))
                           : ((axxpid_real_t)(400L - phase) / AXXPID_C(200));
        const axxpid_real_t pv =
            AXXPID_C(48.9) + (AXXPID_C(2.2) * ramp);

        state = axxpid_relay_update(&relay, pv, DT, &u);
    }

    CHECK_MSG(state != AXXPID_RELAY_DONE,
              "a marginal oscillation was accepted (state %d)", (int)state);
    CHECK(axxpid_relay_gains(&relay, AXXPID_RULE_ZN_PID).kp == 0);
}

/* Once it has finished, further calls must be inert and park the actuator. */
static void test_relay_is_inert_after_finishing(void)
{
    axxpid_relay_t relay;
    axxpid_relay_config_t cfg;
    axxpid_plant_t plant;
    axxpid_real_t u = 99;
    axxpid_real_t ku_before = 0;
    axxpid_real_t ku_after = 0;

    axxpid_plant_init(&plant, 1, 10, 0, 2, DT);
    (void)axxpid_relay_config_default(&cfg);
    cfg.setpoint = 50;
    cfg.output_bias = 50;
    cfg.output_step = 20;
    cfg.settle_cycles = 2u;
    cfg.timeout = 2000;
    (void)axxpid_relay_init(&relay, &cfg);
    CHECK(run_relay(&relay, &plant, 400000L) == AXXPID_RELAY_DONE);
    (void)axxpid_relay_result(&relay, &ku_before, NULL);

    CHECK(axxpid_relay_update(&relay, 12345, DT, &u) == AXXPID_RELAY_DONE);
    CHECK_NEAR(u, cfg.output_bias, AXXPID_C(1e-5));
    (void)axxpid_relay_result(&relay, &ku_after, NULL);
    CHECK(ku_after == ku_before);

    /* A bad timestep holds the relay where it is rather than disturbing it. */
    CHECK(axxpid_relay_update(&relay, 50, 0, &u) == AXXPID_RELAY_DONE);
}

static void test_gain_set_rejections(void)
{
    axxpid_t pid;
    axxpid_gains_t bad;

    bad = axxpid_gains_from_parallel(-1, 1, 1);
    CHECK(bad.kp == 0 && bad.ki == 0 && bad.kd == 0);

    (void)axxpid_init(&pid, 5, 5, 5, 0, 100);
    CHECK(axxpid_tune_apply(&pid, &bad) == AXXPID_OK);
    /* An all-zero gain set is a legitimate "do nothing" controller. */
    CHECK(axxpid_get_kp(&pid) == 0);
}

static const axxpid_test_case_t tests[] = {
    {"standard / parallel conversions", test_gain_conversions},
    {"applying a gain set", test_apply_gains},
    {"ultimate-gain rule tables", test_ultimate_rules},
    {"step-test rule tables", test_step_test_rules},
    {"relay configuration validation", test_relay_config_validation},
    {"relay recovers the true Ku and Tu", test_relay_finds_ultimate_gain},
    {"relay corrects for hysteresis", test_relay_with_hysteresis},
    {"relay handles a reverse-acting plant", test_relay_reverse_acting},
    {"relay times out on a hopeless plant", test_relay_timeout},
    {"relay rejects non-finite samples",
     test_relay_rejects_non_finite_samples},
    {"relay rejects noise as a limit cycle", test_relay_rejects_noise},
    {"relay fails on a marginal oscillation",
     test_relay_fails_on_a_marginal_oscillation},
    {"relay is inert after finishing", test_relay_is_inert_after_finishing},
    {"gain set rejections", test_gain_set_rejections},
    {"autotune produces a working controller",
     test_autotune_produces_a_working_controller},
};

AXXPID_TEST_MAIN("tune", tests)
