/**
 * @file  test_features.c
 * @brief One test per control-law feature, checked against hand arithmetic.
 *
 * Every expected value here is derived on paper from the difference equations
 * in the README, not recorded from a previous run, so these tests catch a
 * change in behaviour rather than merely pinning it.
 */

#include "axxpid_test.h"

/* -------------------------------------------------------------------------- */
/* Proportional and setpoint weighting                                        */
/* -------------------------------------------------------------------------- */

static void test_proportional(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 3, 0, 0, -1000, 1000);

    /* u = kp * (sp - pv) = 3 * (10 - 4) = 18. */
    CHECK_NEAR(axxpid_update(&pid, 10, 4, AXXPID_C(0.1)), 18, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_get_error(&pid), 6, AXXPID_C(1e-5));
}

static void test_setpoint_weight_b(void)
{
    axxpid_t pid;

    /* b = 0 is proportional-on-measurement: u = kp * (0*sp - pv) = -kp*pv. */
    (void)axxpid_init(&pid, 3, 0, 0, -1000, 1000);
    CHECK(axxpid_set_setpoint_weights(&pid, 0, 0) == AXXPID_OK);
    CHECK_NEAR(axxpid_update(&pid, 10, 4, AXXPID_C(0.1)), -12, AXXPID_C(1e-5));

    /* b = 0.5: u = 3 * (0.5*10 - 4) = 3. */
    (void)axxpid_init(&pid, 3, 0, 0, -1000, 1000);
    (void)axxpid_set_setpoint_weights(&pid, AXXPID_C(0.5), 0);
    CHECK_NEAR(axxpid_update(&pid, 10, 4, AXXPID_C(0.1)), 3, AXXPID_C(1e-5));
}

/* -------------------------------------------------------------------------- */
/* Derivative                                                                 */
/* -------------------------------------------------------------------------- */

static void test_derivative_on_measurement(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 2, -1000, 1000);

    /* No derivative output on the very first sample: there is no history to
     * differentiate against. */
    CHECK_NEAR(axxpid_update(&pid, 0, 0, AXXPID_C(0.1)), 0, AXXPID_C(1e-5));

    /* pv rises by 1 over 0.1 s -> d/dt(pv) = 10, D = -kd * 10 = -20. */
    CHECK_NEAR(axxpid_update(&pid, 0, 1, AXXPID_C(0.1)), -20, AXXPID_C(1e-4));
}

/* The classic derivative-kick test: a setpoint step must produce no D spike
 * when the derivative acts on the measurement. */
static void test_no_derivative_kick(void)
{
    axxpid_t pid;
    axxpid_real_t u;

    (void)axxpid_init(&pid, 0, 0, 10, -10000, 10000);

    (void)axxpid_update(&pid, 0, 5, AXXPID_C(0.01));
    (void)axxpid_update(&pid, 0, 5, AXXPID_C(0.01));

    /* Setpoint jumps by 100 while the measurement holds still. */
    u = axxpid_update(&pid, 100, 5, AXXPID_C(0.01));
    CHECK_MSG(axxpid_test_near(u, 0, AXXPID_C(1e-4)),
              "derivative kicked on a setpoint step: %.4f", (double)u);

    /* With c = 1 the derivative acts on the error instead, and the kick comes
     * back: d(error)/dt = 100/0.01 = 10000, D = 10 * 10000 = 100000, clamped. */
    (void)axxpid_init(&pid, 0, 0, 10, -10000, 10000);
    (void)axxpid_set_setpoint_weights(&pid, 1, 1);
    (void)axxpid_update(&pid, 0, 5, AXXPID_C(0.01));
    u = axxpid_update(&pid, 100, 5, AXXPID_C(0.01));
    CHECK(u > 1000);
}

static void test_derivative_filter_tau(void)
{
    axxpid_t pid;
    axxpid_real_t u;

    (void)axxpid_init(&pid, 0, 0, 1, -1000, 1000);
    CHECK(axxpid_set_derivative_filter_tau(&pid, AXXPID_C(0.9)) == AXXPID_OK);

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));

    /* d_raw = -10. alpha = dt/(tau+dt) = 0.1/1.0 = 0.1, so the filtered
     * derivative reaches only a tenth of it on the first sample: D = -1. */
    u = axxpid_update(&pid, 0, 1, AXXPID_C(0.1));
    CHECK_NEAR(u, -1, AXXPID_C(1e-4));

    /* Second sample of the same ramp: 0.1*(-10) + 0.9*(-1) = -1.9. */
    u = axxpid_update(&pid, 0, 2, AXXPID_C(0.1));
    CHECK_NEAR(u, AXXPID_C(-1.9), AXXPID_C(1e-4));
}

static void test_derivative_filter_n(void)
{
    axxpid_t pid;
    axxpid_real_t u;

    /* Tf = (kd/kp)/N = (1/1)/10 = 0.1 s, so alpha = 0.1/(0.1+0.1) = 0.5. */
    (void)axxpid_init(&pid, 1, 0, 1, -1000, 1000);
    CHECK(axxpid_set_derivative_filter(&pid, 10) == AXXPID_OK);

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    u = axxpid_update(&pid, 0, 1, AXXPID_C(0.1));

    /* P = -1 (kp * -pv), D = kd * 0.5 * -10 = -5. */
    CHECK_NEAR(axxpid_get_d_term(&pid), -5, AXXPID_C(1e-4));
    CHECK_NEAR(u, -6, AXXPID_C(1e-4));
}

/* The filter corner must be set by the real elapsed time, or loop jitter
 * silently detunes it. */
static void test_derivative_filter_tracks_dt(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 1, -1000, 1000);
    (void)axxpid_set_derivative_filter_tau(&pid, AXXPID_C(0.1));

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    /* dt = 0.3 -> alpha = 0.3/0.4 = 0.75, d_raw = -1/0.3 = -3.3333,
     * D = 0.75 * -3.3333 = -2.5. */
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.3));
    CHECK_NEAR(axxpid_get_d_term(&pid), AXXPID_C(-2.5), AXXPID_C(1e-4));
}

/* -------------------------------------------------------------------------- */
/* Integrator                                                                 */
/* -------------------------------------------------------------------------- */

static void test_integral_accumulates_with_real_dt(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 2, 0, -1000, 1000);

    /* Two 0.1 s samples then one 0.5 s sample, error 3 throughout:
     * 2*3*(0.1+0.1+0.5) = 4.2. */
    (void)axxpid_update(&pid, 3, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 3, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 3, 0, AXXPID_C(0.5));
    CHECK_NEAR(axxpid_get_i_term(&pid), AXXPID_C(4.2), AXXPID_C(1e-4));
}

static void test_integral_limits(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 0, 10, 0, -1000, 1000);
    CHECK(axxpid_set_integral_limits(&pid, -7, 7) == AXXPID_OK);

    for (i = 0; i < 100; ++i) {
        (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_i_term(&pid), 7, AXXPID_C(1e-5));

    for (i = 0; i < 100; ++i) {
        (void)axxpid_update(&pid, -10, 0, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_i_term(&pid), -7, AXXPID_C(1e-5));
}

static void test_antiwindup_conditional(void)
{
    axxpid_t pid_guarded;
    axxpid_t pid_open;
    int i;

    (void)axxpid_init(&pid_guarded, 1, 5, 0, 0, 10);
    (void)axxpid_set_antiwindup(&pid_guarded, AXXPID_ANTIWINDUP_CONDITIONAL, 1);

    (void)axxpid_init(&pid_open, 1, 5, 0, 0, 10);
    (void)axxpid_set_antiwindup(&pid_open, AXXPID_ANTIWINDUP_NONE, 1);

    /* Hold a large error against a hard limit for 10 s. */
    for (i = 0; i < 100; ++i) {
        (void)axxpid_update(&pid_guarded, 100, 0, AXXPID_C(0.1));
        (void)axxpid_update(&pid_open, 100, 0, AXXPID_C(0.1));
    }

    /* Conditional integration parks the integrator just shy of the point
     * where the output leaves its range; nothing more can accumulate. */
    CHECK_MSG(axxpid_get_i_term(&pid_guarded) < 15,
              "integrator wound up to %.1f despite anti-windup",
              (double)axxpid_get_i_term(&pid_guarded));
    /* Unprotected, it runs away: 5 * 100 * 10 s = 5000. */
    CHECK(axxpid_get_i_term(&pid_open) > 1000);

    /* Now remove the error. The guarded loop lets go at once; the open one is
     * still commanding full output many samples later. */
    (void)axxpid_update(&pid_guarded, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid_open, 0, 0, AXXPID_C(0.1));
    CHECK(axxpid_get_output(&pid_guarded) < 10);
    CHECK_NEAR(axxpid_get_output(&pid_open), 10, AXXPID_C(1e-5));
}

/* Conditional integration must never block a step that brings the output back
 * into range, or the integrator latches and the loop never recovers. */
static void test_conditional_integration_never_latches(void)
{
    axxpid_t pid;
    axxpid_real_t wound_up;
    int i;

    /* A standing feed-forward of 100 holds the output at its upper limit on
     * its own, so the loop stays saturated no matter what the integrator
     * does - exactly the situation in which a naive rule latches. */
    (void)axxpid_init(&pid, 1, 5, 0, 0, 100);
    (void)axxpid_set_feedforward_bias(&pid, 100);
    (void)axxpid_set_integral_limits(&pid, -200, 200);

    for (i = 0; i < 200; ++i) {
        (void)axxpid_update(&pid, 50, 0, AXXPID_C(0.1));
    }
    wound_up = axxpid_get_i_term(&pid);
    CHECK_NEAR(axxpid_get_output(&pid), 100, AXXPID_C(1e-4));
    CHECK(axxpid_is_saturated(&pid));

    /* Reverse the error. The output is still hard against the limit, but the
     * integral step now reduces the saturation instead of deepening it, so it
     * must be allowed through - every sample, monotonically. */
    for (i = 0; i < 40; ++i) {
        const axxpid_real_t previous = axxpid_get_i_term(&pid);
        (void)axxpid_update(&pid, 0, 2, AXXPID_C(0.1));
        CHECK_MSG(axxpid_get_i_term(&pid) < previous,
                  "integrator latched at %.2f while saturated",
                  (double)axxpid_get_i_term(&pid));
    }
    CHECK(axxpid_get_i_term(&pid) < wound_up - 20);
}

/* The saturation test must key off which limit is exceeded, not off the sign
 * of the output. With an actuator that lives between 20% and 80% the sign is
 * always positive and a sign-based test integrates straight through the lower
 * limit. */
static void test_conditional_integration_with_offset_limits(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 1, 5, 0, 20, 80);

    /* Charge the integrator against the upper limit. */
    for (i = 0; i < 200; ++i) {
        (void)axxpid_update(&pid, 20, 0, AXXPID_C(0.1));
    }
    CHECK(axxpid_get_i_term(&pid) > 40);

    /* Now hold a small negative error. The integrator should unwind until the
     * output reaches the *lower* limit of 20 and then stop - never mind that
     * an output of 20 is still a positive number. */
    for (i = 0; i < 400; ++i) {
        (void)axxpid_update(&pid, 0, 5, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_output(&pid), 20, AXXPID_C(0.1));
    CHECK_MSG(axxpid_get_i_term(&pid) > 20,
              "integrator ran past the lower output limit down to %.2f",
              (double)axxpid_get_i_term(&pid));
}

static void test_antiwindup_back_calculation(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 1, 5, 0, 0, 10);
    CHECK(axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(0.5)) == AXXPID_OK);

    for (i = 0; i < 200; ++i) {
        (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
    }

    /* At the fixed point the bleed matches the charge exactly:
     * ki*e*dt = (dt/Tt)*(u_sat - u_unsat), so u_unsat settles at
     * out_max + ki*e*Tt = 10 + 5*100*0.5 = 260, i.e. I settles near
     * 260 - kp*e = 260 - 100 = 160. Bounded, not unbounded. */
    CHECK_MSG(axxpid_get_i_term(&pid) > 100 && axxpid_get_i_term(&pid) < 220,
              "back-calculation integral settled at %.1f, expected ~160",
              (double)axxpid_get_i_term(&pid));

    /* Reverse the error and it comes off the limit in a handful of samples
     * rather than grinding through the accumulated wind-up. */
    for (i = 0; i < 10; ++i) {
        (void)axxpid_update(&pid, 0, 50, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_output(&pid), 0, AXXPID_C(1e-4));
    CHECK(axxpid_get_i_term(&pid) < 50);
}

/* AxxSolder's integral engagement band: park the integrator while the process
 * is still a long way below the setpoint. */
static void test_integral_band(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    CHECK(axxpid_set_integral_band(&pid, 75) == AXXPID_OK);

    /* Error of 100 is outside the band: nothing accumulates. */
    for (i = 0; i < 50; ++i) {
        (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
    }
    CHECK(axxpid_get_i_term(&pid) == 0);

    /* Error of 50 is inside it: normal integration resumes. */
    for (i = 0; i < 10; ++i) {
        (void)axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_i_term(&pid), 50, AXXPID_C(1e-3));

    /* The band is deliberately one-sided: a big *negative* error (overshoot)
     * must keep the integral it has built up, because an asymmetric actuator
     * needs it to back off. */
    for (i = 0; i < 10; ++i) {
        (void)axxpid_update(&pid, 100, 300, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_i_term(&pid), -150, AXXPID_C(1e-2));
}

/* AxxSolder's asymmetric integral gain: unwind faster than you wind up. */
static void test_integral_asymmetry(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    CHECK(axxpid_set_integral_overshoot(&pid, 7, -1) == AXXPID_OK);

    /* Positive error uses the plain gain: 1 * 10 * 0.1 = 1. */
    (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_i_term(&pid), 1, AXXPID_C(1e-5));

    /* Error of -10 is below the -1 threshold, so it drains seven times as
     * fast: 1 + 7 * (-10) * 0.1 = -6. */
    (void)axxpid_update(&pid, 0, 10, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_i_term(&pid), -6, AXXPID_C(1e-4));

    /* A small negative error inside the threshold is *not* amplified - that
     * dead zone is what stops the fast path causing hunting at the setpoint.
     * -6 + 1 * (-0.5) * 0.1 = -6.05. */
    (void)axxpid_update(&pid, 0, AXXPID_C(0.5), AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_i_term(&pid), AXXPID_C(-6.05), AXXPID_C(1e-4));
}

static void test_integral_reset_on_zero_setpoint(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    CHECK(axxpid_set_integral_reset_on_zero_setpoint(&pid, true) == AXXPID_OK);

    (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.5));
    CHECK_NEAR(axxpid_get_i_term(&pid), 5, AXXPID_C(1e-5));

    (void)axxpid_update(&pid, 0, 50, AXXPID_C(0.1));
    CHECK(axxpid_get_i_term(&pid) == 0);
}

/* -------------------------------------------------------------------------- */
/* Output shaping                                                             */
/* -------------------------------------------------------------------------- */

static void test_output_limits(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 100, 0, 0, -5, 20);

    CHECK_NEAR(axxpid_update(&pid, 10, 0, AXXPID_C(0.1)), 20, AXXPID_C(1e-5));
    CHECK(axxpid_is_saturated(&pid));

    CHECK_NEAR(axxpid_update(&pid, -10, 0, AXXPID_C(0.1)), -5, AXXPID_C(1e-5));
    CHECK(axxpid_is_saturated(&pid));

    CHECK_NEAR(axxpid_update(&pid, AXXPID_C(0.05), 0, AXXPID_C(0.1)), 5,
               AXXPID_C(1e-4));
    CHECK(!axxpid_is_saturated(&pid));
}

static void test_output_slew_rate(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 100, 0, 0, 0, 100);
    CHECK(axxpid_set_output_slew_rate(&pid, 50) == AXXPID_OK);

    /* The law wants 100 immediately; 50 units/s over 0.1 s allows 5. */
    CHECK_NEAR(axxpid_update(&pid, 10, 0, AXXPID_C(0.1)), 5, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_update(&pid, 10, 0, AXXPID_C(0.1)), 10, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_update(&pid, 10, 0, AXXPID_C(0.2)), 20, AXXPID_C(1e-5));
    CHECK(axxpid_is_saturated(&pid));
}

static void test_deadband(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 2, 0, 0, -1000, 1000);
    CHECK(axxpid_set_deadband(&pid, 3) == AXXPID_OK);

    /* Inside the band: no action at all. */
    CHECK_NEAR(axxpid_update(&pid, 10, 8, AXXPID_C(0.1)), 0, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_update(&pid, 10, 13, AXXPID_C(0.1)), 0, AXXPID_C(1e-5));

    /* Outside it the error is shifted, not stepped, so the control signal
     * stays continuous across the band edge: e = 5 - 3 = 2, u = 4. */
    CHECK_NEAR(axxpid_update(&pid, 10, 5, AXXPID_C(0.1)), 4, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_update(&pid, 10, 15, AXXPID_C(0.1)), -4, AXXPID_C(1e-5));
}

/* -------------------------------------------------------------------------- */
/* Feed-forward                                                               */
/* -------------------------------------------------------------------------- */

static void test_feedforward_bias_and_gain(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 0, 0, -1000, 1000);
    CHECK(axxpid_set_feedforward_bias(&pid, 7) == AXXPID_OK);
    CHECK(axxpid_set_feedforward_gains(&pid, AXXPID_C(0.5), 0) == AXXPID_OK);

    /* FF = 7 + 0.5*20 = 17, P = 1*(20-15) = 5. */
    CHECK_NEAR(axxpid_update(&pid, 20, 15, AXXPID_C(0.1)), 22, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_get_ff_term(&pid), 17, AXXPID_C(1e-5));
    CHECK_NEAR(axxpid_get_p_term(&pid), 5, AXXPID_C(1e-5));
}

static void test_feedforward_velocity(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 0, -1000, 1000);
    (void)axxpid_set_feedforward_gains(&pid, 0, 2);

    /* Skipped on the first sample: no previous setpoint to differentiate. */
    (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_ff_term(&pid), 0, AXXPID_C(1e-5));

    /* Setpoint ramps by 1 over 0.1 s -> 10 units/s, FF = 2 * 10 = 20. */
    CHECK_NEAR(axxpid_update(&pid, 11, 0, AXXPID_C(0.1)), 20, AXXPID_C(1e-4));
    /* Setpoint holds -> no velocity feed-forward. */
    CHECK_NEAR(axxpid_update(&pid, 11, 0, AXXPID_C(0.1)), 0, AXXPID_C(1e-4));
}

static axxpid_real_t square_root_ff(axxpid_real_t setpoint,
                                    axxpid_real_t measurement,
                                    void *user)
{
    const axxpid_real_t *scale = (const axxpid_real_t *)user;
    (void)measurement;
    return *scale * setpoint * setpoint;
}

static void test_feedforward_callback(void)
{
    axxpid_t pid;
    axxpid_real_t scale = AXXPID_C(0.25);

    (void)axxpid_init(&pid, 0, 0, 0, -1000, 1000);
    CHECK(axxpid_set_feedforward_fn(&pid, square_root_ff, &scale) == AXXPID_OK);

    /* 0.25 * 4^2 = 4. */
    CHECK_NEAR(axxpid_update(&pid, 4, 0, AXXPID_C(0.1)), 4, AXXPID_C(1e-5));

    /* Removing the hook removes the term. */
    CHECK(axxpid_set_feedforward_fn(&pid, NULL, NULL) == AXXPID_OK);
    CHECK_NEAR(axxpid_update(&pid, 4, 0, AXXPID_C(0.1)), 0, AXXPID_C(1e-5));
}

/* Feed-forward must be inside the saturation calculation, or the integrator
 * winds up fighting a term it cannot see. */
static void test_feedforward_participates_in_antiwindup(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 0, 1, 0, 0, 100);
    (void)axxpid_set_feedforward_bias(&pid, 100);

    /* The feed-forward alone already saturates the actuator, so the
     * integrator has nothing to contribute and must not accumulate. */
    for (i = 0; i < 100; ++i) {
        (void)axxpid_update(&pid, 50, 0, AXXPID_C(0.1));
    }
    CHECK_MSG(axxpid_get_i_term(&pid) < 1,
              "integrator wound up behind a saturating feed-forward: %.2f",
              (double)axxpid_get_i_term(&pid));
}

/* -------------------------------------------------------------------------- */
/* Direction, modes and retuning                                              */
/* -------------------------------------------------------------------------- */

static void test_reverse_acting(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 3, 0, 0, -1000, 1000);
    CHECK(axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE) == AXXPID_OK);
    CHECK(axxpid_get_acting(&pid) == AXXPID_ACTING_REVERSE);

    /* Measurement above setpoint now calls for more output, not less. */
    CHECK_NEAR(axxpid_update(&pid, 10, 14, AXXPID_C(0.1)), 12, AXXPID_C(1e-5));
    /* The gains are stored as given, never silently negated. */
    CHECK(axxpid_get_kp(&pid) == 3);
}

static void test_reverse_acting_derivative(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 2, -1000, 1000);
    (void)axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE);

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    /* pv rising means the reverse-acting output must rise too: +20. */
    CHECK_NEAR(axxpid_update(&pid, 0, 1, AXXPID_C(0.1)), 20, AXXPID_C(1e-4));
}

static void test_manual_mode_and_bumpless_transfer(void)
{
    axxpid_t pid;
    axxpid_real_t u;
    int i;

    (void)axxpid_init(&pid, 2, 1, 0, 0, 100);

    CHECK(axxpid_set_mode(&pid, AXXPID_MODE_MANUAL) == AXXPID_OK);
    CHECK(axxpid_get_mode(&pid) == AXXPID_MODE_MANUAL);
    CHECK(axxpid_set_manual_output(&pid, 30) == AXXPID_OK);

    /* Manual mode returns the manual value no matter what the error is. */
    for (i = 0; i < 20; ++i) {
        u = axxpid_update(&pid, 100, 10, AXXPID_C(0.1));
        CHECK_NEAR(u, 30, AXXPID_C(1e-5));
    }

    /* Back to automatic: the first output must continue from 30, not leap to
     * kp*error = 180 (clamped to 100). */
    CHECK(axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC) == AXXPID_OK);
    u = axxpid_update(&pid, 100, 10, AXXPID_C(0.1));
    CHECK_MSG(axxpid_test_near(u, 39, AXXPID_C(0.5)),
              "transfer bumped to %.2f, expected ~39 (30 + ki*e*dt)",
              (double)u);
}

static void test_manual_output_is_clamped(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 0, 0, 0, 50);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
    (void)axxpid_set_manual_output(&pid, 999);

    CHECK_NEAR(axxpid_update(&pid, 10, 0, AXXPID_C(0.1)), 50, AXXPID_C(1e-5));
    CHECK(axxpid_is_saturated(&pid));
}

static void test_bumpless_tuning(void)
{
    axxpid_t pid;
    axxpid_real_t before;
    axxpid_real_t after;

    (void)axxpid_init(&pid, 2, 1, 0, -1000, 1000);
    CHECK(axxpid_set_bumpless_tuning(&pid, true) == AXXPID_OK);

    (void)axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    before = axxpid_get_output(&pid);

    /* Quadruple kp mid-flight. The proportional term jumps by 3*50 = 150, and
     * the integrator must absorb exactly that. */
    CHECK(axxpid_set_tunings(&pid, 8, 1, 0) == AXXPID_OK);
    after = axxpid_update(&pid, 100, 50, AXXPID_C(0.1));

    CHECK_MSG(axxpid_test_near(after, before + 5, AXXPID_C(0.5)),
              "retune bumped the output from %.2f to %.2f",
              (double)before, (double)after);

    /* Without the flag the same change steps the output by 150. */
    (void)axxpid_init(&pid, 2, 1, 0, -1000, 1000);
    (void)axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    before = axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    (void)axxpid_set_tunings(&pid, 8, 1, 0);
    after = axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    CHECK(after - before > 100);
}

/* Changing ki must never step the output, with or without the flag: the
 * integral is stored in output units. */
static void test_ki_change_never_bumps(void)
{
    axxpid_t pid;
    axxpid_real_t before;
    axxpid_real_t after;

    (void)axxpid_init(&pid, 1, 1, 0, -1000, 1000);
    (void)axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    before = axxpid_get_output(&pid);

    (void)axxpid_set_ki(&pid, 100);
    after = axxpid_update(&pid, 100, 50, AXXPID_C(0.0001));

    CHECK_NEAR(after, before, AXXPID_C(1.0));
}

static void test_changing_acting_clears_integral(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_update(&pid, 10, 0, AXXPID_C(1));
    CHECK(axxpid_get_i_term(&pid) > 0);

    /* The stored integral belongs to the old sense; keeping it would slam the
     * actuator the wrong way. */
    (void)axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE);
    (void)axxpid_update(&pid, 10, 10, AXXPID_C(1));
    CHECK(axxpid_get_i_term(&pid) == 0);
}

static void test_output_limits_clamp_live_state(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 10, 0, 0, 0, 1000);
    (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_output(&pid), 1000, AXXPID_C(1e-3));

    /* Tightening the range must pull the live output in immediately. */
    (void)axxpid_set_output_limits(&pid, 0, 50);
    CHECK_NEAR(axxpid_get_output(&pid), 50, AXXPID_C(1e-5));
}

/* -------------------------------------------------------------------------- */
/* Regressions                                                                */
/* -------------------------------------------------------------------------- */

/* The bumpless preload must use the output the actuator could actually have
 * been given. Preloading from an out-of-range manual value looks bumpless for
 * one sample and then pins the actuator at the limit for as long as it takes
 * the integrator to unwind the difference. */
static void test_transfer_preloads_from_the_reachable_output(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 1, 1, 0, 0, 50);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
    (void)axxpid_set_manual_output(&pid, 999);
    (void)axxpid_update(&pid, 10, 10, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_output(&pid), 50, AXXPID_C(1e-5));

    (void)axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC);
    (void)axxpid_update(&pid, 10, 20, AXXPID_C(0.1));

    /* Preloaded from 50, not from 999. */
    CHECK_MSG(axxpid_get_i_term(&pid) < 60,
              "integrator preloaded to %.1f from an unreachable manual output",
              (double)axxpid_get_i_term(&pid));

    /* With a standing error of -10 the output must start falling promptly,
     * not sit at the limit for a minute and a half. */
    for (i = 0; i < 50; ++i) {
        (void)axxpid_update(&pid, 10, 20, AXXPID_C(0.1));
    }
    CHECK_MSG(axxpid_get_output(&pid) < 50,
              "output still pinned at the limit after 5 s: %.2f",
              (double)axxpid_get_output(&pid));
}

/* A slew rate holds the output away from what the control law asked for just
 * as firmly as a hard limit does, so conditional integration has to see it.
 * Otherwise the default configuration winds up through every ramp. */
static void test_conditional_antiwindup_sees_the_slew_limit(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 1, 5, 0, 0, 100);
    (void)axxpid_set_output_slew_rate(&pid, 10);

    for (i = 0; i < 8; ++i) {
        (void)axxpid_update(&pid, 20, 0, AXXPID_C(0.1));
    }

    /* The actuator has only been allowed to reach 8 in 0.8 s. The integrator
     * must not have run off to the 80 an unrestricted output would justify. */
    CHECK_NEAR(axxpid_get_output(&pid), 8, AXXPID_C(0.5));
    CHECK_MSG(axxpid_get_i_term(&pid) < 20,
              "integrator wound up to %.1f behind a slew limit",
              (double)axxpid_get_i_term(&pid));
}

/* dt/Tt is a feedback gain. Above 1 the correction overshoots its own target
 * and rings; above 2 it diverges. Since dt is whatever the caller measured,
 * one late sample must not be able to destabilise the integrator. */
static void test_back_calculation_survives_a_large_dt(void)
{
    axxpid_t pid;
    axxpid_real_t worst = 0;
    int i;

    (void)axxpid_init(&pid, 1, 1, 0, 0, 10);
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(0.05));

    /* dt/Tt = 4: far past the point where an unbounded gain diverges. */
    for (i = 0; i < 200; ++i) {
        const axxpid_real_t value = axxpid_update(&pid, 100, 0, AXXPID_C(0.2));
        const axxpid_real_t magnitude = (value < 0) ? -value : value;

        if (magnitude > worst) {
            worst = magnitude;
        }
    }

    CHECK(worst <= 10);
    CHECK_MSG(axxpid_get_i_term(&pid) < 1000,
              "integrator diverged to %.1f", (double)axxpid_get_i_term(&pid));
    CHECK_NEAR(axxpid_get_output(&pid), 10, AXXPID_C(1e-4));
}

/* Capping dt protects the integrator, but the measurement moved over the
 * whole stall. Dividing that change by the cap would invent a rate of change
 * a hundred times the real one. */
static void test_dt_max_does_not_manufacture_a_derivative(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 1, -10000, 10000);
    (void)axxpid_set_dt_max(&pid, AXXPID_C(0.1));

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, AXXPID_C(0.1), AXXPID_C(0.1));

    /* The loop stalls for 10 s and the measurement drifts up by 5. The true
     * rate is 0.5/s; dividing by the 0.1 s cap would claim 50/s. */
    (void)axxpid_update(&pid, 0, AXXPID_C(5.1), 10);
    CHECK_MSG(axxpid_test_near(axxpid_get_d_term(&pid), 0, AXXPID_C(1.0)),
              "stall produced a derivative of %.2f",
              (double)axxpid_get_d_term(&pid));
}

/* Skipping a sample must not leave the next one differentiating a
 * two-interval change over one interval. */
static void test_rejected_sample_restarts_the_derivative(void)
{
    axxpid_t pid;
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t nan_value = zero / zero;

    (void)axxpid_init(&pid, 0, 0, 1, -10000, 10000);

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_d_term(&pid), -10, AXXPID_C(1e-3));

    /* One bad reading, then a good one on the same ramp. Carrying the stale
     * history would report twice the real slope. */
    (void)axxpid_update(&pid, 0, nan_value, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 3, AXXPID_C(0.1));
    CHECK_MSG(axxpid_test_near(axxpid_get_d_term(&pid), 0, AXXPID_C(1e-3)),
              "derivative after a rejected sample: %.3f (expected a restart)",
              (double)axxpid_get_d_term(&pid));
}

/* "Held at zero" has to mean held at zero, including under back-calculation. */
static void test_integral_band_holds_under_back_calculation(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 2, 1, 0, 0, 100);
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(0.5));
    (void)axxpid_set_integral_band(&pid, 75);

    for (i = 0; i < 5; ++i) {
        (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
        CHECK(axxpid_get_i_term(&pid) == 0);
    }

    /* And the stored value really is zero, not merely reported as zero: once
     * the band releases, integration starts from nothing. */
    (void)axxpid_update(&pid, 100, 30, AXXPID_C(0.1));
    CHECK_MSG(axxpid_get_i_term(&pid) > 0,
              "integral resumed from a hidden negative value: %.3f",
              (double)axxpid_get_i_term(&pid));
    CHECK_NEAR(axxpid_get_i_term(&pid), 7, AXXPID_C(0.5));
}

/* Same for the zero-setpoint reset. */
static void test_zero_setpoint_holds_under_back_calculation(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 2, 1, 0, 0, 100);
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(0.5));
    (void)axxpid_set_integral_reset_on_zero_setpoint(&pid, true);

    (void)axxpid_update(&pid, 50, 0, AXXPID_C(0.5));
    CHECK(axxpid_get_i_term(&pid) > 0);

    (void)axxpid_update(&pid, 0, 80, AXXPID_C(0.1));
    CHECK(axxpid_get_i_term(&pid) == 0);

    /* Resuming, the integrator starts from zero rather than from whatever
     * back-calculation quietly left behind. */
    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    CHECK(axxpid_get_i_term(&pid) == 0);
}

/* Mixing the two entry points must not integrate the same interval twice. */
static void test_mixing_update_and_update_at(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_set_sample_time(&pid, 100u, false);

    CHECK(axxpid_update_at(&pid, 10, 0, 1000u) == true);
    CHECK_NEAR(axxpid_get_i_term(&pid), 1, AXXPID_C(1e-3));

    /* Ten seconds of direct calls, which the ms clock knows nothing about. */
    (void)axxpid_update(&pid, 10, 0, 10);
    CHECK_NEAR(axxpid_get_i_term(&pid), 101, AXXPID_C(1e-2));

    /* Back to the clock: the next call must charge one nominal period, not
     * the ten seconds it has been away. */
    CHECK(axxpid_update_at(&pid, 10, 0, 11000u) == true);
    CHECK_MSG(axxpid_test_near(axxpid_get_i_term(&pid), 102, AXXPID_C(1e-2)),
              "integral %.2f - the gap was counted twice",
              (double)axxpid_get_i_term(&pid));
}

/* A rejected sample must not let update_at quietly drop the interval. */
static void test_update_at_holds_its_clock_on_a_bad_sample(void)
{
    axxpid_t pid;
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t nan_value = zero / zero;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_set_sample_time(&pid, 100u, false);

    CHECK(axxpid_update_at(&pid, 10, 0, 1000u) == true);
    CHECK(axxpid_update_at(&pid, 10, nan_value, 1100u) == false);

    /* The 100 ms the bad sample covered is folded into the next good one, so
     * 200 ms of error has been integrated in total, not 100. */
    CHECK(axxpid_update_at(&pid, 10, 0, 1200u) == true);
    CHECK_NEAR(axxpid_get_i_term(&pid), 3, AXXPID_C(1e-3));
}

/* A failed init must still leave a usable controller. An uninitialised
 * axxpid_t holds a garbage ff_fn that the next update would call. */
static void test_failed_init_leaves_a_safe_controller(void)
{
    axxpid_t pid;
    axxpid_config_t bad;

    memset(&pid, 0xA5, sizeof(pid));
    CHECK(axxpid_init(&pid, -2, 1, 0, 0, 100) == AXXPID_ERR_PARAM);

    CHECK(axxpid_get_kp(&pid) == 0);
    CHECK(axxpid_get_ki(&pid) == 0);
    CHECK(axxpid_get_kd(&pid) == 0);
    CHECK(axxpid_get_mode(&pid) == AXXPID_MODE_AUTOMATIC);
    CHECK(axxpid_get_acting(&pid) == AXXPID_ACTING_DIRECT);

    /* The important part: this must not jump through a garbage pointer. */
    CHECK(axxpid_update(&pid, 100, 0, AXXPID_C(0.1)) == 0);

    memset(&pid, 0x5A, sizeof(pid));
    (void)axxpid_config_default(&bad);
    bad.out_min = 10;
    bad.out_max = 1;
    CHECK(axxpid_init_config(&pid, &bad) == AXXPID_ERR_PARAM);
    CHECK(axxpid_update(&pid, 100, 0, AXXPID_C(0.1)) == 0);
}

/* A NaN gain compares false against every range test, so it has to be caught
 * explicitly or it reaches the integrator. */
static void test_non_finite_config_is_rejected(void)
{
    axxpid_t pid;
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t nan_value = zero / zero;
    const axxpid_real_t inf_value = AXXPID_C(1) / zero;

    CHECK(axxpid_init(&pid, nan_value, 0, 0, 0, 100) == AXXPID_ERR_PARAM);
    CHECK(axxpid_init(&pid, 0, inf_value, 0, 0, 100) == AXXPID_ERR_PARAM);
    CHECK(axxpid_init(&pid, 1, 1, 1, 0, inf_value) == AXXPID_ERR_PARAM);

    CHECK(axxpid_init(&pid, 1, 1, 1, 0, 100) == AXXPID_OK);
    CHECK(axxpid_set_integral(&pid, nan_value) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_manual_output(&pid, nan_value) == AXXPID_ERR_PARAM);
}

/* Setting the integral directly, for persistence and gain-schedule handoff. */
static void test_set_integral(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_update(&pid, 10, 0, 1);
    CHECK_NEAR(axxpid_get_i_term(&pid), 10, AXXPID_C(1e-4));

    CHECK(axxpid_set_integral(&pid, 55) == AXXPID_OK);
    CHECK(axxpid_set_integral(NULL, 0) == AXXPID_ERR_NULL);
    (void)axxpid_update(&pid, 10, 10, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_i_term(&pid), 55, AXXPID_C(1e-4));

    /* Clamped to the integral limits, like every other route to the term.
     * Read back through axxpid_get_integral: axxpid_get_i_term is the
     * snapshot from the last update and has not moved yet. */
    (void)axxpid_set_integral_limits(&pid, -20, 20);
    CHECK(axxpid_set_integral(&pid, 999) == AXXPID_OK);
    CHECK_NEAR(axxpid_get_integral(&pid), 20, AXXPID_C(1e-4));
    CHECK(axxpid_get_integral(NULL) == 0);
    (void)axxpid_update(&pid, 10, 10, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_i_term(&pid), 20, AXXPID_C(1e-4));

    /* Round-trips, which is the point: save it, restore it, carry on. */
    {
        const axxpid_real_t saved = axxpid_get_integral(&pid);

        (void)axxpid_reset(&pid);
        CHECK(axxpid_get_integral(&pid) == 0);
        CHECK(axxpid_set_integral(&pid, saved) == AXXPID_OK);
        CHECK_NEAR(axxpid_get_integral(&pid), saved, AXXPID_C(1e-6));
    }

    /* And the cheap way to clear just the integrator. */
    CHECK(axxpid_set_integral(&pid, 0) == AXXPID_OK);
    (void)axxpid_update(&pid, 10, 10, AXXPID_C(0.1));
    CHECK(axxpid_get_i_term(&pid) == 0);
}

/* Changing the derivative setpoint weight mid-flight must not produce a spike
 * from differencing across the change. */
static void test_changing_derivative_weight_does_not_kick(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 1, -100000, 100000);
    (void)axxpid_update(&pid, 500, 0, AXXPID_C(0.01));
    (void)axxpid_update(&pid, 500, 0, AXXPID_C(0.01));

    (void)axxpid_set_setpoint_weights(&pid, 1, 1);
    (void)axxpid_update(&pid, 500, 0, AXXPID_C(0.01));

    CHECK_MSG(axxpid_test_near(axxpid_get_d_term(&pid), 0, AXXPID_C(1e-3)),
              "weight change kicked the derivative to %.1f",
              (double)axxpid_get_d_term(&pid));
}

/* The telemetry struct carries the inputs too, so one call fills one frame. */
static void test_terms_carry_setpoint_and_measurement(void)
{
    axxpid_t pid;
    axxpid_terms_t terms;

    (void)axxpid_init(&pid, 1, 0, 0, -1000, 1000);
    (void)axxpid_update(&pid, 42, AXXPID_C(17.5), AXXPID_C(0.1));

    CHECK(axxpid_get_terms(&pid, &terms) == AXXPID_OK);
    CHECK_NEAR(terms.setpoint, 42, AXXPID_C(1e-5));
    CHECK_NEAR(terms.measurement, AXXPID_C(17.5), AXXPID_C(1e-5));
    CHECK_NEAR(terms.error, AXXPID_C(24.5), AXXPID_C(1e-5));
}

static const axxpid_test_case_t tests[] = {
    {"proportional term", test_proportional},
    {"setpoint weight b", test_setpoint_weight_b},
    {"derivative on measurement", test_derivative_on_measurement},
    {"no derivative kick on setpoint step", test_no_derivative_kick},
    {"derivative filter by tau", test_derivative_filter_tau},
    {"derivative filter by N", test_derivative_filter_n},
    {"derivative filter follows real dt", test_derivative_filter_tracks_dt},
    {"integral uses real elapsed time", test_integral_accumulates_with_real_dt},
    {"integral limits", test_integral_limits},
    {"conditional-integration anti-windup", test_antiwindup_conditional},
    {"conditional integration never latches",
     test_conditional_integration_never_latches},
    {"conditional integration with offset limits",
     test_conditional_integration_with_offset_limits},
    {"back-calculation anti-windup", test_antiwindup_back_calculation},
    {"integral engagement band", test_integral_band},
    {"asymmetric integral gain", test_integral_asymmetry},
    {"integral reset at zero setpoint", test_integral_reset_on_zero_setpoint},
    {"output limits", test_output_limits},
    {"output slew rate", test_output_slew_rate},
    {"error deadband", test_deadband},
    {"feed-forward bias and setpoint gain", test_feedforward_bias_and_gain},
    {"velocity feed-forward", test_feedforward_velocity},
    {"feed-forward callback", test_feedforward_callback},
    {"feed-forward is inside anti-windup",
     test_feedforward_participates_in_antiwindup},
    {"reverse acting", test_reverse_acting},
    {"reverse acting derivative", test_reverse_acting_derivative},
    {"manual mode and bumpless transfer",
     test_manual_mode_and_bumpless_transfer},
    {"manual output is clamped", test_manual_output_is_clamped},
    {"bumpless retuning", test_bumpless_tuning},
    {"changing ki never bumps", test_ki_change_never_bumps},
    {"changing acting clears the integral", test_changing_acting_clears_integral},
    {"new output limits clamp live state", test_output_limits_clamp_live_state},

    /* Regressions for defects found in review. */
    {"transfer preloads from the reachable output",
     test_transfer_preloads_from_the_reachable_output},
    {"conditional anti-windup sees the slew limit",
     test_conditional_antiwindup_sees_the_slew_limit},
    {"back-calculation survives a large dt",
     test_back_calculation_survives_a_large_dt},
    {"dt_max does not manufacture a derivative",
     test_dt_max_does_not_manufacture_a_derivative},
    {"rejected sample restarts the derivative",
     test_rejected_sample_restarts_the_derivative},
    {"integral band holds under back-calculation",
     test_integral_band_holds_under_back_calculation},
    {"zero setpoint holds under back-calculation",
     test_zero_setpoint_holds_under_back_calculation},
    {"mixing update and update_at", test_mixing_update_and_update_at},
    {"update_at holds its clock on a bad sample",
     test_update_at_holds_its_clock_on_a_bad_sample},
    {"failed init leaves a safe controller",
     test_failed_init_leaves_a_safe_controller},
    {"non-finite configuration is rejected",
     test_non_finite_config_is_rejected},
    {"setting the integral directly", test_set_integral},
    {"changing the derivative weight does not kick",
     test_changing_derivative_weight_does_not_kick},
    {"terms carry setpoint and measurement",
     test_terms_carry_setpoint_and_measurement},
};

AXXPID_TEST_MAIN("features", tests)
