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
    /* Opt out of the derived integral limits, so this measures the
     * anti-windup strategy and nothing else. */
    (void)axxpid_set_integral_limits(&pid_open, -AXXPID_UNLIMITED,
                                     AXXPID_UNLIMITED);

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
    int i;

    /* kp is zero on purpose. The integrator alone has to hold the output at
     * its limit, so the sample where the error reverses is genuinely
     * saturated *and* genuinely wants to integrate downwards - which is the
     * one case a naive anti-windup rule refuses, latching the loop forever.
     * With any proportional term the P contribution masks it, which is how
     * the earlier version of this test came to prove nothing. */
    (void)axxpid_init(&pid, 0, 5, 0, 0, 100);
    (void)axxpid_set_integral_limits(&pid, -200, 200);
    CHECK(axxpid_set_integral(&pid, 150) == AXXPID_OK);

    (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_output(&pid), 100, AXXPID_C(1e-4));
    CHECK_MSG(axxpid_is_saturated(&pid), "setup is not saturated");

    /* Error now negative, output still pinned at the top. The step reduces
     * the saturation, so it must be taken - every sample, monotonically. */
    for (i = 0; i < 40; ++i) {
        const axxpid_real_t previous = axxpid_get_integral(&pid);

        (void)axxpid_update(&pid, 0, 2, AXXPID_C(0.1));
        CHECK_MSG(axxpid_get_integral(&pid) < previous,
                  "integrator latched at %.2f while saturated",
                  (double)axxpid_get_integral(&pid));
        if (i < 8) {
            /* Still pinned, so this is the saturated case, not a free run. */
            CHECK_NEAR(axxpid_get_output(&pid), 100, AXXPID_C(1e-4));
        }
    }
    /* 40 samples at ki*e*dt = 5*2*0.1 = 1 per sample, from 150. */
    CHECK_NEAR(axxpid_get_integral(&pid), 110, AXXPID_C(0.5));
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
    (void)axxpid_set_integral_limits(&pid, -AXXPID_UNLIMITED,
                                     AXXPID_UNLIMITED);
    CHECK(axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(0.5)) == AXXPID_OK);

    for (i = 0; i < 200; ++i) {
        (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
    }

    /* At the fixed point the bleed matches the charge exactly:
     * ki*e*dt = (dt/Tt)*(u_sat - u_unsat), so u_unsat settles at
     * out_max + ki*e*Tt = 10 + 5*100*0.5 = 260, i.e. I settles near
     * 260 - kp*e = 260 - 100 = 160. Bounded, not unbounded. */
    /* The fixed point is exact: the charge ki*e*dt and the bleed
     * (dt/Tt)*(u_sat - u_unsat) cancel, giving u_unsat = out_max + ki*e*Tt =
     * 10 + 5*100*0.5 = 260, so I = 260 - kp*e = 260 - 100 = 160. */
    CHECK_NEAR(axxpid_get_i_term(&pid), 160, AXXPID_C(1.0));

    /* Reverse the error and it comes off the limit in a handful of samples
     * rather than grinding through the accumulated wind-up. */
    for (i = 0; i < 10; ++i) {
        (void)axxpid_update(&pid, 0, 50, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_output(&pid), 0, AXXPID_C(1e-4));
    CHECK(axxpid_get_i_term(&pid) < 50);
}

/* The integral engagement band: park the integrator while the process is
 * still a long way below the setpoint. */
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

/* Asymmetric integral gain: unwind faster than you wind up. */
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

/* A range check of the form `x < 0 || x > 1` does not reject NaN, because
 * every comparison against NaN is false. Each setter that takes a real number
 * must test for finiteness separately, or a bad value walks past the guard
 * and reaches the control law on the next update. This sweeps all of them. */
static void test_no_setter_accepts_a_non_finite_value(void)
{
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t nan_value = zero / zero;
    const axxpid_real_t inf_value = AXXPID_C(1) / zero;
    size_t i;

    for (i = 0; i < 2u; ++i) {
        const axxpid_real_t bad = (i == 0u) ? nan_value : inf_value;
        axxpid_t pid;

        (void)axxpid_init(&pid, 1, 1, 1, 0, 100);

        CHECK(axxpid_set_tunings(&pid, bad, 1, 1) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_tunings(&pid, 1, bad, 1) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_tunings(&pid, 1, 1, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_tunings_standard(&pid, bad, 1, 1) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_tunings_standard(&pid, 1, bad, 1) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_tunings_standard(&pid, 1, 1, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_kp(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_ki(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_kd(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_setpoint_weights(&pid, bad, 0) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_setpoint_weights(&pid, 0, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_output_limits(&pid, 0, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_output_limits(&pid, bad, 1) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_output_slew_rate(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_integral(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_integral_limits(&pid, bad, 1) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_integral_limits(&pid, 0, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_integral_band(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_integral_overshoot(&pid, bad, 0) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_integral_overshoot(&pid, 1, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_derivative_filter(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_derivative_filter_tau(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_deadband(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_feedforward_bias(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_feedforward_gains(&pid, bad, 0) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_feedforward_gains(&pid, 0, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_manual_output(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_dt_max(&pid, bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                    bad) == AXXPID_ERR_PARAM);
        CHECK(axxpid_reset_to(&pid, bad) == AXXPID_ERR_PARAM);

        /* Having rejected every one of those, the controller must still run
         * normally rather than having quietly stored something poisonous. */
        {
            const axxpid_real_t u = axxpid_update(&pid, 50, 10, AXXPID_C(0.1));

            CHECK_MSG(u == u, "output is NaN after rejected setters");
            CHECK(pid.integral == pid.integral);
            CHECK(u > 0);
        }
    }
}

/* The derivative weight is the subtle one: a bad value does not show up on
 * the update that follows, because that update restarts the derivative
 * history. It bites on the one after. */
static void test_non_finite_weight_does_not_bite_one_sample_later(void)
{
    axxpid_t pid;
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t nan_value = zero / zero;
    int i;

    (void)axxpid_init(&pid, 1, 1, 1, 0, 100);
    CHECK(axxpid_set_setpoint_weights(&pid, 0, nan_value) == AXXPID_ERR_PARAM);

    for (i = 0; i < 4; ++i) {
        const axxpid_real_t u = axxpid_update(&pid, 50, 10, AXXPID_C(0.1));

        CHECK_MSG(u == u, "output went NaN on update %d", i);
    }
}

/* A non-finite integral time used to fail the `ti > 0` test and silently turn
 * integral action off, which is far worse than an error return: the loop then
 * runs with a standing offset and nothing says why. */
static void test_non_finite_integral_time_is_reported(void)
{
    axxpid_t pid;
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t nan_value = zero / zero;

    (void)axxpid_init(&pid, 2, 5, 0, 0, 100);
    CHECK(axxpid_set_tunings_standard(&pid, 1, nan_value, 0) ==
          AXXPID_ERR_PARAM);

    /* Rejected, so the original integral gain is still in place. */
    CHECK(axxpid_get_ki(&pid) == 5);
}

/* -------------------------------------------------------------------------- */
/* Gaps found by mutation testing                                             */
/* -------------------------------------------------------------------------- */

/* The filter time constant is Tf = (kd/kp)/N. Every earlier test used
 * kp == kd, where inverting that ratio makes no difference at all. */
static void test_derivative_filter_with_unequal_gains(void)
{
    axxpid_t pid;

    /* kp=2, kd=8, N=4 -> Tf = (8/2)/4 = 1.0 s. With dt = 0.1 the filter
     * passes alpha = 0.1/1.1 of each new sample. A step of 1 over 0.1 s is a
     * raw rate of -10, so D = 8 * (-10) * (0.1/1.1) = -7.2727. Inverting the
     * ratio to (kp/kd)/N would give Tf = 0.0625 and D = -49.5. */
    (void)axxpid_init(&pid, 2, 0, 8, -1000, 1000);
    (void)axxpid_set_derivative_filter(&pid, 4);

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_d_term(&pid), AXXPID_C(-7.27273), AXXPID_C(1e-3));
}

/* With kp = 0 there is no derivative time to divide by, so the N-based filter
 * cannot be computed and must simply not engage. */
static void test_derivative_filter_with_zero_kp(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 1, -1000, 1000);
    (void)axxpid_set_derivative_filter(&pid, 10);

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.1));

    /* Unfiltered: D = 1 * -(1/0.1) = -10. A divide by zero here would give
     * tau = inf, alpha = 0, and D would silently be 0 forever. */
    CHECK_NEAR(axxpid_get_d_term(&pid), -10, AXXPID_C(1e-4));
}

/* Bumpless retuning has to compensate the derivative term as well as the
 * proportional one. The earlier test used kd = 0 and could not tell. */
static void test_bumpless_retuning_compensates_the_derivative(void)
{
    axxpid_t pid;
    axxpid_real_t before;
    axxpid_real_t after;

    (void)axxpid_init(&pid, 2, 1, 4, -10000, 10000);
    (void)axxpid_set_bumpless_tuning(&pid, true);

    /* A steady ramp in the measurement, so the D term is large and constant. */
    (void)axxpid_update(&pid, 50, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 50, 1, AXXPID_C(0.1));
    before = axxpid_update(&pid, 50, 2, AXXPID_C(0.1));

    /* Double kd. The D term doubles; the integral must absorb the change. */
    CHECK(axxpid_set_tunings(&pid, 2, 1, 8) == AXXPID_OK);
    after = axxpid_update(&pid, 50, 3, AXXPID_C(0.1));

    CHECK_MSG(axxpid_test_near(after, before, AXXPID_C(5.0)),
              "retune with kd active bumped the output from %.2f to %.2f",
              (double)before, (double)after);
}

/* Manual mode must keep the derivative history current, or the first
 * automatic sample differentiates across the whole manual period. */
static void test_transfer_with_derivative_active(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 0, 0, 1, -10000, 10000);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
    (void)axxpid_set_manual_output(&pid, 20);

    /* The measurement ramps at 10 per second throughout. */
    for (i = 0; i < 20; ++i) {
        (void)axxpid_update(&pid, 0, (axxpid_real_t)i, AXXPID_C(0.1));
    }

    (void)axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC);
    (void)axxpid_update(&pid, 0, 20, AXXPID_C(0.1));

    /* The ramp never changed, so D must still be -kd * 10 = -10. If manual
     * mode had stopped tracking, the first automatic sample would difference
     * against a two-second-old measurement. */
    CHECK_NEAR(axxpid_get_d_term(&pid), -10, AXXPID_C(1e-3));
}

/* Switching to manual with no explicit value must hold the output where
 * automatic left it. */
static void test_manual_mode_captures_the_live_output(void)
{
    axxpid_t pid;
    axxpid_real_t automatic;

    (void)axxpid_init(&pid, 2, 0, 0, 0, 100);
    automatic = axxpid_update(&pid, 30, 10, AXXPID_C(0.1));
    CHECK_NEAR(automatic, 40, AXXPID_C(1e-4));

    CHECK(axxpid_set_mode(&pid, AXXPID_MODE_MANUAL) == AXXPID_OK);
    CHECK_MSG(axxpid_test_near(axxpid_get_output(&pid), automatic,
                               AXXPID_C(1e-4)),
              "going manual moved the output to %.2f",
              (double)axxpid_get_output(&pid));
    CHECK_NEAR(axxpid_update(&pid, 30, 0, AXXPID_C(0.1)), 40, AXXPID_C(1e-4));

    /* And setting a manual value takes effect immediately, before any
     * further update. */
    CHECK(axxpid_set_manual_output(&pid, 12) == AXXPID_OK);
    CHECK_NEAR(axxpid_get_output(&pid), 12, AXXPID_C(1e-5));

    /* Asking for manual again must not discard that value. */
    CHECK(axxpid_set_mode(&pid, AXXPID_MODE_MANUAL) == AXXPID_OK);
    CHECK_NEAR(axxpid_update(&pid, 30, 0, AXXPID_C(0.1)), 12, AXXPID_C(1e-5));
}

/* Manual to automatic with no update in between still has to preload. */
static void test_transfer_without_an_intervening_update(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 2, 0, 0, 0, 200);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
    (void)axxpid_set_manual_output(&pid, 55);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC);

    /* No manual-mode update ever ran, so the preload on this first automatic
     * sample is the only thing that can make it bumpless. */
    CHECK_NEAR(axxpid_update(&pid, 100, 10, AXXPID_C(0.1)), 55,
               AXXPID_C(1e-3));
}

/* Reverse acting has to apply to the setpoint-weighted part of the
 * proportional term too, not only to the error. */
static void test_reverse_acting_with_setpoint_weight(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 3, 0, 0, -1000, 1000);
    (void)axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE);
    (void)axxpid_set_setpoint_weights(&pid, AXXPID_C(0.5), 0);

    /* P = kp * dir * (b*sp - pv) = 3 * -1 * (0.5*10 - 14) = +27. Dropping the
     * direction from the (1-b)*sp part instead gives -3. */
    CHECK_NEAR(axxpid_update(&pid, 10, 14, AXXPID_C(0.1)), 27, AXXPID_C(1e-4));
}

/* The velocity feed-forward must restart across a capped sample for the same
 * reason the derivative does. */
static void test_velocity_feedforward_across_a_stall(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 0, -100000, 100000);
    (void)axxpid_set_feedforward_gains(&pid, 0, 1);
    (void)axxpid_set_dt_max(&pid, AXXPID_C(0.1));

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 1, 0, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_ff_term(&pid), 10, AXXPID_C(1e-3));

    /* Ten seconds pass and the setpoint moves by 100. The real rate is 10 per
     * second; dividing by the 0.1 s cap would claim 1000. */
    (void)axxpid_update(&pid, 101, 0, 10);
    CHECK_MSG(axxpid_test_near(axxpid_get_ff_term(&pid), 0, AXXPID_C(1e-3)),
              "stall produced a velocity feed-forward of %.2f",
              (double)axxpid_get_ff_term(&pid));
}

/* The saturation flag must report a slew limit too, not only the clamp. */
static void test_saturated_reports_a_slew_limit_alone(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 0, 0, -1000, 1000);
    (void)axxpid_set_output_slew_rate(&pid, 10);

    /* The law wants 100, well inside the limits, but the rate allows 1. */
    CHECK_NEAR(axxpid_update(&pid, 100, 0, AXXPID_C(0.1)), 1, AXXPID_C(1e-4));
    CHECK_MSG(axxpid_is_saturated(&pid),
              "output was rate-limited but not reported as saturated");
}

/* Setters have to bring the live state with them. */
static void test_setters_move_the_live_state(void)
{
    axxpid_t pid;

    /* Tightening the integral limits pulls the stored integral in. */
    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_update(&pid, 55, 0, 1);
    CHECK_NEAR(axxpid_get_integral(&pid), 55, AXXPID_C(1e-4));
    CHECK(axxpid_set_integral_limits(&pid, -20, 20) == AXXPID_OK);
    CHECK_NEAR(axxpid_get_integral(&pid), 20, AXXPID_C(1e-4));

    /* reset_to clamps to the output limits. */
    (void)axxpid_init(&pid, 1, 0, 0, 0, 50);
    CHECK(axxpid_reset_to(&pid, 999) == AXXPID_OK);
    CHECK_NEAR(axxpid_get_output(&pid), 50, AXXPID_C(1e-5));
}

/* Equal integral limits pin the integral, which is the documented way to turn
 * integral action off without touching ki. */
static void test_equal_integral_limits_pin_the_integral(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 1, 10, 0, -1000, 1000);
    CHECK(axxpid_set_integral_limits(&pid, 7, 7) == AXXPID_OK);

    for (i = 0; i < 20; ++i) {
        (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
        CHECK_NEAR(axxpid_get_integral(&pid), 7, AXXPID_C(1e-5));
    }
    for (i = 0; i < 20; ++i) {
        (void)axxpid_update(&pid, -100, 0, AXXPID_C(0.1));
        CHECK_NEAR(axxpid_get_integral(&pid), 7, AXXPID_C(1e-5));
    }
}

/* axxpid_init_config validates the whole struct, so every field it checks
 * needs to actually be checked. */
static void test_init_config_rejects_every_bad_field(void)
{
    axxpid_t pid;
    axxpid_config_t cfg;
    size_t i;

    for (i = 0; i < 11u; ++i) {
        CHECK(axxpid_config_default(&cfg) == AXXPID_OK);
        cfg.out_min = 0;
        cfg.out_max = 100;

        switch (i) {
            case 0u: cfg.kp = -1; break;
            case 1u: cfg.ki = -1; break;
            case 2u: cfg.kd = -1; break;
            case 3u: cfg.out_max = -1; break;
            case 4u: cfg.integral_min = 10; cfg.integral_max = 1; break;
            case 5u: cfg.deadband = -1; break;
            case 6u: cfg.setpoint_weight_b = 2; break;
            case 7u: cfg.setpoint_weight_c = -1; break;
            case 8u: cfg.dt_max = 0; break;
            case 9u: cfg.out_slew_rate = -1; break;
            default: cfg.sample_time_ms = 0u; break;
        }

        CHECK_MSG(axxpid_init_config(&pid, &cfg) == AXXPID_ERR_PARAM,
                  "init_config accepted bad field %u", (unsigned)i);
    }

    /* And the happy path really does take the values given. */
    CHECK(axxpid_config_default(&cfg) == AXXPID_OK);
    cfg.kp = 3;
    cfg.out_min = -5;
    cfg.out_max = 55;
    cfg.deadband = 2;
    cfg.setpoint_weight_b = AXXPID_C(0.5);
    CHECK(axxpid_init_config(&pid, &cfg) == AXXPID_OK);
    CHECK(axxpid_get_kp(&pid) == 3);
    CHECK(pid.cfg.out_max == 55);
    CHECK(pid.cfg.deadband == 2);
}

/* Exact threshold values, which no other test uses. */
static void test_boundary_values(void)
{
    axxpid_t pid;

    /* The overshoot gain applies strictly below the threshold, so an error
     * exactly on it uses the plain gain. */
    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_set_integral_overshoot(&pid, 10, -5);
    (void)axxpid_update(&pid, 0, 5, 1);
    CHECK_NEAR(axxpid_get_integral(&pid), -5, AXXPID_C(1e-4));

    /* The band applies strictly above, so an error exactly on it integrates. */
    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_set_integral_band(&pid, 10);
    (void)axxpid_update(&pid, 10, 0, 1);
    CHECK_NEAR(axxpid_get_integral(&pid), 10, AXXPID_C(1e-4));

    /* A dt exactly at dt_max is not a stall, so the derivative still runs. */
    (void)axxpid_init(&pid, 0, 0, 1, -1000, 1000);
    (void)axxpid_set_dt_max(&pid, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_d_term(&pid), -10, AXXPID_C(1e-4));
}

/* One wild-but-finite reading must not leave the integrator somewhere it can
 * never come back from. */
static void test_survives_an_implausible_reading(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 8, 2, AXXPID_C(0.5), 0, 100);
    (void)axxpid_update(&pid, 50, 49, AXXPID_C(0.01));

    /* A 32-bit garbage ADC word. It is finite, so it passes the NaN guard. */
    (void)axxpid_update(&pid, 50, AXXPID_C(4294967295.0), AXXPID_C(0.01));
    CHECK_MSG(axxpid_get_integral(&pid) < 100000,
              "one bad reading put the integral at %.4g",
              (double)axxpid_get_integral(&pid));

    /* With the measurement above setpoint the output must reach zero, and in
     * a sane time rather than after hours of unwinding. */
    for (i = 0; i < 10000; ++i) {
        (void)axxpid_update(&pid, 50, 60, AXXPID_C(0.01));
    }
    CHECK_MSG(axxpid_get_output(&pid) < 1,
              "still driving %.2f a hundred seconds after a bad reading",
              (double)axxpid_get_output(&pid));
}

/* The mirror of the latching test, against the lower limit. Both halves of
 * the anti-windup condition need their own case: one of them can be broken
 * while the other still passes. */
static void test_conditional_integration_releases_at_the_lower_limit(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 0, 5, 0, 0, 100);
    (void)axxpid_set_integral_limits(&pid, -200, 200);
    CHECK(axxpid_set_integral(&pid, -150) == AXXPID_OK);

    (void)axxpid_update(&pid, 0, 10, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_output(&pid), 0, AXXPID_C(1e-4));
    CHECK_MSG(axxpid_is_saturated(&pid), "setup is not saturated");

    /* Error now positive, output still pinned at the bottom. The step raises
     * the integral towards the usable range, so it must be taken. */
    for (i = 0; i < 40; ++i) {
        const axxpid_real_t previous = axxpid_get_integral(&pid);

        (void)axxpid_update(&pid, 2, 0, AXXPID_C(0.1));
        CHECK_MSG(axxpid_get_integral(&pid) > previous,
                  "integrator latched at %.2f against the lower limit",
                  (double)axxpid_get_integral(&pid));
        if (i < 8) {
            CHECK_NEAR(axxpid_get_output(&pid), 0, AXXPID_C(1e-4));
        }
    }
    CHECK_NEAR(axxpid_get_integral(&pid), -110, AXXPID_C(0.5));
}

/* The manual output is stored as the caller gave it, so narrowing and then
 * widening the output limits restores the original rather than the clipped
 * copy. */
static void test_manual_output_survives_a_limit_change(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 0, 0, 0, 100);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
    (void)axxpid_set_manual_output(&pid, 150);

    CHECK(axxpid_set_output_limits(&pid, 0, 50) == AXXPID_OK);
    CHECK_NEAR(axxpid_update(&pid, 10, 0, AXXPID_C(0.1)), 50, AXXPID_C(1e-5));

    CHECK(axxpid_set_output_limits(&pid, 0, 200) == AXXPID_OK);
    CHECK_MSG(axxpid_test_near(axxpid_update(&pid, 10, 0, AXXPID_C(0.1)), 150,
                               AXXPID_C(1e-4)),
              "manual output came back as %.1f, not the 150 that was set",
              (double)axxpid_get_output(&pid));
}

/* No single sample may throw the integral a long way, however implausible the
 * measurement. This exercises the ordinary integral path, not the transfer
 * preload. */
static void test_one_bad_sample_cannot_move_the_integral_far(void)
{
    axxpid_t pid;
    axxpid_real_t before;
    axxpid_real_t after;

    (void)axxpid_init(&pid, 0, 50, 0, 0, 100);
    (void)axxpid_set_integral_limits(&pid, -AXXPID_UNLIMITED,
                                     AXXPID_UNLIMITED);
    /* Anti-windup off and limits removed, so the per-sample bound is the only
     * thing standing between one bad reading and the integrator. */
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_NONE, 1);
    (void)axxpid_update(&pid, 50, 49, AXXPID_C(0.01));
    before = axxpid_get_integral(&pid);

    /* A 32-bit garbage ADC word. ki*error*dt would be about -2.1e9. */
    (void)axxpid_update(&pid, 50, AXXPID_C(4294967295.0), AXXPID_C(0.01));
    after = axxpid_get_integral(&pid);

    /* The output range is 100 wide, so one sample may move the integral by at
     * most 1000. */
    CHECK_MSG(((before - after) < 1001) && ((before - after) > 0),
              "one bad sample moved the integral by %.4g",
              (double)(before - after));
}

/* Zero is not necessarily an output this actuator can produce. */
static void test_initial_output_is_inside_the_range(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 0, 0, 20, 80);
    CHECK_NEAR(axxpid_get_output(&pid), 20, AXXPID_C(1e-5));

    /* With a slew rate the starting point is visible in the first output: a
     * controller that starts from zero ramps up through values the caller was
     * promised would never appear. */
    (void)axxpid_set_output_slew_rate(&pid, 10);
    CHECK_MSG(axxpid_update(&pid, 100, 0, AXXPID_C(0.01)) >= 20,
              "first output was %.3f, below the configured minimum of 20",
              (double)axxpid_get_output(&pid));

    /* Same after a reset, and for a range entirely below zero. */
    (void)axxpid_reset(&pid);
    CHECK_NEAR(axxpid_get_output(&pid), 20, AXXPID_C(1e-5));

    (void)axxpid_init(&pid, 1, 0, 0, -100, -10);
    CHECK_NEAR(axxpid_get_output(&pid), -10, AXXPID_C(1e-5));
}

static axxpid_real_t broken_ff(axxpid_real_t setpoint,
                               axxpid_real_t measurement,
                               void *user)
{
    volatile axxpid_real_t zero = 0;

    (void)setpoint;
    (void)measurement;
    (void)user;
    return AXXPID_C(1) / zero; /* a divide gone wrong in a lookup table */
}

/* The feed-forward hook is user code summed straight into the output and the
 * integrator, so a bad return value is exactly as damaging as a bad sensor
 * reading and has to be treated the same way. */
static void test_broken_feedforward_hook_is_rejected(void)
{
    axxpid_t pid;
    axxpid_real_t good;
    int i;

    (void)axxpid_init(&pid, 1, 1, 0, 0, 100);
    good = axxpid_update(&pid, 50, 40, AXXPID_C(0.1));

    (void)axxpid_set_feedforward_fn(&pid, broken_ff, NULL);
    CHECK_MSG(axxpid_update(&pid, 50, 40, AXXPID_C(0.1)) == good,
              "a broken feed-forward hook changed the output");
    CHECK(axxpid_get_integral(&pid) == axxpid_get_integral(&pid));

    /* And the controller recovers once the hook is removed. */
    (void)axxpid_set_feedforward_fn(&pid, NULL, NULL);
    for (i = 0; i < 5; ++i) {
        const axxpid_real_t u = axxpid_update(&pid, 50, 40, AXXPID_C(0.1));

        CHECK_MSG(u == u, "output is NaN after the hook was removed");
    }
    CHECK(axxpid_get_output(&pid) > 0);
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
    {"no setter accepts a non-finite value",
     test_no_setter_accepts_a_non_finite_value},
    {"non-finite weight does not bite one sample later",
     test_non_finite_weight_does_not_bite_one_sample_later},
    {"non-finite integral time is reported",
     test_non_finite_integral_time_is_reported},
    {"setting the integral directly", test_set_integral},
    {"changing the derivative weight does not kick",
     test_changing_derivative_weight_does_not_kick},
    {"terms carry setpoint and measurement",
     test_terms_carry_setpoint_and_measurement},
    {"derivative filter with unequal gains",
     test_derivative_filter_with_unequal_gains},
    {"derivative filter with zero kp", test_derivative_filter_with_zero_kp},
    {"bumpless retuning compensates the derivative",
     test_bumpless_retuning_compensates_the_derivative},
    {"transfer with the derivative active",
     test_transfer_with_derivative_active},
    {"manual mode captures the live output",
     test_manual_mode_captures_the_live_output},
    {"transfer without an intervening update",
     test_transfer_without_an_intervening_update},
    {"reverse acting with setpoint weight",
     test_reverse_acting_with_setpoint_weight},
    {"velocity feed-forward across a stall",
     test_velocity_feedforward_across_a_stall},
    {"saturated reports a slew limit alone",
     test_saturated_reports_a_slew_limit_alone},
    {"setters move the live state", test_setters_move_the_live_state},
    {"equal integral limits pin the integral",
     test_equal_integral_limits_pin_the_integral},
    {"init_config rejects every bad field",
     test_init_config_rejects_every_bad_field},
    {"boundary values", test_boundary_values},
    {"survives an implausible reading", test_survives_an_implausible_reading},
    {"conditional integration releases at the lower limit",
     test_conditional_integration_releases_at_the_lower_limit},
    {"manual output survives a limit change",
     test_manual_output_survives_a_limit_change},
    {"one bad sample cannot move the integral far",
     test_one_bad_sample_cannot_move_the_integral_far},
    {"initial output is inside the range",
     test_initial_output_is_inside_the_range},
    {"broken feed-forward hook is rejected",
     test_broken_feedforward_hook_is_rejected},
};

AXXPID_TEST_MAIN("features", tests)
