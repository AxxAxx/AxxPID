/**
 * @file  test_terms.c
 * @brief The P, I, D and feed-forward terms themselves.
 *
 * One test per piece of the control law, each checked against
 * arithmetic worked out by hand from the difference equations in
 * docs/CONTROL_LAW.md - never against a value recorded from a
 * previous run.
 */

#include "axxpid_test.h"

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

static const axxpid_test_case_t tests[] = {
    {"proportional term", test_proportional},
    {"setpoint weight b", test_setpoint_weight_b},
    {"derivative on measurement", test_derivative_on_measurement},
    {"no derivative kick on setpoint step", test_no_derivative_kick},
    {"derivative filter by tau", test_derivative_filter_tau},
    {"derivative filter by N", test_derivative_filter_n},
    {"derivative filter follows real dt",
     test_derivative_filter_tracks_dt},
    {"integral uses real elapsed time",
     test_integral_accumulates_with_real_dt},
    {"error deadband", test_deadband},
    {"feed-forward bias and setpoint gain",
     test_feedforward_bias_and_gain},
    {"velocity feed-forward", test_feedforward_velocity},
    {"feed-forward callback", test_feedforward_callback},
    {"reverse acting", test_reverse_acting},
    {"reverse acting derivative", test_reverse_acting_derivative},
    {"changing the derivative weight does not kick",
     test_changing_derivative_weight_does_not_kick},
    {"terms carry setpoint and measurement",
     test_terms_carry_setpoint_and_measurement},
    {"derivative filter with unequal gains",
     test_derivative_filter_with_unequal_gains},
    {"derivative filter with zero kp",
     test_derivative_filter_with_zero_kp},
    {"reverse acting with setpoint weight",
     test_reverse_acting_with_setpoint_weight},
    {"velocity feed-forward across a stall",
     test_velocity_feedforward_across_a_stall},
    {"boundary values", test_boundary_values},
};

AXXPID_TEST_MAIN("terms", tests)
