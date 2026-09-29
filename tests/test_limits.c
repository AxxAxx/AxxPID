/**
 * @file  test_limits.c
 * @brief Everything that bounds the controller.
 *
 * Output limits, the slew rate, the integral clamp, and the three
 * anti-windup strategies - including the cases where a naive
 * implementation latches the integrator and never recovers.
 */

#include "axxpid_test.h"

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

static const axxpid_test_case_t tests[] = {
    {"integral limits", test_integral_limits},
    {"conditional-integration anti-windup",
     test_antiwindup_conditional},
    {"conditional integration never latches",
     test_conditional_integration_never_latches},
    {"conditional integration with offset limits",
     test_conditional_integration_with_offset_limits},
    {"back-calculation anti-windup",
     test_antiwindup_back_calculation},
    {"integral engagement band", test_integral_band},
    {"asymmetric integral gain", test_integral_asymmetry},
    {"integral reset at zero setpoint",
     test_integral_reset_on_zero_setpoint},
    {"output limits", test_output_limits},
    {"output slew rate", test_output_slew_rate},
    {"feed-forward is inside anti-windup",
     test_feedforward_participates_in_antiwindup},
    {"conditional anti-windup sees the slew limit",
     test_conditional_antiwindup_sees_the_slew_limit},
    {"back-calculation survives a large dt",
     test_back_calculation_survives_a_large_dt},
    {"integral band holds under back-calculation",
     test_integral_band_holds_under_back_calculation},
    {"zero setpoint holds under back-calculation",
     test_zero_setpoint_holds_under_back_calculation},
    {"setting the integral directly", test_set_integral},
    {"saturated reports a slew limit alone",
     test_saturated_reports_a_slew_limit_alone},
    {"equal integral limits pin the integral",
     test_equal_integral_limits_pin_the_integral},
    {"survives an implausible reading",
     test_survives_an_implausible_reading},
    {"conditional integration releases at the lower limit",
     test_conditional_integration_releases_at_the_lower_limit},
    {"one bad sample cannot move the integral far",
     test_one_bad_sample_cannot_move_the_integral_far},
    {"initial output is inside the range",
     test_initial_output_is_inside_the_range},
};

AXXPID_TEST_MAIN("limits", tests)
