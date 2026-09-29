/**
 * @file  test_modes.c
 * @brief Modes, transfers, retuning and bad input.
 *
 * Manual and automatic operation, bumpless transfer in both
 * directions, changing gains on a running loop, and what happens
 * when a caller or a sensor hands the controller something it
 * cannot use.
 */

#include "axxpid_test.h"

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
    {"manual mode and bumpless transfer",
     test_manual_mode_and_bumpless_transfer},
    {"manual output is clamped", test_manual_output_is_clamped},
    {"bumpless retuning", test_bumpless_tuning},
    {"changing ki never bumps", test_ki_change_never_bumps},
    {"changing acting clears the integral",
     test_changing_acting_clears_integral},
    {"new output limits clamp live state",
     test_output_limits_clamp_live_state},
    {"transfer preloads from the reachable output",
     test_transfer_preloads_from_the_reachable_output},
    {"dt_max does not manufacture a derivative",
     test_dt_max_does_not_manufacture_a_derivative},
    {"rejected sample restarts the derivative",
     test_rejected_sample_restarts_the_derivative},
    {"mixing update and update_at",
     test_mixing_update_and_update_at},
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
    {"bumpless retuning compensates the derivative",
     test_bumpless_retuning_compensates_the_derivative},
    {"transfer with the derivative active",
     test_transfer_with_derivative_active},
    {"manual mode captures the live output",
     test_manual_mode_captures_the_live_output},
    {"transfer without an intervening update",
     test_transfer_without_an_intervening_update},
    {"setters move the live state",
     test_setters_move_the_live_state},
    {"init_config rejects every bad field",
     test_init_config_rejects_every_bad_field},
    {"manual output survives a limit change",
     test_manual_output_survives_a_limit_change},
    {"broken feed-forward hook is rejected",
     test_broken_feedforward_hook_is_rejected},
};

AXXPID_TEST_MAIN("modes", tests)
