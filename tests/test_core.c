/**
 * @file  test_core.c
 * @brief Lifecycle, argument validation, scheduling and robustness tests.
 */

#include "axxpid_test.h"

static void test_version(void)
{
    CHECK(strcmp(axxpid_version(), AXXPID_VERSION_STRING) == 0);
    CHECK(AXXPID_VERSION_MAJOR == 1);
}

static void test_defaults(void)
{
    axxpid_config_t cfg;

    CHECK(axxpid_config_default(&cfg) == AXXPID_OK);
    CHECK(axxpid_config_default(NULL) == AXXPID_ERR_NULL);

    CHECK(cfg.kp == 0);
    CHECK(cfg.ki == 0);
    CHECK(cfg.kd == 0);
    CHECK(cfg.acting == AXXPID_ACTING_DIRECT);
    CHECK(cfg.mode == AXXPID_MODE_AUTOMATIC);
    CHECK(cfg.antiwindup == AXXPID_ANTIWINDUP_CONDITIONAL);
    CHECK(cfg.setpoint_weight_b == 1);
    CHECK(cfg.setpoint_weight_c == 0);
    CHECK(cfg.integral_overshoot_gain == 1);
    CHECK(cfg.out_min < cfg.out_max);
    CHECK(cfg.integral_min < cfg.integral_max);
    CHECK(cfg.sample_time_ms > 0u);
}

/* The integral limits must default to "no limit", not to zero: a library that
 * silently disables the I term out of the box is a support nightmare. */
static void test_integral_works_out_of_the_box(void)
{
    axxpid_t pid;
    int i;

    CHECK(axxpid_init(&pid, 0, 1, 0, -1000, 1000) == AXXPID_OK);
    for (i = 0; i < 10; ++i) {
        (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    }
    /* ki=1, error=10, 10 samples of 0.1 s -> integral 10. */
    CHECK_NEAR(axxpid_get_i_term(&pid), 10, AXXPID_C(1e-4));
}

static void test_null_safety(void)
{
    axxpid_terms_t terms;

    CHECK(axxpid_init(NULL, 1, 1, 1, 0, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_init_config(NULL, NULL) == AXXPID_ERR_NULL);
    CHECK(axxpid_reset(NULL) == AXXPID_ERR_NULL);
    CHECK(axxpid_reset_to(NULL, 0) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_tunings(NULL, 1, 1, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_tunings_standard(NULL, 1, 1, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_kp(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_ki(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_kd(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_output_limits(NULL, 0, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_output_slew_rate(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_integral_limits(NULL, 0, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_antiwindup(NULL, AXXPID_ANTIWINDUP_NONE, 1) ==
          AXXPID_ERR_NULL);
    CHECK(axxpid_set_integral_band(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_integral_overshoot(NULL, 1, 0) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_integral_reset_on_zero_setpoint(NULL, true) ==
          AXXPID_ERR_NULL);
    CHECK(axxpid_set_derivative_filter(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_derivative_filter_tau(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_setpoint_weights(NULL, 1, 0) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_deadband(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_feedforward_bias(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_feedforward_gains(NULL, 1, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_feedforward_fn(NULL, NULL, NULL) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_acting(NULL, AXXPID_ACTING_DIRECT) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_mode(NULL, AXXPID_MODE_MANUAL) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_manual_output(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_sample_time(NULL, 10, false) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_dt_max(NULL, 1) == AXXPID_ERR_NULL);
    CHECK(axxpid_set_bumpless_tuning(NULL, true) == AXXPID_ERR_NULL);
    CHECK(axxpid_get_terms(NULL, &terms) == AXXPID_ERR_NULL);

    CHECK(axxpid_update(NULL, 1, 1, 1) == 0);
    CHECK(axxpid_update_at(NULL, 1, 1, 1) == false);
    CHECK(axxpid_get_output(NULL) == 0);
    CHECK(axxpid_get_error(NULL) == 0);
    CHECK(axxpid_get_p_term(NULL) == 0);
    CHECK(axxpid_get_i_term(NULL) == 0);
    CHECK(axxpid_get_d_term(NULL) == 0);
    CHECK(axxpid_get_ff_term(NULL) == 0);
    CHECK(axxpid_get_kp(NULL) == 0);
    CHECK(axxpid_get_ki(NULL) == 0);
    CHECK(axxpid_get_kd(NULL) == 0);
    CHECK(axxpid_is_saturated(NULL) == false);
    CHECK(axxpid_get_mode(NULL) == AXXPID_MODE_MANUAL);
    CHECK(axxpid_get_acting(NULL) == AXXPID_ACTING_DIRECT);
}

static void test_parameter_validation(void)
{
    axxpid_t pid;
    axxpid_config_t cfg;

    CHECK(axxpid_init(&pid, -1, 0, 0, 0, 100) == AXXPID_ERR_PARAM);
    CHECK(axxpid_init(&pid, 0, -1, 0, 0, 100) == AXXPID_ERR_PARAM);
    CHECK(axxpid_init(&pid, 0, 0, -1, 0, 100) == AXXPID_ERR_PARAM);
    /* An inverted output range is rejected the same way. */
    CHECK(axxpid_init(&pid, 1, 1, 1, 100, 0) == AXXPID_ERR_PARAM);
    CHECK(axxpid_init(&pid, 1, 1, 1, 0, 100) == AXXPID_OK);

    CHECK(axxpid_set_tunings(&pid, -1, 1, 1) == AXXPID_ERR_PARAM);
    /* A rejected setter must not have changed anything. */
    CHECK(axxpid_get_kp(&pid) == 1);

    CHECK(axxpid_set_output_limits(&pid, 5, 5) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_output_limits(&pid, 5, 1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_output_limits(&pid, 0, 100) == AXXPID_OK);

    CHECK(axxpid_set_output_slew_rate(&pid, -1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_integral_limits(&pid, 10, 5) == AXXPID_ERR_PARAM);
    /* Equal integral limits are legal: they pin the integrator. */
    CHECK(axxpid_set_integral_limits(&pid, 0, 0) == AXXPID_OK);

    CHECK(axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION, 0) ==
          AXXPID_ERR_PARAM);
    CHECK(axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(0.5)) == AXXPID_OK);
    CHECK(axxpid_set_antiwindup(&pid, (axxpid_antiwindup_t)99, 1) ==
          AXXPID_ERR_PARAM);

    CHECK(axxpid_set_integral_band(&pid, -1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_integral_overshoot(&pid, -1, 0) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_derivative_filter(&pid, -1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_derivative_filter_tau(&pid, -1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_setpoint_weights(&pid, -1, 0) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_setpoint_weights(&pid, 2, 0) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_setpoint_weights(&pid, 0, 2) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_deadband(&pid, -1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_acting(&pid, (axxpid_acting_t)7) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_mode(&pid, (axxpid_mode_t)7) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_sample_time(&pid, 0, false) == AXXPID_ERR_PARAM);
    CHECK(axxpid_set_dt_max(&pid, 0) == AXXPID_ERR_PARAM);

    /* A whole config is validated as one; a bad field rejects the lot. */
    (void)axxpid_config_default(&cfg);
    cfg.out_min = 10;
    cfg.out_max = 1;
    CHECK(axxpid_init_config(&pid, &cfg) == AXXPID_ERR_PARAM);

    (void)axxpid_config_default(&cfg);
    cfg.antiwindup = AXXPID_ANTIWINDUP_BACK_CALCULATION;
    cfg.tracking_time = 0;
    CHECK(axxpid_init_config(&pid, &cfg) == AXXPID_ERR_PARAM);

    (void)axxpid_config_default(&cfg);
    cfg.sample_time_ms = 0u;
    CHECK(axxpid_init_config(&pid, &cfg) == AXXPID_ERR_PARAM);
}

static void test_reset(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 1, 1, 0, -100, 100);
    for (i = 0; i < 20; ++i) {
        (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    }
    CHECK(axxpid_get_i_term(&pid) > 0);

    CHECK(axxpid_reset(&pid) == AXXPID_OK);
    CHECK(axxpid_get_i_term(&pid) == 0);
    CHECK(axxpid_get_output(&pid) == 0);
    CHECK(axxpid_get_d_term(&pid) == 0);
    /* Gains survive a reset. */
    CHECK(axxpid_get_kp(&pid) == 1);
}

static void test_reset_to_is_bumpless(void)
{
    axxpid_t pid;
    axxpid_real_t u;

    /* With no integral action the first closed-loop output must land exactly
     * on the value we resumed from, even though a standing error of 50 and a
     * kp of 2 would otherwise ask for an output of 100. */
    (void)axxpid_init(&pid, 2, 0, 0, 0, 100);
    CHECK(axxpid_reset_to(&pid, 40) == AXXPID_OK);
    CHECK_NEAR(axxpid_get_output(&pid), 40, AXXPID_C(1e-5));

    u = axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    CHECK_NEAR(u, 40, AXXPID_C(1e-4));

    /* With integral action the same sample also takes its first normal
     * integral step - that is control action, not a bump, and it must be no
     * larger than ki * error * dt = 0.5 * 50 * 0.1 = 2.5. */
    (void)axxpid_init(&pid, 2, AXXPID_C(0.5), 0, 0, 100);
    CHECK(axxpid_reset_to(&pid, 40) == AXXPID_OK);

    u = axxpid_update(&pid, 100, 50, AXXPID_C(0.1));
    CHECK_NEAR(u, AXXPID_C(42.5), AXXPID_C(1e-4));
}

/* A single garbage sample must not be able to poison the controller. */
static void test_rejects_bad_input(void)
{
    axxpid_t pid;
    axxpid_real_t good;

    /* Built through a volatile zero so the compiler cannot fold them away,
     * and so the tests need no <math.h> either. */
    volatile axxpid_real_t zero = 0;
    const axxpid_real_t inf_value = AXXPID_C(1) / zero;
    const axxpid_real_t nan_value = zero / zero;

    CHECK(nan_value != nan_value);
    CHECK(inf_value > AXXPID_UNLIMITED);

    (void)axxpid_init(&pid, 1, 1, 1, -100, 100);

    good = axxpid_update(&pid, 10, 5, AXXPID_C(0.1));

    CHECK(axxpid_update(&pid, 10, 5, 0) == good);
    CHECK(axxpid_update(&pid, 10, 5, -1) == good);
    CHECK(axxpid_update(&pid, nan_value, 5, AXXPID_C(0.1)) == good);
    CHECK(axxpid_update(&pid, 10, nan_value, AXXPID_C(0.1)) == good);
    CHECK(axxpid_update(&pid, 10, 5, nan_value) == good);
    CHECK(axxpid_update(&pid, inf_value, 5, AXXPID_C(0.1)) == good);
    CHECK(axxpid_update(&pid, 10, -inf_value, AXXPID_C(0.1)) == good);

    /* And the state is still sane afterwards. */
    CHECK(axxpid_get_i_term(&pid) == axxpid_get_i_term(&pid));
}

static void test_dt_max_clamp(void)
{
    axxpid_t pid_a;
    axxpid_t pid_b;

    (void)axxpid_init(&pid_a, 0, 1, 0, -1000, 1000);
    (void)axxpid_init(&pid_b, 0, 1, 0, -1000, 1000);
    CHECK(axxpid_set_dt_max(&pid_b, AXXPID_C(0.2)) == AXXPID_OK);

    (void)axxpid_update(&pid_a, 10, 0, 5);
    (void)axxpid_update(&pid_b, 10, 0, 5);

    CHECK_NEAR(axxpid_get_i_term(&pid_a), 50, AXXPID_C(1e-3));
    CHECK_NEAR(axxpid_get_i_term(&pid_b), 2, AXXPID_C(1e-3));
}

static void test_update_at_gating(void)
{
    axxpid_t pid;
    uint32_t t;
    int updates = 0;

    (void)axxpid_init(&pid, 1, 0, 0, -1000, 1000);
    (void)axxpid_set_sample_time(&pid, 100u, false);

    /* The first call runs, because the controller backdates its clock by one
     * period rather than throwing the first sample away. */
    CHECK(axxpid_update_at(&pid, 10, 0, 1000u) == true);

    for (t = 1001u; t <= 1400u; ++t) {
        if (axxpid_update_at(&pid, 10, 0, t)) {
            updates++;
        }
    }
    CHECK_MSG(updates == 4, "expected 4 updates in 400 ms at 100 ms, got %d",
              updates);
}

static void test_update_at_every_call(void)
{
    axxpid_t pid;
    int updates = 0;
    uint32_t t;

    (void)axxpid_init(&pid, 1, 0, 0, -1000, 1000);
    (void)axxpid_set_sample_time(&pid, 100u, true);

    for (t = 1000u; t < 1010u; ++t) {
        if (axxpid_update_at(&pid, 10, 0, t)) {
            updates++;
        }
    }
    /* Every distinct millisecond is an update; a repeated timestamp is not. */
    CHECK(updates == 10);
    CHECK(axxpid_update_at(&pid, 10, 0, 1009u) == false);
}

/* A 32-bit millisecond counter wraps every 49.7 days. Unsigned arithmetic
 * must carry the controller across without a 49-day dt. */
static void test_update_at_wraparound(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 1, 0, -1000, 1000);
    (void)axxpid_set_sample_time(&pid, 100u, false);

    /* 0xFFFFFF9C is 100 ms short of the wrap, so the first call (backdated by
     * one period) integrates 0.1 s and the second integrates 0.2 s across the
     * wrap: 1 + 2 = 3, not the 4.29e6 an unguarded subtraction would give. */
    CHECK(axxpid_update_at(&pid, 10, 0, 0xFFFFFF9Cu) == true);
    CHECK_NEAR(axxpid_get_i_term(&pid), 1, AXXPID_C(0.001));

    CHECK(axxpid_update_at(&pid, 10, 0, 100u) == true);
    CHECK_NEAR(axxpid_get_i_term(&pid), 3, AXXPID_C(0.001));
}

static void test_getters_and_terms(void)
{
    axxpid_t pid;
    axxpid_terms_t terms;

    (void)axxpid_init(&pid, 2, 3, 4, -1000, 1000);
    CHECK(axxpid_get_kp(&pid) == 2);
    CHECK(axxpid_get_ki(&pid) == 3);
    CHECK(axxpid_get_kd(&pid) == 4);
    CHECK(axxpid_get_mode(&pid) == AXXPID_MODE_AUTOMATIC);
    CHECK(axxpid_get_acting(&pid) == AXXPID_ACTING_DIRECT);

    (void)axxpid_update(&pid, 10, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 10, 2, AXXPID_C(0.1));

    CHECK(axxpid_get_terms(&pid, &terms) == AXXPID_OK);
    CHECK(axxpid_get_terms(&pid, NULL) == AXXPID_ERR_NULL);
    /* Against hand arithmetic, not against the same field read twice.
     * kp=2, ki=3, kd=4. Sample 1: sp=10 pv=0 dt=0.1 -> I = 3*10*0.1 = 3,
     * D = 0 (no history yet). Sample 2: sp=10 pv=2 -> e = 8, P = 16,
     * I = 3 + 3*8*0.1 = 5.4, D = -4*(2-0)/0.1 = -80. */
    CHECK_NEAR(terms.p, 16, AXXPID_C(1e-4));
    CHECK_NEAR(terms.i, AXXPID_C(5.4), AXXPID_C(1e-4));
    CHECK_NEAR(terms.d, -80, AXXPID_C(1e-3));
    CHECK_NEAR(terms.ff, 0, AXXPID_C(1e-6));
    CHECK_NEAR(terms.error, 8, AXXPID_C(1e-5));

    /* And the single-value getters agree with the struct. */
    CHECK(terms.p == axxpid_get_p_term(&pid));
    CHECK(terms.i == axxpid_get_i_term(&pid));
    CHECK(terms.d == axxpid_get_d_term(&pid));
    CHECK(terms.ff == axxpid_get_ff_term(&pid));
    CHECK(terms.output == axxpid_get_output(&pid));
    CHECK(terms.error == axxpid_get_error(&pid));

    /* The four terms reconstruct the (unclamped) output exactly. */
    CHECK_NEAR(terms.p + terms.i + terms.d + terms.ff, terms.output,
               AXXPID_C(1e-4));
}

static void test_standard_form_conversion(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 0, 0, 0, -1000, 1000);

    /* kp=4, Ti=2 s, Td=0.5 s -> ki = 4/2 = 2, kd = 4*0.5 = 2. */
    CHECK(axxpid_set_tunings_standard(&pid, 4, 2, AXXPID_C(0.5)) == AXXPID_OK);
    CHECK_NEAR(axxpid_get_kp(&pid), 4, AXXPID_C(1e-6));
    CHECK_NEAR(axxpid_get_ki(&pid), 2, AXXPID_C(1e-6));
    CHECK_NEAR(axxpid_get_kd(&pid), 2, AXXPID_C(1e-6));

    /* Ti = 0 is the documented way to ask for no integral action. */
    CHECK(axxpid_set_tunings_standard(&pid, 4, 0, 0) == AXXPID_OK);
    CHECK(axxpid_get_ki(&pid) == 0);

    CHECK(axxpid_set_tunings_standard(&pid, -1, 1, 1) == AXXPID_ERR_PARAM);
}

static void test_individual_gain_setters(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 2, 3, -1000, 1000);
    CHECK(axxpid_set_kp(&pid, 10) == AXXPID_OK);
    CHECK(axxpid_get_kp(&pid) == 10);
    CHECK(axxpid_get_ki(&pid) == 2);
    CHECK(axxpid_get_kd(&pid) == 3);

    CHECK(axxpid_set_ki(&pid, 20) == AXXPID_OK);
    CHECK(axxpid_get_ki(&pid) == 20);
    CHECK(axxpid_set_kd(&pid, 30) == AXXPID_OK);
    CHECK(axxpid_get_kd(&pid) == 30);

    CHECK(axxpid_set_kp(&pid, -1) == AXXPID_ERR_PARAM);
    CHECK(axxpid_get_kp(&pid) == 10);
}

static const axxpid_test_case_t tests[] = {
    {"version", test_version},
    {"config defaults", test_defaults},
    {"integral enabled by default", test_integral_works_out_of_the_box},
    {"NULL safety on every entry point", test_null_safety},
    {"parameter validation", test_parameter_validation},
    {"reset clears state, keeps gains", test_reset},
    {"reset_to resumes bumplessly", test_reset_to_is_bumpless},
    {"NaN / Inf / bad dt are rejected", test_rejects_bad_input},
    {"dt_max caps a stalled loop", test_dt_max_clamp},
    {"update_at rate gating", test_update_at_gating},
    {"update_at every-call mode", test_update_at_every_call},
    {"update_at survives tick wraparound", test_update_at_wraparound},
    {"getters and term breakdown", test_getters_and_terms},
    {"standard-form tunings", test_standard_form_conversion},
    {"individual gain setters", test_individual_gain_setters},
};

AXXPID_TEST_MAIN("core", tests)
