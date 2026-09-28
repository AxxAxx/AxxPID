/**
 * @file  test_closedloop.c
 * @brief Closed-loop behaviour against a simulated process.
 *
 * Unit tests prove the arithmetic; these prove the controller actually
 * controls. Each test drives a first-order-plus-dead-time plant and asserts
 * on the properties an engineer would actually check on a scope: does it
 * settle, does it settle on the setpoint, how far does it overshoot, and does
 * it recover from a disturbance.
 */

#include "axxpid_plant.h"
#include "axxpid_test.h"

#define DT AXXPID_C(0.01)

/** @brief Summary of one closed-loop run. */
typedef struct {
    axxpid_real_t final_value;   /**< Process value at the end of the run. */
    axxpid_real_t peak;          /**< Highest process value reached. */
    axxpid_real_t trough;        /**< Lowest process value reached. */
    axxpid_real_t settle_time;   /**< First time it stayed inside the band. */
    axxpid_real_t final_output;  /**< Controller output at the end. */
    bool settled;                /**< Whether it ever settled. */
} run_result_t;

/**
 * @brief Run a closed loop for @p seconds and report what happened.
 * @param pid       Controller, already configured.
 * @param plant     Plant, already initialised.
 * @param setpoint  Constant setpoint to hold.
 * @param seconds   Duration of the run.
 * @param band      Settling band, in measurement units.
 */
static run_result_t run_loop(axxpid_t *pid,
                             axxpid_plant_t *plant,
                             axxpid_real_t setpoint,
                             axxpid_real_t seconds,
                             axxpid_real_t band)
{
    run_result_t r;
    axxpid_real_t t = 0;
    axxpid_real_t pv = plant->y;
    axxpid_real_t u = 0;
    const long steps = (long)(seconds / DT);
    long i;

    r.peak = pv;
    r.trough = pv;
    r.settle_time = -1;
    r.settled = false;

    for (i = 0; i < steps; ++i) {
        u = axxpid_update(pid, setpoint, pv, DT);
        pv = axxpid_plant_step(plant, u, DT);
        t += DT;

        if (pv > r.peak) {
            r.peak = pv;
        }
        if (pv < r.trough) {
            r.trough = pv;
        }

        /* Settling time is the moment it last entered the band and stayed,
         * so a late excursion correctly pushes it back out. A run that never
         * settles reports -1 rather than a number that looks like a result. */
        if (!axxpid_test_near(pv, setpoint, band)) {
            r.settled = false;
            r.settle_time = -1;
        } else if (!r.settled) {
            r.settled = true;
            r.settle_time = t;
        }
    }

    r.final_value = pv;
    r.final_output = u;
    return r;
}

/* -------------------------------------------------------------------------- */

/* P-only control leaves a standing offset; that is the whole reason the I
 * term exists, and the test documents the size of it. */
static void test_proportional_only_leaves_offset(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    run_result_t r;

    axxpid_plant_init(&plant, 1, 5, 0, 0, DT);
    (void)axxpid_init(&pid, 2, 0, 0, 0, 200);

    r = run_loop(&pid, &plant, 100, 100, AXXPID_C(0.5));

    /* Steady state: u = kp*e and pv = K*u, so pv = 100 * 2/(1+2) = 66.7. */
    CHECK_NEAR(r.final_value, AXXPID_C(66.667), AXXPID_C(0.5));
}

/* Adding integral action must remove that offset exactly. */
static void test_pi_removes_offset(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    run_result_t r;

    axxpid_plant_init(&plant, 1, 5, 0, 0, DT);
    (void)axxpid_init(&pid, 2, AXXPID_C(1.5), 0, 0, 200);

    r = run_loop(&pid, &plant, 100, 100, AXXPID_C(0.5));

    CHECK_NEAR(r.final_value, 100, AXXPID_C(0.5));
    CHECK_MSG(r.settled, "PI loop never settled, ended at %.2f",
              (double)r.final_value);
}

/* The headline test: a PID on a lagged plant with a real dead time reaches
 * setpoint, stays there, and does not overshoot wildly on the way. */
static void test_pid_step_response(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    run_result_t r;

    axxpid_plant_init(&plant, 1, 10, 20, AXXPID_C(0.5), DT);
    (void)axxpid_init(&pid, AXXPID_C(1.5), AXXPID_C(0.3), AXXPID_C(1.0), 0, 300);
    (void)axxpid_set_derivative_filter(&pid, 10);

    r = run_loop(&pid, &plant, 200, 300, AXXPID_C(1.0));

    CHECK_MSG(axxpid_test_near(r.final_value, 200, AXXPID_C(0.5)),
              "settled at %.2f instead of 200", (double)r.final_value);
    CHECK_MSG(r.peak < AXXPID_C(222), "overshoot to %.1f exceeds 11%%",
              (double)r.peak);
    CHECK_MSG(r.settled && (r.settle_time < 120),
              "settled at %.1f s (-1 means never)", (double)r.settle_time);
}

/* A short, hard-limited actuator is the classic wind-up trap: the setpoint is
 * out of reach for a long time, then comes into reach. Without anti-windup
 * the integrator overshoots enormously. */
static void test_saturated_start_does_not_overshoot(void)
{
    axxpid_t guarded;
    axxpid_t unguarded;
    axxpid_plant_t plant_a;
    axxpid_plant_t plant_b;
    run_result_t ra;
    run_result_t rb;

    axxpid_plant_init(&plant_a, 1, 20, 0, 0, DT);
    (void)axxpid_init(&guarded, 2, 1, 0, 0, 120);
    (void)axxpid_set_antiwindup(&guarded, AXXPID_ANTIWINDUP_CONDITIONAL, 1);

    axxpid_plant_init(&plant_b, 1, 20, 0, 0, DT);
    (void)axxpid_init(&unguarded, 2, 1, 0, 0, 120);
    (void)axxpid_set_antiwindup(&unguarded, AXXPID_ANTIWINDUP_NONE, 1);

    ra = run_loop(&guarded, &plant_a, 100, 400, AXXPID_C(1.0));
    rb = run_loop(&unguarded, &plant_b, 100, 400, AXXPID_C(1.0));

    CHECK_NEAR(ra.final_value, 100, AXXPID_C(1.0));
    CHECK_MSG(ra.peak < AXXPID_C(105),
              "anti-windup still overshot to %.1f", (double)ra.peak);
    CHECK_MSG(rb.peak > ra.peak + 5,
              "unprotected loop peaked at %.1f, no worse than the guarded one",
              (double)rb.peak);
}

/* Back-calculation should reach the same endpoint as conditional integration
 * on the same plant. Two different mechanisms, one correct answer. */
static void test_back_calculation_settles_too(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    run_result_t r;

    axxpid_plant_init(&plant, 1, 20, 0, 0, DT);
    (void)axxpid_init(&pid, 2, 1, 0, 0, 120);
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(2.0));

    r = run_loop(&pid, &plant, 100, 400, AXXPID_C(1.0));

    CHECK_NEAR(r.final_value, 100, AXXPID_C(1.0));
    CHECK_MSG(r.peak < AXXPID_C(115),
              "back-calculation overshot to %.1f", (double)r.peak);
}

/* Steady-state feed-forward should put the actuator roughly in the right
 * place before the feedback loop has had to build up any error at all. */
static void test_feedforward_improves_tracking(void)
{
    axxpid_t plain;
    axxpid_t assisted;
    axxpid_plant_t plant_a;
    axxpid_plant_t plant_b;
    axxpid_real_t error_plain = 0;
    axxpid_real_t error_assisted = 0;
    axxpid_real_t pv_a;
    axxpid_real_t pv_b;
    long i;

    /* Plant gain is 0.5, so holding pv = sp needs u = 2*sp. A feed-forward
     * gain of 2 is the exact plant inverse. */
    axxpid_plant_init(&plant_a, AXXPID_C(0.5), 10, 0, 0, DT);
    axxpid_plant_init(&plant_b, AXXPID_C(0.5), 10, 0, 0, DT);

    /* A deliberately gentle integrator, which is exactly when feed-forward
     * earns its keep: without it the loop has to integrate its way to an
     * output of 200 from nothing. */
    (void)axxpid_init(&plain, 1, AXXPID_C(0.05), 0, 0, 400);

    (void)axxpid_init(&assisted, 1, AXXPID_C(0.05), 0, 0, 400);
    (void)axxpid_set_feedforward_gains(&assisted, 2, 0);

    pv_a = plant_a.y;
    pv_b = plant_b.y;
    for (i = 0; i < 100000; ++i) {
        const axxpid_real_t sp = 100;
        axxpid_real_t e;

        pv_a = axxpid_plant_step(&plant_a,
                                 axxpid_update(&plain, sp, pv_a, DT), DT);
        pv_b = axxpid_plant_step(&plant_b,
                                 axxpid_update(&assisted, sp, pv_b, DT), DT);

        /* Score the first fifty seconds. Feed-forward puts the actuator
         * where it belongs immediately, so the assisted loop is essentially
         * done once the plant lag has played out, while the plain loop is
         * still waiting for its integrator to climb. Over a long enough run
         * both get there - the point is how much error they accumulate on the
         * way. */
        if (i < 5000) {
            e = sp - pv_a;
            error_plain += (e < 0 ? -e : e) * DT;
            e = sp - pv_b;
            error_assisted += (e < 0 ? -e : e) * DT;
        }
    }

    CHECK_MSG(error_assisted < error_plain * AXXPID_C(0.5),
              "feed-forward barely helped: IAE %.1f vs %.1f",
              (double)error_assisted, (double)error_plain);

    /* Both must still land exactly on the setpoint - feed-forward assists the
     * integrator, it does not replace it, and it must not leave an offset. */
    CHECK_NEAR(pv_a, 100, AXXPID_C(0.5));
    CHECK_NEAR(pv_b, 100, AXXPID_C(0.5));
}

/* Velocity feed-forward is for ramps: it should cut the lag behind a moving
 * setpoint substantially. */
static void test_velocity_feedforward_tracks_a_ramp(void)
{
    axxpid_t plain;
    axxpid_t assisted;
    axxpid_plant_t plant_a;
    axxpid_plant_t plant_b;
    axxpid_real_t lag_plain = 0;
    axxpid_real_t lag_assisted = 0;
    axxpid_real_t pv_a;
    axxpid_real_t pv_b;
    long i;

    axxpid_plant_init(&plant_a, 1, 5, 0, 0, DT);
    axxpid_plant_init(&plant_b, 1, 5, 0, 0, DT);

    (void)axxpid_init(&plain, 2, 1, 0, 0, 500);
    (void)axxpid_set_feedforward_gains(&plain, 1, 0);

    (void)axxpid_init(&assisted, 2, 1, 0, 0, 500);
    /* Steady-state inverse is 1; a ramp additionally needs tau * d(sp)/dt. */
    (void)axxpid_set_feedforward_gains(&assisted, 1, 5);

    pv_a = plant_a.y;
    pv_b = plant_b.y;
    for (i = 0; i < 2000; ++i) {
        /* Ramp from 0 to 100 over 20 s. */
        const axxpid_real_t sp = AXXPID_C(0.05) * (axxpid_real_t)i;
        axxpid_real_t e;

        pv_a = axxpid_plant_step(&plant_a,
                                 axxpid_update(&plain, sp, pv_a, DT), DT);
        pv_b = axxpid_plant_step(&plant_b,
                                 axxpid_update(&assisted, sp, pv_b, DT), DT);

        if (i > 200) {
            e = sp - pv_a;
            lag_plain += (e < 0 ? -e : e) * DT;
            e = sp - pv_b;
            lag_assisted += (e < 0 ? -e : e) * DT;
        }
    }

    CHECK_MSG(lag_assisted < lag_plain * AXXPID_C(0.1),
              "velocity feed-forward barely helped: %.2f vs %.2f",
              (double)lag_assisted, (double)lag_plain);
}

/* A load disturbance must be rejected back to zero error by the integrator. */
static void test_disturbance_rejection(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    axxpid_real_t pv;
    axxpid_real_t worst = 0;
    long i;

    axxpid_plant_init(&plant, 1, 10, 0, 0, DT);
    (void)axxpid_init(&pid, 2, 1, AXXPID_C(0.5), -200, 200);
    (void)axxpid_set_derivative_filter(&pid, 10);

    pv = plant.y;
    for (i = 0; i < 20000; ++i) {
        const axxpid_real_t u = axxpid_update(&pid, 50, pv, DT);

        /* A step load appears halfway through and stays. */
        plant.ambient = (i > 5000) ? AXXPID_C(-30) : AXXPID_C(0);
        pv = axxpid_plant_step(&plant, u, DT);

        if (i > 5000) {
            const axxpid_real_t e = 50 - pv;
            const axxpid_real_t mag = (e < 0) ? -e : e;
            if (mag > worst) {
                worst = mag;
            }
        }
    }

    CHECK_NEAR(pv, 50, AXXPID_C(0.2));
    CHECK_MSG(worst < 10, "disturbance dipped the process by %.1f",
              (double)worst);
}

/* Reverse-acting control of a cooling process: the same loop, wired the other
 * way round, must behave identically. */
static void test_reverse_acting_closed_loop(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    axxpid_real_t pv;
    long i;

    /* Negative plant gain: more output drives the measurement down. */
    axxpid_plant_init(&plant, AXXPID_C(-1.0), 10, 100, 0, DT);

    (void)axxpid_init(&pid, 2, 1, 0, 0, 200);
    (void)axxpid_set_acting(&pid, AXXPID_ACTING_REVERSE);

    pv = plant.y;
    for (i = 0; i < 20000; ++i) {
        pv = axxpid_plant_step(&plant, axxpid_update(&pid, 40, pv, DT), DT);
    }

    CHECK_NEAR(pv, 40, AXXPID_C(0.2));
    CHECK(axxpid_get_output(&pid) > 0);
}

/* Loop jitter is a fact of life on a busy MCU. The controller integrates and
 * differentiates against the true elapsed time, so a jittery loop must reach
 * the same steady state as a metronomic one. */
static void test_jittery_sample_time(void)
{
    axxpid_t steady;
    axxpid_t jittery;
    axxpid_plant_t plant_a;
    axxpid_plant_t plant_b;
    axxpid_real_t pv_a;
    axxpid_real_t pv_b;
    uint32_t seed = 12345u;
    long i;

    axxpid_plant_init(&plant_a, 1, 10, 0, 0, DT);
    axxpid_plant_init(&plant_b, 1, 10, 0, 0, DT);
    (void)axxpid_init(&steady, 2, 1, AXXPID_C(0.2), 0, 300);
    (void)axxpid_init(&jittery, 2, 1, AXXPID_C(0.2), 0, 300);

    pv_a = plant_a.y;
    pv_b = plant_b.y;
    for (i = 0; i < 20000; ++i) {
        /* The jittery loop is called at a random dt averaging DT, but the
         * plant is still stepped at the true DT, so only the controller sees
         * the jitter. */
        axxpid_real_t dt;

        seed = (seed * 1103515245u) + 12345u;
        dt = DT * (AXXPID_C(0.5) +
                   (AXXPID_C(1.0) * (axxpid_real_t)((seed >> 16) & 0xFFFu) /
                    AXXPID_C(4095.0)));

        pv_a = axxpid_plant_step(&plant_a,
                                 axxpid_update(&steady, 100, pv_a, DT), DT);
        pv_b = axxpid_plant_step(&plant_b,
                                 axxpid_update(&jittery, 100, pv_b, dt), DT);
    }

    CHECK_NEAR(pv_a, 100, AXXPID_C(0.5));
    CHECK_MSG(axxpid_test_near(pv_b, 100, AXXPID_C(1.0)),
              "jittery loop settled at %.3f", (double)pv_b);
}

/* The AxxSolder configuration, end to end, on a soldering-iron-shaped plant:
 * a heater that can only add heat and an asymmetric integrator to match. */
static void test_axxsolder_profile(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    run_result_t r;

    /* A tip that reaches ~775 C at full power and cools towards 25 C ambient,
     * so holding 330 C needs about 200 of the 0..500 output. That matters:
     * it has to sit comfortably inside the +/- 300 integral clamp, or the
     * clamp itself becomes the binding constraint and leaves a standing
     * error that has nothing to do with the controller. */
    axxpid_plant_init(&plant, AXXPID_C(1.5), 6, 25, AXXPID_C(0.2), DT);

    (void)axxpid_init(&pid, 8, 2, AXXPID_C(0.5), 0, 500);
    (void)axxpid_set_integral_limits(&pid, -300, 300);
    (void)axxpid_set_integral_band(&pid, 75);
    (void)axxpid_set_integral_overshoot(&pid, 7, -1);
    (void)axxpid_set_integral_reset_on_zero_setpoint(&pid, true);
    (void)axxpid_set_sample_time(&pid, 25u, false);

    r = run_loop(&pid, &plant, 330, 200, AXXPID_C(2.0));

    /* A tenth of a degree, not a degree: the point of the integrator is that
     * there is no standing error, and a loose band here would hide one. */
    CHECK_MSG(axxpid_test_near(r.final_value, 330, AXXPID_C(0.1)),
              "tip settled at %.2f C instead of 330 C",
              (double)r.final_value);
    CHECK_MSG(axxpid_get_i_term(&pid) < 300,
              "integral pinned at its clamp (%.1f); the clamp, not the loop, "
              "is setting the output", (double)axxpid_get_i_term(&pid));
    CHECK_MSG(r.peak < AXXPID_C(333),
              "tip overshot to %.1f C on the way up", (double)r.peak);
    CHECK_MSG(r.settled && (r.settle_time < 60),
              "tip settled at %.1f s (-1 means never)",
              (double)r.settle_time);

    /* Setpoint of zero means "off": the integrator must let go completely. */
    r = run_loop(&pid, &plant, 0, 20, AXXPID_C(1e9));
    CHECK(axxpid_get_i_term(&pid) == 0);
    CHECK_NEAR(axxpid_get_output(&pid), 0, AXXPID_C(1e-4));
}

/* Anti-windup off, integrating straight through saturation: the control
 * against which the other two strategies are judged. */
static void test_antiwindup_none_integrates_through_saturation(void)
{
    axxpid_t pid;
    int i;

    (void)axxpid_init(&pid, 0, 2, 0, 0, 10);
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_NONE, 1);
    /* Opt out of the derived integral limits: this measures what happens with
     * no anti-windup, not what happens with a clamp. */
    (void)axxpid_set_integral_limits(&pid, -AXXPID_UNLIMITED,
                                     AXXPID_UNLIMITED);

    /* No protection at all means exactly ki*e*dt every sample, limit or no
     * limit: 2 * 100 * 0.1 * 50 = 1000. */
    for (i = 0; i < 50; ++i) {
        (void)axxpid_update(&pid, 100, 0, AXXPID_C(0.1));
    }
    CHECK_NEAR(axxpid_get_i_term(&pid), 1000, AXXPID_C(0.5));
    CHECK_NEAR(axxpid_get_output(&pid), 10, AXXPID_C(1e-5));

    /* And the price: 50 s of reversed error before it comes off the limit. */
    for (i = 0; i < 100; ++i) {
        (void)axxpid_update(&pid, 0, 10, AXXPID_C(0.1));
    }
    CHECK_MSG(axxpid_get_output(&pid) > 9,
              "unprotected integrator recovered too quickly to be unprotected");
}

/* An explicit filter time constant must win over the N-derived one, which is
 * what you want when you retune but the sensor noise bandwidth does not move. */
static void test_derivative_filter_tau_overrides_n(void)
{
    axxpid_t pid;

    (void)axxpid_init(&pid, 1, 0, 1, -1000, 1000);
    (void)axxpid_set_derivative_filter(&pid, 10);
    (void)axxpid_set_derivative_filter_tau(&pid, AXXPID_C(0.9));

    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.1));

    /* tau = 0.9 gives alpha = 0.1 and D = -1. The N = 10 setting would have
     * given tau = 0.1, alpha = 0.5 and D = -5. */
    CHECK_NEAR(axxpid_get_d_term(&pid), -1, AXXPID_C(1e-4));

    /* Clearing tau hands control back to N. */
    (void)axxpid_set_derivative_filter_tau(&pid, 0);
    (void)axxpid_reset(&pid);
    (void)axxpid_update(&pid, 0, 0, AXXPID_C(0.1));
    (void)axxpid_update(&pid, 0, 1, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_d_term(&pid), -5, AXXPID_C(1e-4));
}

/* Manual mode has to track the feed-forward too, or the transfer bumps by
 * exactly the feed-forward term. */
static void test_manual_mode_tracks_feedforward(void)
{
    axxpid_t pid;
    axxpid_real_t u;

    (void)axxpid_init(&pid, 2, 1, 0, 0, 200);
    (void)axxpid_set_feedforward_bias(&pid, 40);
    (void)axxpid_set_mode(&pid, AXXPID_MODE_MANUAL);
    (void)axxpid_set_manual_output(&pid, 60);

    (void)axxpid_update(&pid, 50, 45, AXXPID_C(0.1));
    CHECK_NEAR(axxpid_get_output(&pid), 60, AXXPID_C(1e-5));

    /* 60 = P(10) + I + FF(40), so the integrator must be sitting at 10. */
    CHECK_NEAR(axxpid_get_i_term(&pid), 10, AXXPID_C(1e-4));

    (void)axxpid_set_mode(&pid, AXXPID_MODE_AUTOMATIC);
    u = axxpid_update(&pid, 50, 45, AXXPID_C(0.1));
    CHECK_MSG(axxpid_test_near(u, AXXPID_C(60.5), AXXPID_C(0.01)),
              "transfer with feed-forward bumped to %.2f", (double)u);
}

/* A slew rate and back-calculation together: the integrator tracks the
 * rate-limited actuator rather than running ahead of it, and lets go cleanly
 * once the rate limit stops binding. */
static void test_slew_rate_with_back_calculation(void)
{
    axxpid_t pid;
    axxpid_plant_t plant;
    axxpid_real_t pv;
    axxpid_real_t previous = 0;
    axxpid_real_t worst_jump = 0;
    long i;

    axxpid_plant_init(&plant, 1, 10, 0, 0, DT);
    (void)axxpid_init(&pid, 3, 2, 0, 0, 200);
    (void)axxpid_set_antiwindup(&pid, AXXPID_ANTIWINDUP_BACK_CALCULATION,
                                AXXPID_C(1.0));
    (void)axxpid_set_output_slew_rate(&pid, 25);

    pv = plant.y;
    for (i = 0; i < 40000; ++i) {
        const axxpid_real_t u = axxpid_update(&pid, 80, pv, DT);
        const axxpid_real_t jump = (u > previous) ? (u - previous)
                                                  : (previous - u);

        if (i > 0 && jump > worst_jump) {
            worst_jump = jump;
        }
        previous = u;
        pv = axxpid_plant_step(&plant, u, DT);
    }

    /* The rate limit was honoured every single sample... */
    CHECK_MSG(worst_jump <= (AXXPID_C(25) * DT) + AXXPID_C(1e-4),
              "output jumped by %.4f, above the %.4f the slew rate allows",
              (double)worst_jump, (double)(AXXPID_C(25) * DT));
    /* ...and the loop still reaches setpoint with no standing error. */
    CHECK_NEAR(pv, 80, AXXPID_C(0.2));
}

static const axxpid_test_case_t tests[] = {
    {"P-only leaves a steady-state offset",
     test_proportional_only_leaves_offset},
    {"PI removes the offset", test_pi_removes_offset},
    {"PID step response on a plant with dead time", test_pid_step_response},
    {"anti-windup prevents saturation overshoot",
     test_saturated_start_does_not_overshoot},
    {"back-calculation settles too", test_back_calculation_settles_too},
    {"anti-windup off integrates through saturation",
     test_antiwindup_none_integrates_through_saturation},
    {"derivative filter tau overrides N",
     test_derivative_filter_tau_overrides_n},
    {"manual mode tracks feed-forward", test_manual_mode_tracks_feedforward},
    {"slew rate with back-calculation",
     test_slew_rate_with_back_calculation},
    {"feed-forward improves tracking", test_feedforward_improves_tracking},
    {"velocity feed-forward tracks a ramp",
     test_velocity_feedforward_tracks_a_ramp},
    {"load disturbance is rejected", test_disturbance_rejection},
    {"reverse-acting closed loop", test_reverse_acting_closed_loop},
    {"jittery sample time reaches the same place", test_jittery_sample_time},
    {"AxxSolder soldering-iron profile", test_axxsolder_profile},
};

AXXPID_TEST_MAIN("closed loop", tests)
