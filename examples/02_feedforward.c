/**
 * @file  02_feedforward.c
 * @brief Feed-forward on a heater, three ways, side by side.
 *
 * Feed-forward is the single biggest win available to most temperature loops
 * and it is almost never in a PID library. The idea is simple: if you already
 * know roughly how much power a given setpoint needs, do not make the
 * integrator discover it from scratch every time.
 *
 *     cc -Iinclude src/axxpid.c examples/02_feedforward.c -o ff && ./ff
 */

#include <stdio.h>

#include "axxpid/axxpid.h"

#define DT       0.05f
#define AMBIENT  25.0f
#define TAU      20.0f  /* Thermal time constant, seconds. */
#define HEATER_K 0.6f   /* Degrees above ambient per unit of power. */

/**
 * @brief A gain-scheduled feed-forward hook.
 *
 * Losses to the surroundings grow with the temperature difference, so the
 * power needed to hold a setpoint is not proportional to the setpoint itself
 * but to how far above ambient it is. Anything you can express in C can go
 * here: a polynomial, a lookup table, an inverse plant model, or a term from
 * a measured disturbance such as airflow.
 */
/* The signature has to match axxpid_ff_fn_t exactly. axxpid_real_t is float
 * unless you build with -DAXXPID_USE_DOUBLE=1; writing `float` here compiles
 * in the default build and then fails to compile in the other one. */
static axxpid_real_t holding_power(axxpid_real_t setpoint,
                                   axxpid_real_t measurement,
                                   void *user)
{
    (void)measurement;
    (void)user;
    return (setpoint - AMBIENT) / HEATER_K;
}

/** @brief Run one heater loop and report the accumulated absolute error. */
static float run(axxpid_t *pid, const char *label)
{
    float temperature = AMBIENT;
    float integrated_error = 0.0f;
    int step;

    for (step = 0; step < 16000; ++step) {
        const float setpoint = 240.0f;
        const float power = axxpid_update(pid, setpoint, temperature, DT);
        float error;

        temperature += (DT / TAU) *
                       ((HEATER_K * power) + AMBIENT - temperature);

        /* Score only the first 200 s: feed-forward is about where the
         * actuator starts, not where it ends up. Given long enough all
         * three land on the setpoint. */
        error = setpoint - temperature;
        if (step < 4000) {
            integrated_error += (error < 0.0f ? -error : error) * DT;
        }
    }

    (void)printf("%-28s final %7.2f C   IAE %8.1f   I term %7.2f\n", label,
                 (double)temperature, (double)integrated_error,
                 (double)axxpid_get_i_term(pid));
    return integrated_error;
}

int main(void)
{
    axxpid_t plain;
    axxpid_t with_gain;
    axxpid_t with_hook;

    /* 1. No feed-forward. The integrator has to climb all the way to the
     *    holding power on its own, which is slow and leaves a long tail. */
    axxpid_init(&plain, 1.5f, 0.05f, 0.0f, 0.0f, 600.0f);

    /* 2. Setpoint-proportional feed-forward. One number, and the actuator
     *    starts in roughly the right place - though a plain gain cannot
     *    express the ambient offset, so it aims slightly high and the
     *    integrator has to trim it back down. */
    axxpid_init(&with_gain, 1.5f, 0.05f, 0.0f, 0.0f, 600.0f);
    axxpid_set_feedforward_gains(&with_gain, 1.0f / HEATER_K, 0.0f);

    /* 3. A hook that models the plant properly, including the ambient offset
     *    a plain gain cannot represent. */
    axxpid_init(&with_hook, 1.5f, 0.05f, 0.0f, 0.0f, 600.0f);
    axxpid_set_feedforward_fn(&with_hook, holding_power, NULL);

    (void)printf("Heater holding 240 C, 800 s run (first 200 s scored)\n\n");
    (void)run(&plain, "PI only");
    (void)run(&with_gain, "PI + setpoint-gain FF");
    (void)run(&with_hook, "PI + modelled FF");

    (void)printf(
        "\nNote how the integral term shrinks as the feed-forward takes over\n"
        "the steady-state work. That is the point: the integrator is left to\n"
        "handle what the model got wrong, not the whole job.\n");
    return 0;
}
