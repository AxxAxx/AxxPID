/**
 * @file  03_autotune.c
 * @brief Relay autotune: measure the plant, pick gains, run the loop.
 *
 *     cc -Iinclude src/axxpid.c src/axxpid_tune.c examples/03_autotune.c \
 *        -o autotune && ./autotune
 */

#include <stdio.h>

#include "axxpid/axxpid_tune.h"

#define DT 0.02f

/** @brief A lagged process with a transport delay, as most real ones have. */
typedef struct {
    float value;
    float delay[128];
    int head;
} process_t;

static float process_step(process_t *p, float input)
{
    const float delayed = p->delay[p->head];
    p->delay[p->head] = input;
    p->head = (p->head + 1) % 128;

    p->value += (DT / 8.0f) * ((0.8f * delayed) - p->value);
    return p->value;
}

int main(void)
{
    process_t plant;
    axxpid_relay_t relay;
    axxpid_relay_config_t relay_cfg;
    axxpid_relay_state_t state = AXXPID_RELAY_RUNNING;
    axxpid_gains_t gains;
    axxpid_t pid;
    /* These are written through by the library, so they must be its own
     * scalar type. A `float` here is silently four bytes too small when the
     * library is built with -DAXXPID_USE_DOUBLE=1, and the write runs off the
     * end of it. */
    axxpid_real_t ku = 0;
    axxpid_real_t tu = 0;
    float measurement;
    long i;

    for (i = 0; i < 128; ++i) {
        plant.delay[i] = 0.0f;
    }
    plant.value = 0.0f;
    plant.head = 0;

    /* ---------------------------------------------------------------- */
    /* Step 1: provoke a limit cycle and measure it.                     */
    /* ---------------------------------------------------------------- */

    axxpid_relay_config_default(&relay_cfg);
    relay_cfg.setpoint = 50.0f;    /* Tune around the operating point you  */
    relay_cfg.output_bias = 60.0f; /* actually run at - plants are not     */
    relay_cfg.output_step = 15.0f; /* linear, and gains are local.         */
    relay_cfg.hysteresis = 0.2f;   /* Just above the measurement noise.    */
    relay_cfg.cycles = 4;
    relay_cfg.settle_cycles = 2;
    relay_cfg.timeout = 1200.0f;

    if (axxpid_relay_init(&relay, &relay_cfg) != AXXPID_OK) {
        (void)printf("bad relay configuration\n");
        return 1;
    }

    (void)printf("Autotuning (the process will oscillate on purpose)...\n");

    measurement = plant.value;
    for (i = 0; i < 2000000L; ++i) {
        axxpid_real_t output = 0; /* written through by the library */

        state = axxpid_relay_update(&relay, measurement, DT, &output);
        if (state != AXXPID_RELAY_RUNNING) {
            break;
        }
        measurement = process_step(&plant, output);
    }

    if (state != AXXPID_RELAY_DONE) {
        (void)printf("autotune did not converge (state %d)\n", (int)state);
        return 1;
    }

    axxpid_relay_result(&relay, &ku, &tu);
    (void)printf("  ultimate gain Ku   = %.3f\n", (double)ku);
    (void)printf("  ultimate period Tu = %.3f s\n", (double)tu);

    /* This example knows its own plant, so it can show how good the estimate
     * is. Solving the FOPDT phase condition for K = 0.8, L = 2.56 s, T = 8 s
     * gives the true values below. A relay estimate reads Ku low - the
     * describing function keeps only the fundamental of a square wave - and
     * that is fine, because every tuning rule has a safety factor built in
     * and a low Ku gives gentle gains. */
    (void)printf("  (true values for this plant: Ku = 6.955, Tu = 9.183 s;\n");
    (void)printf("   a relay reads Ku low by 10-20%%, which is expected)\n\n");

    /* ---------------------------------------------------------------- */
    /* Step 2: turn the measurement into gains.                          */
    /* ---------------------------------------------------------------- */

    (void)printf("%-24s %8s %8s %8s\n", "rule", "kp", "ki", "kd");
    {
        const struct {
            const char *name;
            axxpid_rule_t rule;
        } table[] = {
            {"Ziegler-Nichols PID", AXXPID_RULE_ZN_PID},
            {"Pessen (fastest)", AXXPID_RULE_PESSEN},
            {"some overshoot", AXXPID_RULE_SOME_OVERSHOOT},
            {"no overshoot", AXXPID_RULE_NO_OVERSHOOT},
            {"Tyreus-Luyben (robust)", AXXPID_RULE_TYREUS_LUYBEN_PID},
        };
        size_t k;

        for (k = 0; k < sizeof(table) / sizeof(table[0]); ++k) {
            const axxpid_gains_t g = axxpid_relay_gains(&relay, table[k].rule);
            (void)printf("%-24s %8.3f %8.3f %8.3f\n", table[k].name,
                         (double)g.kp, (double)g.ki, (double)g.kd);
        }
    }

    /* Tyreus-Luyben is the sane default. On this plant it is about
     * three-quarters of the Ziegler-Nichols proportional gain but roughly six
     * times slower on the integral, which is where the calm comes from. Start
     * here and tighten only if you need to. */
    gains = axxpid_relay_gains(&relay, AXXPID_RULE_TYREUS_LUYBEN_PID);

    /* ---------------------------------------------------------------- */
    /* Step 3: run the tuned loop.                                       */
    /* ---------------------------------------------------------------- */

    axxpid_init(&pid, 0.0f, 0.0f, 0.0f, 0.0f, 150.0f);
    axxpid_set_derivative_filter(&pid, 10.0f);
    axxpid_tune_apply(&pid, &gains);

    plant.value = 0.0f;
    plant.head = 0;
    for (i = 0; i < 128; ++i) {
        plant.delay[i] = 0.0f;
    }

    (void)printf("\nStep to 80 with the tuned gains:\n");
    measurement = plant.value;
    for (i = 0; i < 15000; ++i) {
        measurement =
            process_step(&plant, axxpid_update(&pid, 80.0f, measurement, DT));
        if ((i % 1500) == 0) {
            (void)printf("  t = %6.1f s   pv = %7.3f\n",
                         (double)((float)i * DT), (double)measurement);
        }
    }
    (void)printf("  settled at %.3f\n", (double)measurement);

    return 0;
}
