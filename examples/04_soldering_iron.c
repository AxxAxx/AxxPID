/**
 * @file  04_soldering_iron.c
 * @brief Controlling a one-way actuator, and what each setting is for.
 *
 * AxxPID's less usual features - the integral engagement band, the asymmetric
 * integral gain, the explicit integral clamp - all exist because a soldering
 * iron needs them. The defining problem is that a heater is a one-way
 * actuator: it can pour power in at will, but the only way down is to wait
 * for the tip to lose heat to the air. Authority used on the way up has no
 * counterpart on the way down.
 *
 * The plant here is a two-node thermal model, because that is what actually
 * makes a soldering iron overshoot: the heating element runs far hotter than
 * the tip, and keeps dumping its stored heat into the tip after the
 * controller has already backed off.
 *
 *     cc -Iinclude src/axxpid.c examples/04_soldering_iron.c -o iron && ./iron
 */

#include <stdio.h>

#include "axxpid/axxpid.h"

#define DT        0.025f /* 25 ms control period.                           */
#define AMBIENT   25.0f
#define MAX_POWER 500.0f /* Full heater power, in output units.             */

/* Two-node thermal model: heating element -> tip -> air. */
#define ELEMENT_MASS  1.0f  /* Heat capacity of the element.                */
#define TIP_MASS      9.0f  /* Heat capacity of the tip: much larger.       */
#define ELEMENT_TO_TIP 1.2f /* Conductance from element into tip.           */
#define TIP_TO_AIR    0.15f /* Conductance from tip into the surroundings.  */

typedef struct {
    float element; /* Element temperature, C. */
    float tip;     /* Tip temperature, C - this is what the sensor reads. */
} iron_t;

static void iron_init(iron_t *iron)
{
    iron->element = AMBIENT;
    iron->tip = AMBIENT;
}

/**
 * @brief Advance the iron one control period.
 * @param iron  The iron.
 * @param power Requested power, 0..MAX_POWER. Negative is clamped away,
 *              because a heater genuinely cannot cool.
 * @param load  Heat being drawn off by the joint being soldered.
 * @return The tip temperature.
 */
static float iron_step(iron_t *iron, float power, float load)
{
    float conducted;

    if (power < 0.0f) {
        power = 0.0f;
    }
    conducted = ELEMENT_TO_TIP * (iron->element - iron->tip);

    iron->element += DT * ((power - conducted) / ELEMENT_MASS);
    iron->tip += DT * ((conducted - (TIP_TO_AIR * (iron->tip - AMBIENT)) -
                        load) / TIP_MASS);
    return iron->tip;
}

/** @brief Configure a controller for a small, one-way thermal load. */
static void configure_soldering_iron(axxpid_t *pid)
{
    /* Gains for a T245-sized tip, and the heater PWM range. */
    axxpid_init(pid, 8.0f, 2.0f, 0.5f, 0.0f, MAX_POWER);

    /* Cap how much authority the integrator can ever hold. Without this the
     * integrator alone could command full power indefinitely. */
    axxpid_set_integral_limits(pid, -300.0f, 300.0f);

    /* Park the integrator while the tip is more than 75 C below target: the
     * heater is flat out during a heat-up anyway, so anything the integrator
     * accumulated there is a debt it would have to pay back as overshoot. */
    axxpid_set_integral_band(pid, 75.0f);

    /* Above target, drain the integrator seven times faster than it filled.
     * The -1 C threshold keeps a small steady-state error out of the fast
     * path, so the loop does not hunt when it is already where it should be. */
    axxpid_set_integral_overshoot(pid, 7.0f, -1.0f);

    /* A setpoint of zero is how the state machine says "sleep"; the
     * integrator lets go completely so the tip restarts from a clean state. */
    axxpid_set_integral_reset_on_zero_setpoint(pid, true);

    /* Only needed if you drive the loop with axxpid_update_at(). */
    axxpid_set_sample_time(pid, 25, false);
}

/* -------------------------------------------------------------------------- */

/** @brief Cold start to the working temperature, with the P/I/D split. */
static void scenario_heat_up(void)
{
    axxpid_t pid;
    iron_t iron;
    float tip;
    float peak = AMBIENT;
    int step;

    configure_soldering_iron(&pid);
    iron_init(&iron);
    tip = iron.tip;

    (void)printf("--- Cold start to 330 C ---\n");
    (void)printf("   time      tip  element        P        I        D"
                 "    power\n");

    for (step = 0; step < 2400; ++step) {
        const float power = axxpid_update(&pid, 330.0f, tip, DT);

        tip = iron_step(&iron, power, 0.0f);
        if (tip > peak) {
            peak = tip;
        }

        if ((step % 300) == 0) {
            /* The breakdown you would graph on a display while tuning. */
            axxpid_terms_t terms;

            axxpid_get_terms(&pid, &terms);
            (void)printf("%7.1f s %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f\n",
                         (double)((float)step * DT), (double)tip,
                         (double)iron.element, (double)terms.p,
                         (double)terms.i, (double)terms.d,
                         (double)terms.output);
        }
    }

    (void)printf("peak %.1f C (overshoot %+.1f C), settled at %.1f C\n\n",
                 (double)peak, (double)(peak - 330.0f), (double)tip);
}

/**
 * @brief The case the asymmetric integral gain exists for.
 *
 * Solder a heavy joint and the tip is dragged down; the integrator charges up
 * to hold temperature against the load. Lift the iron and that stored
 * integral is still commanding power into a tip that no longer needs it.
 * How fast the integrator can be made to let go is the whole ballgame,
 * because the actuator cannot help - it is already at zero.
 */
static void scenario_load_transient(void)
{
    const struct {
        const char *label;
        float asymmetry;
    } variants[] = {
        {"symmetric integral", 1.0f},
        {"asymmetric, 3x", 3.0f},
        {"asymmetric, 7x", 7.0f},
    };
    size_t v;

    (void)printf("--- Soldering a heavy joint, then lifting the iron ---\n");

    for (v = 0; v < sizeof(variants) / sizeof(variants[0]); ++v) {
        axxpid_t pid;
        iron_t iron;
        float tip;
        float peak = 0.0f;
        float charged;
        int step;

        configure_soldering_iron(&pid);
        axxpid_set_integral_overshoot(&pid, variants[v].asymmetry, -1.0f);
        iron_init(&iron);
        tip = iron.tip;

        /* Settle at working temperature. */
        for (step = 0; step < 8000; ++step) {
            tip = iron_step(&iron, axxpid_update(&pid, 330.0f, tip, DT), 0.0f);
        }
        /* 80 s on a big joint drawing heat away. */
        for (step = 0; step < 3200; ++step) {
            tip = iron_step(&iron, axxpid_update(&pid, 330.0f, tip, DT), 55.0f);
        }
        charged = axxpid_get_i_term(&pid);

        /* Iron lifted: the load vanishes instantly. */
        for (step = 0; step < 8000; ++step) {
            tip = iron_step(&iron, axxpid_update(&pid, 330.0f, tip, DT), 0.0f);
            if (tip > peak) {
                peak = tip;
            }
        }

        (void)printf("%-30s integral at lift %6.1f   overshoot %+5.1f C\n",
                     variants[v].label, (double)charged,
                     (double)(peak - 330.0f));
    }

    (void)printf(
        "\nDraining the integrator faster than it filled cuts the overshoot\n"
        "with no cost to the heat-up, because the fast path only ever runs\n"
        "while the tip is already above target.\n\n");
}

/** @brief Setpoint zero means off, and the controller must let go completely. */
static void scenario_sleep(void)
{
    axxpid_t pid;
    iron_t iron;
    float tip;
    int step;

    configure_soldering_iron(&pid);
    iron_init(&iron);
    tip = iron.tip;

    for (step = 0; step < 8000; ++step) {
        tip = iron_step(&iron, axxpid_update(&pid, 330.0f, tip, DT), 0.0f);
    }
    (void)printf("--- Sleep ---\nrunning: integral %.1f, power %.1f\n",
                 (double)axxpid_get_i_term(&pid),
                 (double)axxpid_get_output(&pid));

    (void)axxpid_update(&pid, 0.0f, tip, DT);
    (void)printf("setpoint 0: integral %.1f, power %.1f\n",
                 (double)axxpid_get_i_term(&pid),
                 (double)axxpid_get_output(&pid));
}

int main(void)
{
    scenario_heat_up();
    scenario_load_transient();
    scenario_sleep();

    (void)printf(
        "\nA note on the integral band: on this plant kp * error already\n"
        "saturates the heater during the whole heat-up, so conditional\n"
        "anti-windup blocks the integrator anyway and the band changes\n"
        "nothing. It earns its place on a loop with a lower kp, where the\n"
        "output is not saturated but the error is still far too large for\n"
        "the integrator to have anything useful to say. Measure before you\n"
        "reach for it.\n");
    return 0;
}
