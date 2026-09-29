/*
 * 01_minimal.c - the smallest useful AxxPID program.
 *
 * A PI loop filling a simulated tank. Build and run it on your desktop to
 * watch a loop settle before you put anything on hardware:
 *
 *     cc -Iinclude src/axxpid.c examples/01_minimal.c -o minimal && ./minimal
 */

#include <stdio.h>

#include "axxpid/axxpid.h"

#define DT 0.05f /* Seconds per step. On hardware, your timer rate. */

/* Stand-in for the real world: a tank that fills through the valve and drains
 * in proportion to how full it is. Replace with your own process. */
static float tank_step(float level, float valve)
{
    return level + DT * (0.1f * valve - 0.08f * level);
}

int main(void)
{
    axxpid_t pid;
    float level = 0.0f;     /* What the sensor reads: tank level, 0..100 %. */
    float setpoint = 70.0f; /* What we want it to be.                       */
    int step;

    /*          kp    ki    kd     min     max  */
    axxpid_init(&pid, 2.0f, 0.8f, 0.0f, 0.0f, 100.0f);

    printf("  time    level   valve\n");

    for (step = 0; step < 1600; ++step) {
        /* ---- the whole integration ---- */
        float valve = axxpid_update(&pid, setpoint, level, DT);
        /* ------------------------------- */

        level = tank_step(level, valve);

        if (step % 100 == 0) {
            printf("%6.2f s %7.2f %7.2f\n", step * DT, level, valve);
        }

        if (step == 400) {
            setpoint = 40.0f; /* A third of the way in, ask for less. */
        }
    }

    printf("\nfinal level %.2f %%, setpoint %.2f %%\n", level, setpoint);
    return 0;
}
