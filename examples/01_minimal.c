/**
 * @file  01_minimal.c
 * @brief The smallest useful AxxPID program: a PI loop on a simulated tank.
 *
 * Build and run it on your desktop to see what the controller does before you
 * put it on hardware:
 *
 *     cc -Iinclude src/axxpid.c examples/01_minimal.c -o minimal && ./minimal
 */

#include <stdio.h>

#include "axxpid/axxpid.h"

/* Control period. On real hardware this is your timer interrupt rate. */
#define DT 0.05f

int main(void)
{
    axxpid_t pid;
    float level = 0.0f;   /* Process value: tank level, 0..100 %. */
    float setpoint = 70.0f;
    int step;

    /* Three gains and the actuator range is all most loops ever need. */
    axxpid_init(&pid, 2.0f, 0.8f, 0.0f, 0.0f, 100.0f);

    (void)printf("  time    level   output\n");

    for (step = 0; step < 700; ++step) {
        /* --- this is the whole integration --------------------------- */
        const float valve = axxpid_update(&pid, setpoint, level, DT);
        /* ------------------------------------------------------------- */

        /* Stand-in for the real world: a tank that fills through the valve
         * and drains in proportion to how full it is. */
        level += DT * (0.1f * valve - 0.08f * level);

        if ((step % 50) == 0) {
            (void)printf("%6.2f s %7.2f %8.2f\n", (double)((float)step * DT),
                         (double)level, (double)valve);
        }

        /* Halfway through, ask for less. */
        if (step == 200) {
            setpoint = 40.0f;
        }
    }

    (void)printf("\nfinal level %.2f %% (setpoint %.2f %%)\n", (double)level,
                 (double)setpoint);
    return 0;
}
