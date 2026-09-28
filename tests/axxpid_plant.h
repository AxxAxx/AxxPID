/**
 * @file  axxpid_plant.h
 * @brief Simulated processes used by the closed-loop tests.
 */

#ifndef AXXPID_PLANT_H
#define AXXPID_PLANT_H

#include <stddef.h>
#include <string.h>

#include "axxpid/axxpid.h"

/* -------------------------------------------------------------------------- */
/* Plant models                                                               */
/* -------------------------------------------------------------------------- */

/** @brief Maximum dead-time slots in ::axxpid_plant_t. */
#define AXXPID_PLANT_DELAY_MAX 4096

/**
 * @brief First-order-plus-dead-time process with an optional ambient term.
 *
 * @f$ \tau \dot{y} = K u(t - L) + y_\infty - y @f$
 *
 * With @c ambient set, the process decays towards @c ambient with no input,
 * which is what makes it a realistic stand-in for a heater: it can be driven
 * up quickly but only cools at its own pace.
 */
typedef struct {
    axxpid_real_t y;       /**< Current process value. */
    axxpid_real_t gain;    /**< Steady-state gain K, per unit of output. */
    axxpid_real_t tau;     /**< Time constant in seconds. */
    axxpid_real_t ambient; /**< Value the process decays to with zero input. */

    axxpid_real_t dead_time;                        /**< Dead time L, seconds. */
    axxpid_real_t delay[AXXPID_PLANT_DELAY_MAX];    /**< Transport delay line. */
    size_t delay_len;                               /**< Slots in use. */
    size_t delay_head;                              /**< Write cursor. */
} axxpid_plant_t;

/**
 * @brief Initialise a plant.
 * @param p         Plant.
 * @param gain      Steady-state gain.
 * @param tau       Time constant in seconds.
 * @param ambient   Resting value.
 * @param dead_time Dead time in seconds (0 for none).
 * @param dt        The fixed timestep the plant will be stepped with.
 */
static void axxpid_plant_init(axxpid_plant_t *p,
                              axxpid_real_t gain,
                              axxpid_real_t tau,
                              axxpid_real_t ambient,
                              axxpid_real_t dead_time,
                              axxpid_real_t dt)
{
    size_t i;

    memset(p, 0, sizeof(*p));
    p->y = ambient;
    p->gain = gain;
    p->tau = tau;
    p->ambient = ambient;
    p->dead_time = dead_time;

    p->delay_len = (size_t)((dead_time / dt) + AXXPID_C(0.5));
    if (p->delay_len > AXXPID_PLANT_DELAY_MAX) {
        p->delay_len = AXXPID_PLANT_DELAY_MAX;
    }
    for (i = 0; i < p->delay_len; ++i) {
        p->delay[i] = 0;
    }
    p->delay_head = 0;
}

/**
 * @brief Advance the plant one timestep under output @p u.
 * @return The new process value.
 */
static axxpid_real_t axxpid_plant_step(axxpid_plant_t *p,
                                       axxpid_real_t u,
                                       axxpid_real_t dt)
{
    axxpid_real_t effective = u;

    if (p->delay_len > 0u) {
        effective = p->delay[p->delay_head];
        p->delay[p->delay_head] = u;
        p->delay_head = (p->delay_head + 1u) % p->delay_len;
    }

    p->y += (dt / p->tau) * ((p->gain * effective) + p->ambient - p->y);
    return p->y;
}

#endif /* AXXPID_PLANT_H */
