/**
 * @file  stm32_hal_snippet.c
 * @brief Dropping AxxPID into an STM32Cube project.
 *
 * Not built by the test suite - it needs the vendor HAL. Copy the parts you
 * need. Two integration styles are shown: a timer interrupt with a fixed
 * period, and a free-running main loop rate-limited by HAL_GetTick().
 *
 * To add AxxPID to a CubeIDE project: copy `include/axxpid/` and `src/` into
 * the project, add `src/axxpid.c` to the build, and add `include/` to the
 * include paths. There is nothing else to configure - the library does not
 * reference the HAL, does not allocate, and has no init order requirement.
 */

#include "main.h" /* Your CubeMX-generated header. */

#include "axxpid/axxpid.h"

/* -------------------------------------------------------------------------- */
/* Style A: fixed-rate timer interrupt (preferred)                            */
/* -------------------------------------------------------------------------- */

/* A hardware timer gives you a genuinely constant control period, which is
 * what every stability margin you tune for assumes. Prefer this. */

#define CONTROL_PERIOD_S 0.001f /* 1 kHz timer. */

static axxpid_t motor_pid;

void motor_control_init(void)
{
    /* Symmetric drive: full reverse to full forward. */
    axxpid_init(&motor_pid, 0.8f, 12.0f, 0.004f, -1000.0f, 1000.0f);

    /* An encoder differentiated at 1 kHz is noisy; filter the derivative. */
    axxpid_set_derivative_filter(&motor_pid, 12.0f);

    /* Protect the gearbox from a step command. */
    axxpid_set_output_slew_rate(&motor_pid, 20000.0f);
}

/** @brief Call from HAL_TIM_PeriodElapsedCallback for your control timer. */
void motor_control_isr(void)
{
    extern float speed_setpoint_rpm;
    extern float encoder_read_rpm(void);
    extern void motor_set_duty(int32_t duty);

    const float measured = encoder_read_rpm();
    const float command = axxpid_update(&motor_pid, speed_setpoint_rpm,
                                        measured, CONTROL_PERIOD_S);

    motor_set_duty((int32_t)command);
}

/* -------------------------------------------------------------------------- */
/* Style B: free-running main loop on HAL_GetTick()                           */
/* -------------------------------------------------------------------------- */

/* When the control loop shares the main loop with a display, USB and a menu
 * system, let axxpid_update_at() do the rate limiting.
 * It measures the true elapsed interval, so the occasional late call costs
 * you accuracy rather than correctness. */

static axxpid_t heater_pid;

void heater_control_init(void)
{
    axxpid_init(&heater_pid, 8.0f, 2.0f, 0.5f, 0.0f, 500.0f);
    axxpid_set_integral_limits(&heater_pid, -300.0f, 300.0f);
    axxpid_set_integral_band(&heater_pid, 75.0f);
    axxpid_set_integral_overshoot(&heater_pid, 7.0f, -1.0f);
    axxpid_set_integral_reset_on_zero_setpoint(&heater_pid, true);

    /* Recompute every 25 ms, however often the main loop comes round. */
    axxpid_set_sample_time(&heater_pid, 25, false);
}

void heater_control_poll(void)
{
    extern float target_temperature;
    extern float thermocouple_read_celsius(void);
    extern void heater_set_power(float power);

    if (axxpid_update_at(&heater_pid, target_temperature,
                         thermocouple_read_celsius(), HAL_GetTick())) {
        /* Only true on the samples where the controller actually ran. */
        heater_set_power(axxpid_get_output(&heater_pid));
    }
}

/* -------------------------------------------------------------------------- */
/* Telemetry                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Pack the P/I/D split for a tuning graph or a serial trace.
 *
 * Watching the three contributions separately is the single most useful thing
 * you can do while tuning: it tells you immediately whether an oscillation is
 * the proportional gain, an integrator that will not let go, or a derivative
 * amplifying sensor noise.
 */
void heater_telemetry(float *p, float *i, float *d, float *output)
{
    axxpid_terms_t terms;

    axxpid_get_terms(&heater_pid, &terms);
    *p = terms.p;
    *i = terms.i;
    *d = terms.d;
    *output = terms.output;
}

/* -------------------------------------------------------------------------- */
/* Safety                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief What to do when the loop has been stopped or the plant disturbed.
 *
 * AxxPID rejects a non-finite sample on its own, so a single bad ADC read
 * cannot poison the integrator. What it cannot know is that you switched the
 * heater off behind its back, or that the operator changed the tip. Tell it.
 */
void heater_on_fault_cleared(void)
{
    /* Start again from zero. */
    axxpid_reset(&heater_pid);
}

void heater_on_resume_from_standby(float last_known_power)
{
    /* Or resume from where the actuator actually is, with no step. */
    axxpid_reset_to(&heater_pid, last_known_power);
}
