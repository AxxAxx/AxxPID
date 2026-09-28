/*
 * AxxPID on Arduino: a thermostat driving a heater through a PWM pin.
 *
 * PlatformIO: add to platformio.ini
 *     lib_deps = https://github.com/<you>/AxxPID.git
 *
 * Arduino IDE: copy include/axxpid/ and src/*.c into
 * ~/Documents/Arduino/libraries/AxxPID/src/ so the layout is
 * src/axxpid/axxpid.h and src/axxpid.c.
 */

#include <axxpid/axxpid.h>

const uint8_t HEATER_PIN = 9;
const uint8_t SENSOR_PIN = A0;

axxpid_t pid;
float setpoint = 60.0f;   // degrees C

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_PIN, OUTPUT);

  axxpid_init(&pid, 12.0f, 0.4f, 8.0f, 0.0f, 255.0f);

  // A cheap thermistor read through an 8-bit ADC is noisy, so filter the
  // derivative rather than letting it amplify the noise.
  axxpid_set_derivative_filter(&pid, 10.0f);

  // Recompute every 250 ms; a thermal loop gains nothing from going faster.
  axxpid_set_sample_time(&pid, 250, false);
}

float readTemperature() {
  // Replace with whatever your sensor actually needs.
  return analogRead(SENSOR_PIN) * (5.0f / 1023.0f) * 100.0f;
}

void loop() {
  // Call as often as you like: axxpid_update_at() runs the control law only
  // when the sample time has elapsed, using the true measured interval.
  if (axxpid_update_at(&pid, setpoint, readTemperature(), millis())) {
    analogWrite(HEATER_PIN, (int)axxpid_get_output(&pid));

    axxpid_terms_t terms;
    axxpid_get_terms(&pid, &terms);
    Serial.print(F("pv "));   Serial.print(setpoint - terms.error);
    Serial.print(F("  P ")); Serial.print(terms.p);
    Serial.print(F("  I ")); Serial.print(terms.i);
    Serial.print(F("  D ")); Serial.print(terms.d);
    Serial.print(F("  u ")); Serial.println(terms.output);
  }

  // Everything else your sketch does goes here, uninterrupted.
}
