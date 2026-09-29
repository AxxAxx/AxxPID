/*
 * AxxPID on Arduino: a thermostat driving a heater through a PWM pin.
 *
 * PlatformIO: add to platformio.ini
 *     lib_deps = https://github.com/AxxAxx/AxxPID.git
 *
 * Arduino IDE: the IDE only compiles what it finds under src/, so the
 * library has to be rearranged slightly. Create
 * ~/Documents/Arduino/libraries/AxxPID/ and put in it:
 *
 *     library.properties      (copied from the repository root)
 *     src/axxpid.c            (from src/)
 *     src/axxpid_tune.c       (from src/, only if you want the autotuner)
 *     src/axxpid/axxpid.h     (from include/axxpid/)
 *     src/axxpid/axxpid_tune.h
 *
 * library.properties is what tells the IDE to look inside src/ at all;
 * without it the sketch will not find the headers.
 */

#include <axxpid/axxpid.h>

const uint8_t HEATER_PIN = 9;
const uint8_t SENSOR_PIN = A0;

/* Anything outside this band means the sensor is unplugged, shorted, or
 * reading something that is not the process. See readTemperature(). */
const float TEMP_MIN_VALID = 5.0f;
const float TEMP_MAX_VALID = 150.0f;

axxpid_t pid;
float setpoint = 60.0f;   // degrees C

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_PIN, OUTPUT);
  analogWrite(HEATER_PIN, 0);

  axxpid_init(&pid, 12.0f, 0.4f, 8.0f, 0.0f, 255.0f);

  // A sensor read through the ADC is noisy, so filter the derivative rather
  // than letting it amplify that noise into the PWM.
  //
  // Note this uses the explicit time constant, not the N form. N sets the
  // filter to Td/N, where Td = kd/kp -- here 8/12 = 0.67 s, so even N = 10
  // leaves a 67 ms filter against a 250 ms sample period, which does almost
  // nothing. The N form only bites when Td is much longer than the sample
  // period. When it is not, say what you want in seconds.
  axxpid_set_derivative_filter_tau(&pid, 1.0f);

  // Recompute every 250 ms; a thermal loop gains nothing from going faster.
  axxpid_set_sample_time(&pid, 250, false);
}

// Returns NAN if the reading is not plausible.
//
// This matters more than it looks. AxxPID rejects a NaN or an infinity, but
// analogRead() cannot produce either: an unplugged sensor reads 0, which
// looks like a perfectly valid 0 degrees C, and the controller will hold the
// heater at full power forever trying to warm it up. Only your code knows
// what a plausible reading is.
float readTemperature() {
  // An LM35-style linear sensor on a 10-bit ADC: 10 mV per degree C.
  // Replace with whatever your sensor actually needs.
  float celsius = analogRead(SENSOR_PIN) * (5.0f / 1023.0f) * 100.0f;

  if (celsius < TEMP_MIN_VALID || celsius > TEMP_MAX_VALID) {
    return NAN;
  }
  return celsius;
}

void loop() {
  float temperature = readTemperature();

  if (isnan(temperature)) {
    analogWrite(HEATER_PIN, 0);   // heater off, and stay off
    axxpid_reset(&pid);           // do not carry the integral across a fault
    Serial.println(F("sensor fault - heater off"));
    delay(250);
    return;
  }

  // Call as often as you like: axxpid_update_at() runs the control law only
  // when the sample time has elapsed, using the true measured interval.
  if (axxpid_update_at(&pid, setpoint, temperature, millis())) {
    analogWrite(HEATER_PIN, (int)axxpid_get_output(&pid));

    axxpid_terms_t terms;
    axxpid_get_terms(&pid, &terms);
    Serial.print(F("pv "));  Serial.print(terms.measurement);
    Serial.print(F("  P ")); Serial.print(terms.p);
    Serial.print(F("  I ")); Serial.print(terms.i);
    Serial.print(F("  D ")); Serial.print(terms.d);
    Serial.print(F("  u ")); Serial.println(terms.output);
  }

  // Everything else your sketch does goes here, uninterrupted.
}
