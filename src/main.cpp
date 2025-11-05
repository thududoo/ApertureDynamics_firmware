#include <SimpleFOC.h>
#include <Arduino.h>

// --- Configuration ---
const int NUM_DENTS = 120;

// --- Anti-vibration Tuning ---
const float LPF_TIMESCALE = 0.015;   // [s] Time constant for sensor LPF. Start with 0.01
const float RAMP_TIMESCALE = 0.05;   // [s] Time constant for PID output ramp. Start with 0.05
const float DEAD_ZONE_RAD = radians(0.01); // [rad] Dead zone. Start with 0.2 degrees

// 2. Sensor Instance
MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

// 3. Motor and Driver Instances
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(7, 6, 5, 4);



/**
 * Normalizes an angle to the shortest path, from -PI to +PI.
 * This is the correct function for PID error calculation.
 */
float normalizeErrorAngle(float angle) {
  float a = fmod(angle + _PI, _2PI);
  if (a < 0) a += _2PI; // Ensure positive
  return a - _PI;     // Shift back to [-PI, PI]
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Haptic Knob with Detents Initialized (Torque Mode)");

  // Initialize the sensor
  sensor.init();

  // Initialize and link the driver
  driver.voltage_power_supply = 15;
  driver.init();
  motor.linkDriver(&driver);

  // Link the sensor to the motor
  motor.linkSensor(&sensor);

  // Configure motor control parameters
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 4;
  motor.velocity_limit = 50;

  // --- IMPROVEMENT 1: LOW-PASS FILTER (LPF) ---
  // Smooths the noisy sensor readings.
  motor.LPF_angle.Tf = LPF_TIMESCALE;

  // Configure PID
  motor.PID_velocity.P = 4; // Your P-value
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0.05; // Your D-value

  // --- IMPROVEMENT 3: PID OUTPUT RAMP ---
  // Smooths the PID output torque, reducing jitter.
  // motor.PID_velocity.output_ramp = RAMP_TIMESCALE;

  // Initialize the motor
  motor.init();

  // Force sensor direction (if needed from our previous debugging)
  // motor.sensor_direction = Direction::CCW;

  // Align motor and sensor
  Serial.print("Aligning motor and sensor... ");
  if (!motor.initFOC()) {
    Serial.println("FAILED!");
    while (1) delay(100);
  }
  Serial.println("Success.");
}

void loop() {
  // This function is crucial. It reads the sensor (and applies the LPF)
  motor.loopFOC();

  if (isnan(motor.shaft_angle) || isinf(motor.shaft_angle)) {
    Serial.println(F("!!! CRITICAL: Bad sensor read (NaN/Inf). Halting motor!"));
    motor.move(0); // Command zero torque
    motor.PID_velocity.reset(); // Reset PID
    return; // Skip the rest of this loop to prevent the crash
  }

  // --- Haptic Detent Logic ---
  const float DENT_ANGLE = _2PI / NUM_DENTS;

  // We use the filtered angle from the motor, not the raw sensor
  float current_angle = motor.shaft_angle;
  float target_angle = round(current_angle / DENT_ANGLE) * DENT_ANGLE;

  // Calculate the error using your correct normalization function
  float error;

  // Get the absolute (always positive) velocity
  float current_velocity = fabsf(motor.shaft_velocity);

  if (current_velocity > 5) {
    // Spinning fast: make detents disappear
    error = 0.;
  } else {
    // Spinning slow: calculate restoring force as normal
    error = normalizeErrorAngle(current_angle - target_angle);
  }


  // --- CORRECTED DEAD ZONE & RAMP LOGIC ---

  if (fabsf(error) < DEAD_ZONE_RAD) {
    // 1. We are IN the dead zone. Force torque to zero.
    motor.move(0);

    // 2. CRITICAL FIX: Reset the PID controller.
    // This clears the integral (I) and resets the
    // output ramp's internal state to 0.
    motor.PID_velocity.reset();

  } else {
    // 1. We are OUTSIDE the dead zone.
    // 2. Calculate torque. The PID ramp will now correctly
    //    smooth the transition from 0.
    float torque = motor.PID_velocity(-error);

    // 3. Set the torque on the motor
    motor.move(torque);
  }

  // Optional: Print values for debugging.
  Serial.print(current_angle);
  Serial.print("\t");
  Serial.print(target_angle);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.println(motor.voltage.q); // Print the actual applied voltage
}
