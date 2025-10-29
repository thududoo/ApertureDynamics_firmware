// 1. Include the SimpleFOC library
#include <SimpleFOC.h>
#include <Arduino.h>
// --- Configuration ---
const int NUM_DENTS = 120; // How many "clicks" or detents per revolution

// 2. Sensor Instance
// Using the generic I2C sensor class for the MT6701
// Constructor: MagneticSensorI2C(I2C_address, bits_per_revolution, angle_register_msb)
MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

// 3. Motor and Driver Instances
// BLDCMotor(pole_pairs)
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, enable_pin)
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

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
  motor.controller = MotionControlType::torque; // We will control the motor's torque directly
  motor.voltage_limit = 4;   // Volts - Tweak this for more/less strength
  motor.velocity_limit = 50; // Rad/s

  motor.PID_velocity.P = 10;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0.05;

  // Initialize the motor
  motor.init();
  motor.initFOC();

  Serial.println("Motor and Sensor ready. Turn the knob to feel the detents.");
}

void loop() {
  // This function is crucial. It reads the sensor and calculates the motor's electrical angle.
  motor.loopFOC();

  // --- Haptic Detent Logic (Torque Mode) ---

  // Calculate the angle of each detent
  const float DENT_ANGLE = _2PI / NUM_DENTS;

  // Get the current, real angle from the sensor
  float current_angle = sensor.getAngle();

  // Find the closest detent angle
  float target_angle = round(current_angle / DENT_ANGLE) * DENT_ANGLE;

  // Calculate the angular error from the nearest detent.
  // _normalizeAngle is CRITICAL to prevent large torque spikes at the 0-2PI crossover.
  float error = current_angle - target_angle;

  // Calculate the torque to apply. This acts like a spring-damper system.
  // The first term is a proportional force that pulls the knob towards the detent (the "spring").
  // The second term is a damping force that resists motion, preventing oscillation.
  float torque = motor.PID_velocity(-error);

  // Set the torque on the motor. In torque mode, motor.move() takes a voltage value.
  motor.move(torque);

  // Optional: Print values for debugging. It's best to keep this commented out for performance.
  /*
  Serial.print(current_angle);
  Serial.print("\t");
  Serial.print(target_angle);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.println(torque);
  */
}
