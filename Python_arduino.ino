#include <AccelStepper.h>

// Stepper Motor Pins
#define PUL1_PIN 9
#define DIR1_PIN 8
#define PUL2_PIN 7
#define DIR2_PIN 6

// Limit Switch Pins
#define LIMIT_SWITCH_X_PIN 2
#define LIMIT_SWITCH_Y_PIN 3
#define FINAL_SWITCH_X_PIN 4
#define FINAL_SWITCH_Y_PIN 5

// Water Pump Pins (L298N Motor Driver)
#define PUMP1_ENABLE_PIN 23
#define PUMP1_CONTROL_PIN1 25
#define PUMP1_CONTROL_PIN2 27
#define PUMP2_ENABLE_PIN 33
#define PUMP2_CONTROL_PIN1 29
#define PUMP2_CONTROL_PIN2 31
#define PUMP3_ENABLE_PIN 35
#define PUMP3_CONTROL_PIN1 37
#define PUMP3_CONTROL_PIN2 39

// Motor Wheel Pins (HW231 Motor Driver)
#define WHEEL_MOTOR_IN1 10
#define WHEEL_MOTOR_IN2 11
#define WHEEL_MOTOR_IN3 12
#define WHEEL_MOTOR_IN4 13

// Workspace dimensions (in mm)
const float WORKSPACE_WIDTH = 200.0;  // Width of the workspace
const float WORKSPACE_HEIGHT = 30.0; // Height of the workspace

// Stepper motor step size
const float STEP_SIZE_X = 0.01; // mm per step for X-axis
const float STEP_SIZE_Y = 0.01; // mm per step for Y-axis

// Stepper motor objects
AccelStepper stepperX(AccelStepper::DRIVER, PUL1_PIN, DIR1_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, PUL2_PIN, DIR2_PIN);

// Safety flag
bool safetyTriggered = false;

// Motor wheel speed (PWM values: 0 - 255)
int motorWheelSpeed = 50;

void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);

  // Configure stepper motor parameters
  stepperX.setMaxSpeed(1000); // Maximum speed for X-axis stepper motor
  stepperX.setAcceleration(500); // Acceleration for X-axis stepper motor
  stepperY.setMaxSpeed(500); // Maximum speed for Y-axis stepper motor
  stepperY.setAcceleration(300); // Acceleration for Y-axis stepper motor

  // Configure limit switch pins
  pinMode(LIMIT_SWITCH_X_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y_PIN, INPUT_PULLUP);
  pinMode(FINAL_SWITCH_X_PIN, INPUT_PULLUP);
  pinMode(FINAL_SWITCH_Y_PIN, INPUT_PULLUP);

  // Configure pump motor pins
  pinMode(PUMP1_ENABLE_PIN, OUTPUT);
  pinMode(PUMP1_CONTROL_PIN1, OUTPUT);
  pinMode(PUMP1_CONTROL_PIN2, OUTPUT);
  pinMode(PUMP2_ENABLE_PIN, OUTPUT);
  pinMode(PUMP2_CONTROL_PIN1, OUTPUT);
  pinMode(PUMP2_CONTROL_PIN2, OUTPUT);
  pinMode(PUMP3_ENABLE_PIN, OUTPUT);
  pinMode(PUMP3_CONTROL_PIN1, OUTPUT);
  pinMode(PUMP3_CONTROL_PIN2, OUTPUT);

  // Configure motor wheel pins
  pinMode(WHEEL_MOTOR_IN1, OUTPUT);
  pinMode(WHEEL_MOTOR_IN2, OUTPUT);
  pinMode(WHEEL_MOTOR_IN3, OUTPUT);
  pinMode(WHEEL_MOTOR_IN4, OUTPUT);

  // Ensure all pumps and motors are off initially
  deactivateAllPumps();
  stopMotorWheels();

  Serial.println("Stepper motors, pumps, and motor wheels ready");

  // Start motor wheels moving
  startMotorWheels();
}

void loop() {
  if (Serial.available() > 0 && !safetyTriggered) {
    // Read input from Python (coordinates and pump index)
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace and newline characters

    if (input.length() > 0) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        float norm_x = input.substring(0, firstComma).toFloat();  // Extract X-coordinate
        float norm_y = input.substring(firstComma + 1, secondComma).toFloat(); // Extract Y-coordinate
        int pumpIndex = input.substring(secondComma + 1).toInt(); // Extract pump index

        if (norm_x >= 0.0 && norm_x <= 1.0 && norm_y >= 0.0 && norm_y <= 1.0) {
          // Stop motor wheels before moving stepper motors
          stopMotorWheels();

          // Convert normalized coordinates to steps
          long steps_x = (norm_x * WORKSPACE_WIDTH) / STEP_SIZE_X;
          long steps_y = (norm_y * WORKSPACE_HEIGHT) / STEP_SIZE_Y;

          // Move to target position
          delay(2000);
          moveToPositionSeparate(steps_x, steps_y);

          // Activate the specified water pump
          activatePump(pumpIndex);

          // Simulate pump operation for 5 seconds
          delay(2000);

          // Deactivate the pump
          deactivateAllPumps();

          // Return to initial position
          returnToInitialPosition();

          // Restart motor wheels for 3 seconds
          startMotorWheels();
          delay(3000);
          stopMotorWheels();

          Serial.println("Move complete");
        } else {
          Serial.println("Error: Coordinates out of bounds");
        }
      } else {
        Serial.println("Error: Invalid input format");
      }
    } else {
      Serial.println("Error: Empty input");
    }
  }
}

// Move stepper motors to the target position separately
void moveToPositionSeparate(long steps_x, long steps_y) {
  stepperX.moveTo(steps_x);
  while (stepperX.distanceToGo() != 0 && !safetyTriggered) {
    checkSafety();
    stepperX.run();
  }

  stepperY.moveTo(-steps_y); // Negate steps_y for opposite direction
  while (stepperY.distanceToGo() != 0 && !safetyTriggered) {
    checkSafety();
    stepperY.run();
  }
}

// Return stepper motors to the initial position using limit switches
void returnToInitialPosition() {
  stepperX.move(-100000);
  while (digitalRead(LIMIT_SWITCH_X_PIN) != LOW && !safetyTriggered) {
    checkSafety();
    stepperX.run();
  }
  stepperX.stop();
  stepperX.setCurrentPosition(0);

  stepperY.move(100000);
  while (digitalRead(LIMIT_SWITCH_Y_PIN) != LOW && !safetyTriggered) {
    checkSafety();
    stepperY.run();
  }
  stepperY.stop();
  stepperY.setCurrentPosition(0);

  Serial.println("Returned to initial position");
}

// Check safety switches
void checkSafety() {
  if (digitalRead(FINAL_SWITCH_X_PIN) == LOW || digitalRead(FINAL_SWITCH_Y_PIN) == LOW) {
    safetyTriggered = true;
    stepperX.stop();
    stepperY.stop();
    stopMotorWheels();
    deactivateAllPumps();
    Serial.println("Safety trigger activated!");
  }
}

// Activate pump
void activatePump(int pumpIndex) {
  deactivateAllPumps();

  switch (pumpIndex) {
    case 1:
      digitalWrite(PUMP1_ENABLE_PIN, HIGH);
      digitalWrite(PUMP1_CONTROL_PIN1, HIGH);
      digitalWrite(PUMP1_CONTROL_PIN2, LOW);
      break;
    case 2:
      digitalWrite(PUMP2_ENABLE_PIN, HIGH);
      digitalWrite(PUMP2_CONTROL_PIN1, HIGH);
      digitalWrite(PUMP2_CONTROL_PIN2, LOW);
      break;
    case 3:
      digitalWrite(PUMP3_ENABLE_PIN, HIGH);
      digitalWrite(PUMP3_CONTROL_PIN1, HIGH);
      digitalWrite(PUMP3_CONTROL_PIN2, LOW);
      break;
    default:
      Serial.println("Error: Invalid pump index");
  }
}

// Deactivate all pumps
void deactivateAllPumps() {
  digitalWrite(PUMP1_ENABLE_PIN, LOW);
  digitalWrite(PUMP1_CONTROL_PIN1, LOW);
  digitalWrite(PUMP1_CONTROL_PIN2, LOW);
  digitalWrite(PUMP2_ENABLE_PIN, LOW);
  digitalWrite(PUMP2_CONTROL_PIN1, LOW);
  digitalWrite(PUMP2_CONTROL_PIN2, LOW);
  digitalWrite(PUMP3_ENABLE_PIN, LOW);
  digitalWrite(PUMP3_CONTROL_PIN1, LOW);
  digitalWrite(PUMP3_CONTROL_PIN2, LOW);
}

// Start motor wheels
void startMotorWheels() {
  analogWrite(WHEEL_MOTOR_IN1, motorWheelSpeed);
  analogWrite(WHEEL_MOTOR_IN2, 0);
  analogWrite(WHEEL_MOTOR_IN3, motorWheelSpeed);
  analogWrite(WHEEL_MOTOR_IN4, 0);
}

// Stop motor wheels
void stopMotorWheels() {
  analogWrite(WHEEL_MOTOR_IN1, 0);
  analogWrite(WHEEL_MOTOR_IN2, 0);
  analogWrite(WHEEL_MOTOR_IN3, 0);
  analogWrite(WHEEL_MOTOR_IN4, 0);
}
