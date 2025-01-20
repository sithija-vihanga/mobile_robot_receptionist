#include <Arduino.h>

// Motor Pins
#define RIGHT_VR 13  // PWM pin for right motor speed control
#define RIGHT_ZF 12  // Direction pin for right motor
#define RIGHT_EL 11  // Enable pin for right motor

#define LEFT_VR 10   // PWM pin for left motor speed control
#define LEFT_ZF 9    // Direction pin for left motor
#define LEFT_EL 8    // Enable pin for left motor

// Function to control motor direction and speed
void controlMotor(float leftMotor, float rightMotor) {
  // Map float values to 0-255 for PWM
  int leftSpeed = map(abs(leftMotor) * 10, 0, 100, 0, 255);
  int rightSpeed = map(abs(rightMotor) * 10, 0, 100, 0, 255);

  // Set left motor direction
  if (leftMotor >= 0) {
    digitalWrite(LEFT_ZF, HIGH); // Forward
  } else {
    digitalWrite(LEFT_ZF, LOW); // Reverse
  }

  // Set right motor direction
  if (rightMotor >= 0) {
    digitalWrite(RIGHT_ZF, LOW); // Forward
  } else {
    digitalWrite(RIGHT_ZF, HIGH); // Reverse
  }

  // Adjust speed using PWM
  analogWrite(LEFT_VR, leftSpeed);
  analogWrite(RIGHT_VR, rightSpeed);

  // Enable motors
  digitalWrite(LEFT_EL, HIGH);
  digitalWrite(RIGHT_EL, HIGH);
}

void disableMotors() {
  // Disable motors
  digitalWrite(LEFT_EL, LOW);
  digitalWrite(RIGHT_EL, LOW);
  analogWrite(LEFT_VR, 0);
  analogWrite(RIGHT_VR, 0);
}

void setup() {
  // Set motor pins as outputs
  pinMode(RIGHT_VR, OUTPUT);
  pinMode(RIGHT_ZF, OUTPUT);
  pinMode(RIGHT_EL, OUTPUT);

  pinMode(LEFT_VR, OUTPUT);
  pinMode(LEFT_ZF, OUTPUT);
  pinMode(LEFT_EL, OUTPUT);

  // Begin Serial communication
  Serial.begin(115200);
  Serial.println("Motor Control via Serial Input Starting...");
  delay(5000);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Parse the input format "leftmotor,rightmotor"
    int commaIndex = input.indexOf(',');

    if (commaIndex > 0) {
      float leftMotor = input.substring(0, commaIndex).toFloat();
      float rightMotor = input.substring(commaIndex + 1).toFloat();

      // Control motors based on parsed input
      controlMotor(leftMotor, rightMotor);

      Serial.write("Left Motor: ");
      Serial.write(String(leftMotor).c_str());
      Serial.write(", Right Motor: ");
      Serial.write(String(rightMotor).c_str());
      Serial.write("\n");
      // Serial.write(String("9,9,9,9,").c_str());
      // Serial.write("\n");


      // Disable motors after operation
      //delay(1000); // Optional delay for motor operation
      //disableMotors();
    } else {
      Serial.println("Invalid input format. Use 'leftmotor,rightmotor'.");
    }
  }
}
