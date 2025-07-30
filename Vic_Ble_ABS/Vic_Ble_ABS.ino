/*
 * ESP32 PCA9685 Continuous Rotation Servo Control via USB Serial & Bluetooth
 * Description: This script controls a continuous rotation servo using an ESP32
 * and a PCA9685 driver. It accepts serial commands from the Arduino IDE's
 * Serial Monitor (USB) or a Bluetooth Serial Terminal app on a smartphone.
 *
 * This code is intended for a CONTINUOUS ROTATION SERVO.
 *
 * Connections:
 * ESP32 Pin GPIO 22 -> PCA9685 SDA
 * ESP32 Pin GPIO 21 -> PCA9685 SCL
 * ESP32 5V         -> PCA9685 VCC
 * ESP32 GND        -> PCA9685 GND
 * External 5-6V    -> PCA9685 V+ (for powering the servo)
 */

// --- LIBRARIES ---
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h" // Library for Bluetooth functionality
String device_name = "ESP32-BT-Slave";

// --- OBJECTS ---
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
BluetoothSerial SerialBT; // Object for Bluetooth Serial

// --- CONSTANTS ---
// PWM values for a continuous rotation servo.
// You may need to fine-tune SERVO_STOP for your specific motor.
#define CLOCKWISE_SPEED         600
#define COUNTER_CLOCKWISE_SPEED 100
#define SERVO_STOP              0

// Define the PCA9685 channel for the servo
#define SERVO_CHANNEL 0

// --- HELPER FUNCTION FOR HANDLING INPUT ---
// This function contains the motor control logic and is called
// when a command is received from either USB or Bluetooth.
void handleInput(char input) {
  char command = tolower(input); // Convert input to lowercase

  if (command == 'a') {
    pca9685.setPWM(SERVO_CHANNEL, 0, CLOCKWISE_SPEED);
    // Print feedback to both terminals
    Serial.println("Rotating Clockwise...");
    SerialBT.println("Rotating Clockwise...");
  }
  else if (command == 'b') {
    pca9685.setPWM(SERVO_CHANNEL, 0, COUNTER_CLOCKWISE_SPEED);
    Serial.println("Rotating Counter-Clockwise...");
    SerialBT.println("Rotating Counter-Clockwise...");
  }
  else if (command == 's') {
    pca9685.setPWM(SERVO_CHANNEL, 0, SERVO_STOP);
    Serial.println("Motor Stopped.");
    SerialBT.println("Motor Stopped.");
  }
}


void setup() {
  // Start the standard USB Serial Monitor
  Serial.begin(115200);

  // --- BLUETOOTH INITIALIZATION ---
  // Start the Bluetooth serial port and give the device a name
  if (!SerialBT.begin("ESP32_BT_Slave")) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized. Ready to pair.");
  }

  // --- PCA9685 INITIALIZATION ---
  pca9685.begin();
  pca9685.setPWMFreq(50); // Standard 50Hz for servos

  // --- PRINT INSTRUCTIONS ---
  const char* instructions =
    "---------------------------------\n"
    "Send 'A' to rotate clockwise.\n"
    "Send 'B' to rotate counter-clockwise.\n"
    "Send 'S' to stop the motor.\n"
    "---------------------------------";
  Serial.println(instructions);
  SerialBT.println(instructions);


  // --- SET INITIAL STATE TO STOPPED ---
  pca9685.setPWM(SERVO_CHANNEL, 0, SERVO_STOP);
  Serial.println("Motor is stopped.");
  SerialBT.println("Motor is stopped.");
}

void loop() {
  // Check for input from the USB Serial Monitor
  if (Serial.available()) {
    char command = Serial.read();
    handleInput(command);
  }

  // Check for input from the Bluetooth Serial Terminal
  if (SerialBT.available()) {
    char command = SerialBT.read();
    handleInput(command);
  }
}
