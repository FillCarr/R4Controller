#include <Arduino.h>
#include <Adafruit_AMG88xx.h>
#include <Wire.h>
#include <TFLI2C.h>

// AMG8833 Thermal Imager setup
Adafruit_AMG88xx amg;
const float temperatureThreshold = 45.0; // Temperature threshold in Celsius

// TF-Luna LIDAR setup
TFLI2C tflI2C;
int16_t tfDist;
int16_t tfAddr = TFL_DEF_ADR; // Default I2C address for TF-Luna
const int distanceThreshold = 20; // Distance threshold in cm

// Motor A pins
const int dirA = 12;
const int pwmA = 3;
const int brakeA = 9;

// Motor B pins
const int dirB = 13;
const int pwmB = 11;
const int brakeB = 8;

// Global variables for conditions
bool fireDetected = false;
bool closeObjectDetected = false;
bool motorsRunning = false;

// Timing variables
unsigned long previousDistanceCheck = 0;
unsigned long previousTemperatureCheck = 0;
unsigned long previousMotorCheck = 0;
const unsigned long distanceInterval = 200; // Milliseconds between distance checks
const unsigned long temperatureInterval = 500; // Milliseconds between temperature checks
const unsigned long motorInterval = 100; // Milliseconds between motor control checks

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize AMG8833
    if (!amg.begin()) {
        Serial.println("AMG8833 not found!");
        while (1);
    }

    // Initialize motors
    pinMode(dirA, OUTPUT);
    pinMode(pwmA, OUTPUT);
    pinMode(brakeA, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(brakeB, OUTPUT);
}

void loop() {
    unsigned long currentMillis = millis();

    // Distance check every distanceInterval
    if (currentMillis - previousDistanceCheck >= distanceInterval) {
        previousDistanceCheck = currentMillis;
        readDistance();
    }

    // Temperature check every temperatureInterval
    if (currentMillis - previousTemperatureCheck >= temperatureInterval) {
        previousTemperatureCheck = currentMillis;
        readTemperature();
    }

    // Motor control check every motorInterval
    if (currentMillis - previousMotorCheck >= motorInterval) {
        previousMotorCheck = currentMillis;
        controlMotors();
    }
}

void readDistance() {
    if (tflI2C.getData(tfDist, tfAddr)) {
        Serial.print("Distance from TF-Luna: ");
        Serial.print(tfDist);
        Serial.println(" cm");

        closeObjectDetected = (tfDist <= distanceThreshold);
        if (closeObjectDetected) {
            Serial.println("Object detected within range!");
        }
    } else {
        Serial.println("Failed to read from TF-Luna.");
        closeObjectDetected = false;
    }
}

void readTemperature() {
    float pixels[64];
    amg.readPixels(pixels);
    fireDetected = false;

    for (int i = 0; i < 64; i++) {
        if (pixels[i] >= temperatureThreshold) {
            fireDetected = true;
            Serial.println("High temperature detected! Potential fire source.");
            break;
        }
    }
}

void controlMotors() {
    if ((fireDetected || closeObjectDetected) && !motorsRunning) {
        runMotors();
        motorsRunning = true;
    } else if (!(fireDetected || closeObjectDetected) && motorsRunning) {
        stopMotors();
        motorsRunning = false;
    }
}

// Function to run motors
void runMotors() {
    Serial.println("Running motors...");

    // Motor A
    analogWrite(pwmA, 200);    // Set speed
    digitalWrite(dirA, HIGH);   // Set direction
    digitalWrite(brakeA, LOW);  // Disable brake

    // Motor B
    analogWrite(pwmB, 200);     // Set speed
    digitalWrite(dirB, HIGH);   // Set direction
    digitalWrite(brakeB, LOW);  // Disable brake
}

// Function to stop motors
void stopMotors() {
    Serial.println("Stopping motors...");

    // Motor A
    digitalWrite(brakeA, HIGH); // Enable brake
    analogWrite(pwmA, 0);       // Set speed to 0

    // Motor B
    digitalWrite(brakeB, HIGH); // Enable brake
    analogWrite(pwmB, 0);       // Set speed to 0
}
