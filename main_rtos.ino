#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
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

// Task handles
TaskHandle_t distanceTask, temperatureTask, motorControlTask;

// Global variables for conditions
bool fireDetected = false;
bool closeObjectDetected = false;
bool motorsRunning = false;

// Timing variables
unsigned long distanceLatency = 0;
unsigned long temperatureLatency = 0;
unsigned long motorControlLatency = 0;

// Task to read distance from TF-Luna LIDAR
void readDistanceTask(void *pvParameters) {
    unsigned long startTime, endTime;
    while (true) {
        startTime = micros(); // Start timing in µs
        if (tflI2C.getData(tfDist, tfAddr)) {
            closeObjectDetected = (tfDist <= distanceThreshold);
        } else {
            closeObjectDetected = false;
        }
        endTime = micros(); // End timing in µs
        distanceLatency = endTime - startTime;
        vTaskDelay(pdMS_TO_TICKS(200)); // Run every 200 ms
    }
}

// Task to read temperature from AMG8833
void readTemperatureTask(void *pvParameters) {
    unsigned long startTime, endTime;
    float pixels[64];

    while (true) {
        startTime = micros(); // Start timing in µs
        amg.readPixels(pixels);
        fireDetected = false;
        for (int i = 0; i < 64; i++) {
            if (pixels[i] >= temperatureThreshold) {
                fireDetected = true;
                break;
            }
        }
        endTime = micros(); // End timing in µs
        temperatureLatency = endTime - startTime;
        vTaskDelay(pdMS_TO_TICKS(500)); // Run every 500 ms
    }
}

// Task to control motors based on sensor readings
void motorControlTaskFunc(void *pvParameters) {
    unsigned long startTime, endTime;

    // Initialize motors
    pinMode(dirA, OUTPUT);
    pinMode(pwmA, OUTPUT);
    pinMode(brakeA, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(brakeB, OUTPUT);

    while (true) {
        startTime = micros(); // Start timing in µs
        if (fireDetected && !closeObjectDetected) {
            if (!motorsRunning) {
                runMotors();
                motorsRunning = true;
            }
        } else {
            if (motorsRunning) {
                stopMotors();
                motorsRunning = false;
            }
        }
        endTime = micros(); // End timing in µs
        motorControlLatency = endTime - startTime;

        // Print latency only if all task latencies are greater than zero
        if (distanceLatency > 0 && temperatureLatency > 0 && motorControlLatency > 0) {
            unsigned long totalLatency = distanceLatency + temperatureLatency + motorControlLatency;
            Serial.println("Task Latency Metrics:");
            Serial.print("Distance Task Latency: ");
            Serial.print(distanceLatency);
            Serial.println(" µs");
            Serial.print("Temperature Task Latency: ");
            Serial.print(temperatureLatency);
            Serial.println(" µs");
            Serial.print("Motor Control Task Latency: ");
            Serial.print(motorControlLatency);
            Serial.println(" µs");
            Serial.print("Total Task Latency: ");
            Serial.print(totalLatency);
            Serial.println(" µs");
            Serial.println();
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100 ms
    }
}

// Function to run motors
void runMotors() {
    Serial.println("Running motors...");
    analogWrite(pwmA, 200);    // Motor A speed
    digitalWrite(dirA, HIGH);  // Motor A direction
    digitalWrite(brakeA, LOW); // Disable brake A

    analogWrite(pwmB, 200);    // Motor B speed
    digitalWrite(dirB, HIGH);  // Motor B direction
    digitalWrite(brakeB, LOW); // Disable brake B
}

// Function to stop motors
void stopMotors() {
    Serial.println("Stopping motors...");
    digitalWrite(brakeA, HIGH); // Enable brake A
    analogWrite(pwmA, 0);       // Stop Motor A

    digitalWrite(brakeB, HIGH); // Enable brake B
    analogWrite(pwmB, 0);       // Stop Motor B
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize AMG8833
    if (!amg.begin()) {
        Serial.println("AMG8833 not found!");
        while (1);
    }

    // Create tasks
    xTaskCreate(readDistanceTask, "Distance Task", 256, NULL, 1, &distanceTask);
    xTaskCreate(readTemperatureTask, "Temperature Task", 256, NULL, 1, &temperatureTask);
    xTaskCreate(motorControlTaskFunc, "Motor Control Task", 256, NULL, 1, &motorControlTask);

    Serial.println("Starting FreeRTOS Scheduler...");
    vTaskStartScheduler(); // Start the scheduler
}

void loop() {
    // Empty as FreeRTOS manages the tasks
}
