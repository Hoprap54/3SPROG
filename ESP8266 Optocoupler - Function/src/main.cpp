#include <Arduino.h>

const int optocouplerPin = 5; // Change as per your hardware setup
volatile unsigned long lastInterruptTime = 0;
volatile float wheelVelocity = 0.0;
const float wheelCircumference = 1.0; // Set the actual circumference of your wheel in meters
const unsigned long timeout = 2000000; // Timeout in microseconds (e.g., 2 seconds)
const unsigned long debounceTime = 50000; // Debounce time in microseconds (e.g., 50 milliseconds)

void IRAM_ATTR detectRotation() {
    unsigned long currentTime = micros();
    if (currentTime - lastInterruptTime > debounceTime) {
        unsigned long timeDifference = currentTime - lastInterruptTime;
        
        // Calculate velocity (m/s), assuming timeDifference is in microseconds
        if (timeDifference > 0 && timeDifference < timeout) {
            wheelVelocity = (wheelCircumference / timeDifference) * 1000000.0;
        }

        lastInterruptTime = currentTime;
    }
}

void Optocoupler() {
    unsigned long currentTime = micros(); // Get the current time

    noInterrupts();
    unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;
    float currentVelocity = (timeSinceLastInterrupt < timeout) ? wheelVelocity : 0.0;
    interrupts();

    Serial.print("Wheel Velocity: ");
    Serial.print(currentVelocity);
    Serial.println(" m/s");

    delay(1000); // Update rate - can be adjusted
}

void setup() {
    Serial.begin(921600);
    pinMode(optocouplerPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(optocouplerPin), detectRotation, FALLING);
    lastInterruptTime = micros(); // Initialize lastInterruptTime
}

void loop() {
    Optocoupler();
}
