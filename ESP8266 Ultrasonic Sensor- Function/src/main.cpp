

#include <Arduino.h>
// Define the Trig and Echo pin:
#define TRIG_PIN D2
#define ECHO_PIN D1

// Define the speed of sound in cm/us:
constexpr float SOUND_SPEED = 0.0343;

// Define maximum distance we want to ping for (in meters):
constexpr float MAX_DISTANCE = 4;


void Ultrasonic () {
  // Clear the TRIG_PIN by setting it LOW:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the TRIG_PIN high for 10 microseconds:
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the time for the echo (duration):
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance:
  float distance = duration * SOUND_SPEED / 2;

  // Print the distance on the Serial Monitor:
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Delay between measurements:
  delay(500);
}



void setup() {
  // Initialize serial communication:
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize the HC-SR04's Trig and Echo pins:
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  Ultrasonic();
}
