#include <Arduino.h>

#include <Arduino.h>

// Define the Trig and Echo pin:
#define TRIG_PIN D2
#define ECHO_PIN D1

// Define the speed of sound in cm/us:
constexpr float SOUND_SPEED = 0.0343;

// Define maximum distance we want to ping for (in meters):
constexpr float MAX_DISTANCE = 4;

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
























/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-hc-sr04-ultrasonic-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
/*
const int trigPin = 12;
const int echoPin = 14;

//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void setup() {
  Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_VELOCITY/2;
  
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  // Prints the distance on the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);
  
  delay(1000);
}
*/

/*
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(921600);
  Serial.println("Hello world, Hello from the setup");
}

void loop() {
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Hello from the loop");
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}
*/



/*
// Define the connections to the HC-SR04
const int trigPin = D1; // GPIO 5
const int echoPin = D2; // GPIO 4

void setup() {
  // Begin Serial Communication at a baud rate of 9600:
  Serial.begin(9600);

  // Define the Trig and Echo pins:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin. pulseIn returns the duration (length of the pulse) in microseconds:
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance:
  long distance = duration * 0.0343 / 2; // Speed of sound wave divided by 2 (go and back)

  // Output the distance on the Serial Monitor:
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Wait for 50 milliseconds before the next loop:
  delay(50);
}


*/