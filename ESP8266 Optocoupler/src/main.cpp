#include <Arduino.h>

const int sensorPin = D5;  // Change this to the pin you are using
volatile int pulseCount = 0;
unsigned long startTime;
float velocity = 0.0;
const int totalMarkers = 3; // Replace N with the actual number of markers

void IRAM_ATTR handleInterrupt() {
    pulseCount++;
}

void setup() {
    Serial.begin(921600);
    pinMode(sensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(sensorPin), handleInterrupt, RISING);

    // Initialize startTime
    startTime = millis();
}

void loop() {
    if (pulseCount >= totalMarkers) {
        // Calculate time difference
        unsigned long endTime = millis();
        unsigned long timeTaken = endTime - startTime; // time taken in milliseconds

        // Calculate velocity (example calculation, adjust as per your requirement)
        velocity = (float)totalMarkers / (timeTaken / 1000.0); // Markers per second

        // Reset for next measurement
        pulseCount = 0;
        startTime = millis();

        // Print the calculated velocity
        Serial.print("Velocity: ");
        Serial.print(velocity);
        Serial.println(" markers/sec");
    }

    // Other code can run here
}






/*
#include <Arduino.h>

volatile int pulseCount = 0;
unsigned long previousMillis = 0;
const int sensorPin = D5;  // replace D5 with your GPIO pin connected to DO
const float wheelRadius = 10.0;  // replace with your wheel's radius in cm

void IR_interrupt() {
    pulseCount++;
}

void setup() {
    Serial.begin(921600);
    pinMode(sensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(sensorPin), IR_interrupt, FALLING);
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 1000) {  // 1-second interval
        previousMillis = currentMillis;

        float rpm = (float)pulseCount * 60.0 / 1000.0;  // convert to rotations per minute
        float wheelCircumference = 2.0 * PI * wheelRadius;  // in cm
        float velocity = rpm * wheelCircumference / 60.0;  // cm per minute

        Serial.print("Pulses: ");
        Serial.print(pulseCount);
        Serial.print(", RPM: ");
        Serial.print(rpm);
        Serial.print(", Velocity: ");
        Serial.print(velocity);
        Serial.println(" cm/min");

        delay(1000);

        pulseCount = 0;  // reset the count for the next interval
    }
}





/*
const int digitalSensorPin = D5;  // replace D5 with the actual GPIO pin for DO
const int analogSensorPin = A0;   // ADC pin on ESP8266 for AD

void setup() {
  Serial.begin(921600);
  pinMode(digitalSensorPin, INPUT);
}

void loop() {
  int digitalValue = digitalRead(digitalSensorPin); // Digital reading
  int analogValue = analogRead(analogSensorPin);    // Analog reading
  
  Serial.print("Digital Value: ");
  Serial.print(digitalValue);
  Serial.print(", Analog Value: ");
  Serial.println(analogValue);

  delay(100);  // Adjust delay as needed for your application
}

















/*
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