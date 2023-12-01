/*
For pin connection look at global variables.
*/

//Libraries included
#include <Arduino.h>


//Global Variables
int switchPin = D2;  // This is the pin connection - can be changed.
int switchState = 0; // Current state of the switch

void Bumpersensor() {
  switchState = digitalRead(switchPin);

  if (switchState == LOW) {
    Serial.println("Switch Pressed");
  } else {
    Serial.println("Switch Released");
  }

  delay(100);
}


void setup() {
  //sets the switchpin as an input with an internal pull-up resistor. 
  //Meaning the pin is read as HIGH and becomes LOW when pressed.
  pinMode(switchPin, INPUT_PULLUP); // sets the switchpin as an input with an internal pull-up resistor. Meaning the pin is read as HIGH and becomes LOW when pressed.
  Serial.begin(921600); // Initialize serial communication at baud rate of 921600 bits per second.
}

void loop() {
    Bumpersensor();
}

