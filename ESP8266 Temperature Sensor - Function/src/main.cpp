#include <Arduino.h>

const int analogPin = A0;  // Analog pin where the sensor is connected
const float Vcc = 3.3;     // Supply voltage
const float offsetVoltage = 0.4; // 400 mV offset for MCP9701A at 0 degrees Celsius
const float scale = 0.01953; // Scale factor for MCP9701A (19.53 mV/°C)


void Temperaturesensor(){
  int sensorValue = analogRead(analogPin); // Read the sensor
  float voltage = sensorValue * (Vcc / 1023.0); // Convert to voltage
  float temperatureC = (voltage - offsetVoltage) / scale; // Convert to temperature

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  delay(1000); // Wait for a second before next read
}


void setup() {
  Serial.begin(921600); // Start serial communication at 921600 baud
}

void loop() {
  Temperaturesensor();
}
