/*
 * UltrasonicSensor.c
 *
 * Created: 12/12/2023
 * Author : Ali
 */ 

#define F_CPU 16000000UL



// Ultrasonic sensor 1
#define TRIG_PIN PD5 // Assuming you're using pin D5 for Trig - Common trigpin for all ultrasonic sensor
#define ECHO_PIN1 PD2 // Assuming you're using pin D2 for Echo

// Ultrasonic sensor 2
#define ECHO_PIN2 PD3

// Ultrasonic sensor 3
#define ECHO_PIN3 PD4

// Ultrasonic sensor 4
#define ECHO_PIN4 PB4




//Header files
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

// Function to set trigpin to output and echopin to input
void initializeSensor() {
    DDRD |= (1 << TRIG_PIN); // Set Trig pin as output
    DDRD &= ~((1 << ECHO_PIN1) | (1 << ECHO_PIN2) | (1 << ECHO_PIN3)); // Set Echo pins as input
}

long getduration(uint8_t echoPin) {
    long count = 0;
    const long maxCount = 30000; // Adjust as necessary

    // Send trigger pulse
    PORTD |= (1 << TRIG_PIN); // Trigger the sensor - ON
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN); // turned OFF

    // Wait for echo to start
    while(!(PIND & (1 << echoPin))) {
        if (count++ > maxCount) {
            printf("Timeout waiting for echo start on pin %d\n", echoPin);
            return -1; // Timeout, no echo detected
        }
    }

    count = 0; // Reset count for measuring the high time of the echo signal

    // Measure the length of the high signal
    while(PIND & (1 << echoPin)) {
        if (count++ > maxCount) {
            printf("Timeout waiting for echo end on pin %d\n", echoPin);
            return -1; // Timeout, echo too long
        }
    }

    return count;
}



// Calculate distance
double calculateDistance(long duration) {
    return duration * 0.034 / 2; // Speed of sound in cm/us
}



// Main function for ultrasonic sensor
void ultrasonicsensor() {
    while (1) {
        // Sensor 1
        long duration1 = getduration(ECHO_PIN1);
        double distance1 = calculateDistance(duration1);
        printf("Sensor 1 Duration: %ld, Distance: %.2f cm\n", duration1, distance1);

        _delay_ms(500); // Delay between sensor readings

        // Sensor 2
        long duration2 = getduration(ECHO_PIN2);
        double distance2 = calculateDistance(duration2);
        printf("Sensor 2 Duration: %ld, Distance: %.2f cm\n", duration2, distance2);

        _delay_ms(500); // Delay before the next set of measurements

        // Sensor 3
        long duration3 = getduration(ECHO_PIN3);
        double distance3 = calculateDistance(duration3);
        printf("Sensor 3 Duration: %ld, Distance: %.2f cm\n", duration3, distance3);

        _delay_ms(500); // Delay before the next set of measurements

    }
}



int main() {
    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
    initializeSensor();
    ultrasonicsensor();
    return 0;
}



