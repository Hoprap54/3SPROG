/*
 * Optocoupler.c
 *
 * Created: 13/12/2023
 * Author : Ali
 */ 

#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

// Define the optocoupler input pin (analog)
#define OPTO_PIN 7  // Using ADC7 (A7 on Arduino Nano)
#define SEGMENTS_PER_REVOLUTION 5
#define WHEEL_CIRCUMFERENCE 0.314

volatile uint16_t pulseCount = 0;

void setup() {

    // Set the prescaler and enable ADC
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}


int analogRead(uint8_t pin) {
    // Select AVCC as the reference and ADC7 as the input channel
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);
    // Start the ADC conversion
    ADCSRA |= (1 << ADEN) | (1 << ADSC);
    // Wait for the conversion to finish
    while (ADCSRA & (1 << ADSC));
    // Return the ADC value
    return ADC;
}



void optocoupler() {
    int optoState = 0;
    int lastOptoState = 0;
    int analogThreshold = 600;  // Threshold based on observed values
    const uint32_t delayTime = 1000;  // Delay time in milliseconds

    while (1) {
        pulseCount = 0;  // Reset pulse count at the start of each cycle

        // Perform reading and counting for approximately one second
        for (uint32_t i = 0; i < delayTime; i++) {
            int analogValue = analogRead(OPTO_PIN);
            optoState = (analogValue > analogThreshold) ? 1 : 0;

            if (optoState && !lastOptoState) {
                pulseCount++;
            }
            lastOptoState = optoState;

            // This delay will roughly accumulate to 1 second with some error
            _delay_ms(1);  
        }

        // After approximately one second, calculate and print the pulse count and speed
        float revolutions = (float)pulseCount / SEGMENTS_PER_REVOLUTION;
        float distance = revolutions * WHEEL_CIRCUMFERENCE;
        float speed = distance;  // Speed in meters/second

        printf("Pulse Count: %u\n", pulseCount);
        printf("Speed: %f m/s\n", speed);
    }    
}


int main() {
    uart_init();
    io_redirect();
    setup();
    optocoupler();

    return 0;
}
