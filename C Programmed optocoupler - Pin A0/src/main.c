/*
 * Optocoupler.c
 *
 * Created: 13/12/2023
 * Author : Ali
 */ 

#define F_CPU 16000000UL
#define OPTO_PIN 0  
#define DEBOUNCE_TIME 10 
#define WHEEL_CIRCUMFERENCE 0.314 
#define SEGMENTS_PER_REVOLUTION 5

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"



void delayMilliseconds(int ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

void initOptoPin() {
    DDRC &= ~(1 << OPTO_PIN); 
    PORTC |= (1 << OPTO_PIN); 
}

int readOptoPinWithDebounce() {
    int stableState = (PINC & (1 << OPTO_PIN)) ? 1 : 0;
    delayMilliseconds(DEBOUNCE_TIME);
    int currentState = (PINC & (1 << OPTO_PIN)) ? 1 : 0;
    return (currentState == stableState) ? currentState : -1;
}

void calculateSpeed(uint16_t pulseCount, uint32_t timeElapsed) {
    float revolutions = (float)pulseCount / SEGMENTS_PER_REVOLUTION;
    float distance = revolutions * WHEEL_CIRCUMFERENCE;
    float speed = distance / (timeElapsed / 1000.0); 
    printf("Pulse Count: %u\n", pulseCount);
    printf("Speed: %f m/s\n", speed);
}

void OptocouplerA0() {
    delayMilliseconds(500); 
    uint16_t pulseCount = 0;
    uint32_t timeElapsed = 0;
    int lastOptoState = 1;

    while (1) {
        int currentOptoState = readOptoPinWithDebounce();
        if(currentOptoState == -1) continue; // Ignore unstable state

        if (lastOptoState == 1 && currentOptoState == 0) {
            pulseCount++;
        }
        lastOptoState = currentOptoState;

        delayMilliseconds(100); 
        timeElapsed += 100;

        if (timeElapsed >= 1000) {
            calculateSpeed(pulseCount, timeElapsed);
            pulseCount = 0;
            timeElapsed = 0;
        }
    }
}

int main() {
    uart_init();
    io_redirect();
    initOptoPin();
    OptocouplerA0();
    return 0;
}









/*

#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"


#define OPTO_PIN 0  // Use A0 as digital input
#define DEBOUNCE_TIME 10 // 10 milliseconds debounce time

#define WHEEL_CIRCUMFERENCE 0.314 // Example in meters
#define SEGMENTS_PER_REVOLUTION 5

void delayMilliseconds(int ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

void initOptoPin() {
    DDRC &= ~(1 << OPTO_PIN); // Set A0 as input
    PORTC |= (1 << OPTO_PIN); // Optionally enable internal pull-up resistor
}

OptocouplerA0() {
    
    // Add an initial delay to stabilize the setup
    delayMilliseconds(500); // 500 ms initial delay

    uint16_t pulseCount = 0;
    uint32_t timeElapsed = 0;
    int lastOptoState = 1; // Initialize to 1 assuming no pulse initially

    while (1) {
        int currentOptoState = (PINC & (1 << OPTO_PIN)) ? 1 : 0;
        
        // printf("Opto State: %d\n", currentOptoState);

        // Count pulse on falling edge (transition from 1 to 0)
        if (lastOptoState == 1 && currentOptoState == 0) {
            pulseCount++;
            // printf("Pulse detected\n");
            delayMilliseconds(50); // Increase debounce time
        }
        lastOptoState = currentOptoState;

        delayMilliseconds(100); // Increment time in 100ms steps
        timeElapsed += 100;

        if (timeElapsed >= 1000) { // Every second
            // Calculate speed
            float revolutions = (float)pulseCount / SEGMENTS_PER_REVOLUTION;
            float distance = revolutions * WHEEL_CIRCUMFERENCE;
            float speed = distance / (timeElapsed / 1000.0); // Speed in meters per second

            printf("Pulse Count: %u\n", pulseCount);
            printf("Speed: %f m/s\n", speed);

            pulseCount = 0; // Reset pulse count for the next interval
            timeElapsed = 0;
        }
    }
}

int main() {
    uart_init();
    io_redirect();

    initOptoPin();
    OptocouplerA0();

    return 0;
}
*/