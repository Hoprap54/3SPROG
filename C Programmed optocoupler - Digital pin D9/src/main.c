#define F_CPU 16000000UL
#define OPTO_PIN PB1  // D9 corresponds to pin PB1
#define DEBOUNCE_TIME 10 // 10 milliseconds debounce time
#define WHEEL_CIRCUMFERENCE 0.06754 // Example in meters
#define SEGMENTS_PER_REVOLUTION 12

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

uint32_t millis() {
    // Function to return the number of milliseconds since the program started
    static uint32_t millis_counter = 0;
    millis_counter += 10;
    _delay_ms(10);
    return millis_counter;
}

int main() {
    uart_init();
    io_redirect();

    DDRB &= ~(1 << OPTO_PIN); // Set OPTO_PIN as input with pull-up resistor
    PORTB |= (1 << OPTO_PIN);

    uint16_t pulseCount = 0;
    uint32_t lastMillis = 0, currentMillis;
    uint8_t lastState = 1; // Assuming pull-up, initial state is HIGH
   // uint32_t lastDebounceTime = 0;

while (1) {
    currentMillis = millis();
    uint8_t currentState = (PINB >> OPTO_PIN) & 1;

    //printf("Current State: %d\n", currentState);

    // Falling edge detection
    if (lastState == 1 && currentState == 0) {
        _delay_ms(DEBOUNCE_TIME); // Debounce
        currentState = (PINB >> OPTO_PIN) & 1; // Re-read the state after debounce time
        if (currentState == 0) { // Confirm the pulse
            pulseCount++;
            //printf("Pulse detected\n");
        }
    }

    lastState = currentState; // Update lastState at the end of the loop

    if (currentMillis - lastMillis >= 1000) {
        // Debugging: Print the pulse count
      //  printf("Pulse Count: %u\n", pulseCount);

        float revolutions = (float)pulseCount / SEGMENTS_PER_REVOLUTION;
        float distance = revolutions * WHEEL_CIRCUMFERENCE;
        float speed = distance / ((currentMillis - lastMillis) / 1000.0f); // Speed in meters per second

        printf("Speed: %f m/s\n", speed);

        pulseCount = 0;
        lastMillis = currentMillis;
    }
}


    return 0;
}
