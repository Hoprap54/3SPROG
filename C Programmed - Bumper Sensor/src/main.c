/*
 * BumperSensor.c
 *
 * Created: 14/12/2023
 * Author : Ali
 */ 

#define F_CPU 16000000UL
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

#define ADC_PIN 1 // ADC1 corresponds to A1 on Arduino Nano
#define DEBOUNCE_DELAY 50 // 50 milliseconds

// Define ranges for each sensor
#define RANGE_START_1 200
#define RANGE_END_1 300
#define RANGE_START_2 1
#define RANGE_END_2 150
#define RANGE_START_3 430
#define RANGE_END_3 650

void initADC() {
    ADMUX = (1<<REFS0) | (ADC_PIN); // Use AVcc as the reference and select ADC1
    ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // Enable the ADC and set the prescaler to 8
}

uint16_t readADC() {
    ADCSRA |= (1<<ADSC); // Start an ADC conversion
    while (ADCSRA & (1<<ADSC)); // Wait until the conversion is done
    return ADC; // Return the ADC value
}

int main() {
    uart_init();
    io_redirect();
    initADC();

    PORTC |= (1 << 1); // Enable pull-up resistor

    while (1) {
        uint16_t adcValue = readADC();

        if (adcValue >= RANGE_START_1 && adcValue <= RANGE_END_1) {
            printf("Sensor 1 Pressed!\n");
        } else if (adcValue >= RANGE_START_2 && adcValue <= RANGE_END_2) {
            printf("Sensor 2 Pressed!\n");
        } else if (adcValue >= RANGE_START_3 && adcValue <= RANGE_END_3) {
            printf("Sensor 3 Pressed!\n");
        } else {
            printf("No Sensor Pressed.\n");
        }

        _delay_ms(DEBOUNCE_DELAY);
    }

    return 0;
}
