/*
 * Optocoupler.c
 *
 * Created: 13/12/2023
 * Author : Beni
 */ 

#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"



#define ADC_PIN_A7 7  // ADC channel for A7

// MCP9701A sensor constants
const float V_0 = 400.0;   // Output voltage at 0ºC in mV
const float TC = 19.5;     // Temperature coefficient in mV/ºC

// Function prototypes
uint16_t adc_read(uint8_t adc_channel);
float calculate_temperature(float voltage); // Changed parameter type to float
void print_temperature_and_voltage(float temperature, float voltage);
void initialize_adc(void);

int main(void) {
    uart_init();           // Initialize UART
    io_redirect();         // Redirect stdout to USART

    initialize_adc();      // Initialize ADC

    while (1) {
        uint16_t adcValue = adc_read(ADC_PIN_A7);
        float voltage = adcValue * (5000.0 / 1024.0); // Convert ADC value to voltage in mV
        float temperature = calculate_temperature(voltage);

        print_temperature_and_voltage(temperature, voltage);

        _delay_ms(1000);    // Delay for 1 second
    }
}

void initialize_adc(void) {
    ADMUX = (1<<REFS0); // Select Vref=AVcc, right adjust result
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, set prescaler to 128
}

uint16_t adc_read(uint8_t adc_channel) {
    ADMUX = (ADMUX & 0xF8) | (adc_channel & 0x07); // Select ADC channel (channel must be between 0 and 7)
    ADCSRA |= (1<<ADSC); // Start conversion
    while (ADCSRA & (1<<ADSC)); // Wait for conversion to complete

    return ADC; // Return 10-bit result
}

float calculate_temperature(float voltage) { // Updated parameter type
    return (voltage - V_0) / TC; // Calculate temperature in Celsius
}

void print_temperature_and_voltage(float temperature, float voltage) {
    printf("Temp: %.2f ºC   V: %.2f mV\n", temperature, voltage);
}
