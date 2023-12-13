#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "usart.h"
#include "lcd.h"
#include "i2cmaster.h"

#define ADC_PIN 0 // ADC channel
uint16_t adc_result;
int v_in;

uint16_t adc_read(uint8_t adc_channel); 

void display(int);

int main(void){
    uart_init();
    i2c_init();
    LCD_init();

    DDRB |= (1 << DDB5); // D13 as output for LED

    ADMUX = (1<<REFS0);  // Select Vref = AVc
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); // Set prescaler to 128 and turn on the ADC module
    
    while(1){
        for(int i = 0; i <= 3; i++){
            adc_result = adc_read(ADC_PIN);
          
            display(i);
            _delay_ms(100);
        }
    }
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xF0;        // Clear any previously used channel, but keep internal reference
    ADMUX |= adc_channel; // Set the desired channel 

    // Start a conversion
    ADCSRA |= (1 << ADSC);
    // Wait for the conversion to complete
    while ( (ADCSRA & (1 << ADSC)) );   // Will remain 1 as long as the conversion is in progress
    // now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return ADC;
}

void display(int index){
    LCD_set_cursor(0, index);
    printf("%d mV %d", v_in, adc_result);
}

void adda_converter(){
    v_in = (adc_result * (4770 / 1024.0)); // Multiply it with the ratio otherwise the int is overloaded
}