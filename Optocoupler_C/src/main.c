/*
 * HelloWorld.c
 *
 * Created: 12/9/2022 10:43:27 PM
 * Author : Alin
 */ 


 
#define F_CPU 16000000UL
#define BAUD 9600
#define CIRCUMFERENCE 0.067544242

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "usart.h"

#include <avr/interrupt.h>

int speed=0;
int seconds = 0;
int counter = 0;
int timer =0;


// ENA -> D4

//void PWM_Motor(unsigned char);

int main(void){
    uart_init();
    io_redirect();

    sei(); //Enable global interrupt
    DDRC &= (1<<PC0); //Setting pin for A0
    TCCR1A = 0x00;
    TCCR1B = (1<<ICNC1);


    ADMUX = ADMUX | 0x40;//ADC0 single ended input on PortC0
    ADCSRB = ADCSRB & (0xF8);//Free running mode
    ADCSRA = ADCSRA | 0xE7; //Enable, Start conversion, slow input clock

    while (1){
        //PWM_Motor(100);     // Select speed from 1-255. + number + speed
        printf("Speed: %d",speed);
    }   
}

ISR (TIMER1_CAPT_vect){
    timer = ICR1+65335*counter; //Updating timer value 
    TCNT1 = 0;                  //Resetting the timer to zero
    counter = 0;                //Resetting timer overflow

    seconds = ((double)timer*1000)/15625000; //Time calculations (Seconds)
    speed = CIRCUMFERENCE/seconds; 

}

/*
inline void PWM_Motor(unsigned char duty){ 
    DDRD |= (1 << DD6);     // Declare this ENA pin as output     
    TCCR0A |= (1<<WGM01) | (1<<WGM00);  // Fast PWM configuration
    TCCR0A |= (1<<COM0A1);              // Non inverting output

    TCCR0B |= (1<<CS02) | (1<<CS00);    // 1024 Prescaler
    OCR0A = duty; // Set the value to be compared
}
*/
