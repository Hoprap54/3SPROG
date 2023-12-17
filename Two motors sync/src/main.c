#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#define IN1 PB5
#define IN2 PB0
#define IN3 DD7
#define IN4 PB4

void PWM_MotorR(unsigned char);
void PWM_MotorL(unsigned char duty);

int main(void){
    uart_init();
    /* I/O */
    DDRD |= (1<<IN3);
    DDRB |= (1<<IN1) | (1<<IN2) | (1<<IN4);
    
    /* Set a direction in the motors */
    PORTD |= (1<<IN3);
    PORTB |= (1<<IN1) |(1<<IN4);
    PORTB &= ~(1<<IN2);
    
    while (1){
        PWM_MotorR(100);     // Select speed from 1-255. + number + speed
        PWM_MotorL(100);     // Select speed from 1-255. + number + speed
    }
}

/*TIMER0*/
inline void PWM_MotorR(unsigned char duty){ 
    DDRD |= (1 << PB2);     // Declare this ENA pin as output     
    TCCR0A |= (1<<WGM01) | (1<<WGM00);  // Fast PWM configuration
    TCCR0A |= (1<<COM0A1);              // Non inverting output

    TCCR0B |= (1<<CS02) | (1<<CS00);    // 1024 Prescaler
    OCR0A = duty; // Set the value to be compared
}

/*TIMER1*/
void PWM_MotorL(unsigned char duty){
    DDRB |= (1 << PB3);     // Declare this ENA pin as output     

    TCCR1A |= (1<<WGM11) | (1<<WGM10);  // Fast PWM configuration
    TCCR1B |= (1<<WGM13) | (1<<WGM12);
    TCCR1A |= (1<<COM1A1);              // Non inverting output

    TCCR1B |= (1<<CS12) | (1<<CS10);    // 1024 Prescaler
    OCR1A = duty; // Set the value to be compared
}