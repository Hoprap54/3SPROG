#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>
#include "usart.h"

typedef struct
{   
    /* PID gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low pass filter time constant (in seconds):
    It is a measure of the speed of response of the filter. A smaller time constant means a faster response, but more noise will pass through.
    The time constant (tau - τ) of a first-order low-pass filter is typically calculated using the cut-off frequency (fc) of the filter. The formula is:
    τ = 1/2*π*fc.
    The cut-off frequency is the frequency at which the output power falls to half its maximum value, also known as the -3dB point */
    float tau;

    /* Output limits: useful for preventing the controller trying to drive the motors beyond their physical limitations.
    The limits for constraining the controller output are typically determined based on the physical limits of the actuator in your control system1.
    For example, if you’re controlling a motor speed that accepts a PWM signal ranging from 0 to 255, then your controller output should be constrained
    within this range */
    float min_lim;
    float max_lim;
	
    /* Integrator limits */
    float int_min_lim;
    float int_max_lim;

    /* Sample time (in seconds). Basically how often we want the controller to read the parameter being controlled (speed in our case). */
    float T = 1; /*A good rule of thumb is to make the controller have a 10 times higher bandwidth, than the system it controls */

    /* Controller "memory" */
    float integrator;
    float prev_Error;			/* Required for integrator */
    float differentiator;
    float prev_Measurement;		/* Required for differentiator */

    /* Controller output */
    float output;

} PID_vars;

PID_vars PID_addr; /* asigning an address to the structure */


void PID_reset(&PID_addr); /* function to reset the controller */
float PID_update(&PID_addr);


int main(void) 
{ 
    uart_init();
    PID_reset(&PID_addr);

    while (1)
    {
       PID_update(&PID_addr);
    }
}   


void PID_reset(&PID_addr)     /* function to reset the controller */
{
    /* Controller "memory" */
    PID_addr->integrator = 0.0f;
    PID_addr->prev_Error = 0.0f;			/* Required for integrator */
    
    PID_addr->differentiator = 0.0f;
    PID_addr->prev_Measurement = 0.0f;		/* Required for differentiator */

    /* Controller output */
    PID_addr->output = 0.0f;
}



float PID_update(&PID_addr, float setpoint, float measurement)
{
    /* Pass the value returned from the optocoupler function, to the measurement variable */
    measurement = optocoupler_function(); /* FUNCTION MISSING - HAVE TO GO CHECK ON GITHUB */

   /* Error signal */
    float error = setpoint - measurement; /* setpoint is the speed we want to maintain, Measurement id basically the input from the optocoupler */

	/* Proportional */
    float proportional = PID_addr->Kp * error;   /* As we know the proportional term is the proportional gain (Kp) multiplied by the error */

	/* Integral: Depends on previous errors (summed up) AND previous integral value.
    WHAT IS INTEGRATOR WINDUP: In a PID controller, the integral term accumulates the error over time, aiming to eliminate the steady-state error. However, when the controller output
    reaches its saturation limit, the integral term can continue to accumulate error, leading to a large unwanted output when the controller comes out of saturation. This can cause overshoot
    and instability in the control system (this is called integrator wind-up). We will use anti-wind-up (essencialy we will clamp the integrator, to prevent it from reaching excessively large
    values, when the control signal is already at its maximum possible value for extended periods of time)*/
     PID_addr->integrator = PID_addr->integrator + 0.5f * PID_addr->Ki * PID_addr->T * (error + PID_addr->prev_Error); /* The integrator is essentialy the previous value of the integrator + the integrator gain (Ki) times the sampling time (T) times the previous errors */

    /* Find the integrator limits */
    if (PID_addr->max_lim > proportional)
    {
       PID_addr->int_max_lim = PID_addr->max_lim - proportional;
    }
    else
    {
       PID_addr->int_max_lim = 0.0f; 
    }

    /* Anti-wind-up via integrator clamping. This is figuring out the limits of the integrator as to not saturate the output and make it overdrive */
    if (PID_addr->integrator > PID_addr->int_max_lim)
    {
        PID_addr->integrator = PID_addr->int_max_lim;
    }
    else if (PID_addr->integrator < PID_addr->int_min_lim)
    {
        PID_addr->integrator = PID_addr->int_min_lim;
    }

    /* Derivative - with a low pass filter (also called band-limited differentiator): The derivative tends to strongly amplify High Frequency noice
    (the derivative is an infinitely increasing slope). To combat this we low pass filter the signal.
    Derivative on measurement: The derivative produces a large kick during the setpoint change (if we have an impulse change or step change in the setpoint,
    the derivative will produce a kick). To avoid that we use derivative-on-measurement.TLDR: Instead of taking the derivative of the error signal, we take the
    derivative of the measurement fed back by the sensor. The end result is the same but without the kick */	
    PID_addr->differentiator = -(2.0f * PID_addr->Kd * (measurement - PID_addr->prev_Measurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                             + (2.0f * PID_addr->tau - PID_addr->T) * PID_addr->differentiator)
                             / (2.0f * PID_addr->tau + PID_addr->T);


    /* Compute output and apply limits */
    PID_addr->output = proportional + PID_addr->integrator + PID_addr->differentiator;

    /* The output needs to be constrained. Just as a precaution, in case the controller tries to drive the motors beyond their capabilities */
    if (PID_addr->output > PID_addr->max_lim) {

        PID_addr->output = PID_addr->max_lim;

    } else if (PID_addr->output < PID_addr->min_lim) {

        PID_addr->output = PID_addr->min_lim;

    }

	/* Store error and measurement for later use */
    PID_addr->prev_Error       = error;
    PID_addr->prev_Measurement = measurement;

	/* Return controller output */
    return PID_addr->output;

}



