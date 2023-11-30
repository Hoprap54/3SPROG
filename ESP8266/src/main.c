#include <stdio.h>

// PID constants
#DEFINE Kp = 0.0;	//when we figure out setpoint we can find these values through calculations or PID simulator
#DEFINE Ki = 0.0;
#DEFINE Kd = 0.0;

// PID variables
double setpoint = 0.0;     // how do we figure that out? Testing?
double output = 0.0;       // The control output
double input = 0.0;        // The control input - current position

double error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double prev_error = 0.0;   // The previous error (for calculating the derivative)

// Function prototypes
double read_position(void);			//need a function to read current position
void control_movement(double output);		//need a function to control the movement
double read_obstacle_sensor(void);		//need a function to read input from obstacle sensor
void rotation(double angle);			//need function to control robot rotation?


int main(void)
	{
	while (1)
		{
       		input = read_position();  			     // Read the current position - store into input
		error = setpoint - input;
		integral += error;
        	derivative = error - prev_error;
		output = Kp*error + Ki*integral + Kd*derivative;     // Calculate the control output
		control_movement(output);   			     // Control the robot's movement using output value
        	prev_error = error;         			     // Update the previous error
    		}
	return 0;
	}



//functions
double read_position(void)
	{
	do_stuff;	//idk what to do here yet
	return 0.0; //needs to return smth?
	}

void control_movement(double output)
	{
	double rotation_angle = ; 	//how do we set a value and it translates into actual degrees of rotation?
   	double safe_distance = ;	//Is this even needed? some value we will set? or probably limit switch being pressed
	double distance = read_obstacle_sensor();
	if (distance < safe_distance)
		{
		rotation(rotation_angle);
		}
	}
