#include <PID_control.h>


void init_PID(float sample_rate, float kp, float ki, float kd, PID_Control *values)
{
	values->sampling_rate = sample_rate;
	values->kp = kp;
	values->ki = ki;
	values->kd = kd;
}

void resetIntegralValues_PID(PID_Control *values)
{
	values->ealt = 0;
	values->esum = 0;
	values->y = 0;
}


/*Simple PID-Calcultion from http://www.rn-wissen.de/index.php/Regelungstechnik*/
void calc_PID(float istwert, float sollwert, PID_Control *values)
{	
		values->e = istwert - sollwert;

		//f�r I-Anteil aufsummieren:
		values->esum = values->esum + (values->e);
	
		//PID-Regler	 	
		values->y = (values->kp*values->e) + (values->esum*values->ki*values->sampling_rate) + (values->kd*(values->e - values->ealt)/values->sampling_rate);
	
		//f�r D-Anteil:
		values->ealt = values->e;
}

/*
 * The yaw PID calculation needs a special handling as the input value has
 * an integrating character
 * */
void calc_PID_YAW(float istwert, float sollwert, PID_Control *values)
{
	values->e = istwert - sollwert;

	if(values->e >= 180) values->e -= 360;
	else if (values->e <= -180) values->e += 360;

	/* Keep the e-value in a maintainable range */
	if(values->e > 50) values->e = 50;
	if(values->e < -50) values->e = -50;

	//f�r I-Anteil aufsummieren:
	values->esum = values->esum + (values->e);

	//PID-Regler
	values->y = (values->kp*values->e) + (values->esum*values->ki*values->sampling_rate) + (values->kd*(values->e - values->ealt)/values->sampling_rate);

	//f�r D-Anteil:
	values->ealt = values->e;
}


