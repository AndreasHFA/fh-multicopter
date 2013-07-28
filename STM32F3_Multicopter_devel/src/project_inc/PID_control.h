#ifndef PID_CONTROL_H
#define PID_CONTROL_H


typedef struct{

	//Variables for the PID-Control
	float sampling_rate;	/*sample rate in HZ*/
	float y;
	float kp;
	float ki;
	float kd;
	float e;
	float esum;
	float ealt;
}PID_Control;

/*function prototypes*/
void init_PID(float sample_rate, float kp, float ki, float kd, PID_Control *values);
void resetIntegralValues_PID(PID_Control *values);
void calc_PID(float istwert, float sollwert, PID_Control *values);
void calc_PID_YAW(float istwert, float sollwert, PID_Control *values);

#endif /* PID_CONTROL_H */
