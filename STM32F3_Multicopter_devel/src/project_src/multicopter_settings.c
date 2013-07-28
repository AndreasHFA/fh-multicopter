#include <multicopter_settings.h>
#include <stm32_configuration.h>
#include <string.h>


void SETTINGS_init(MulticopterSettings *settings)
{
	settings->gyroSensTreshold = 20;
	settings->gyroDir.sensDirection[sensX] = -1;
	settings->gyroDir.sensDirection[sensY] = -1;
	settings->gyroDir.sensDirection[sensZ] = 1;
	settings->gyroDir.sensOrder[sensX] = 0;
	settings->gyroDir.sensOrder[sensY] = 1;
	settings->gyroDir.sensOrder[sensZ] = 2;

	settings->enableDriftCorrection = 1;

	settings->enableACC = 1;
	settings->accDir.sensDirection[sensX] = 1;
	settings->accDir.sensDirection[sensY] = -1;
	settings->accDir.sensDirection[sensZ] = 1;
	settings->accDir.sensOrder[sensX] = 1;
	settings->accDir.sensOrder[sensY] = 0;
	settings->accDir.sensOrder[sensZ] = 2;
	settings->accMaxGOverflow = 1.3;

	settings->enableMAG = 1;
	settings->magLowpassValue = 5;//20;
	settings->magDir.sensDirection[sensX] = 1;
	settings->magDir.sensDirection[sensY] = -1;
	settings->magDir.sensDirection[sensZ] = 1;
	settings->magDir.sensOrder[sensX] = 0;
	settings->magDir.sensOrder[sensY] = 1;
	settings->magDir.sensOrder[sensZ] = 2;

	settings->stopSpeed = 50*PRESCALEVALUE;
	settings->minSpeed = 55*PRESCALEVALUE;
	settings->maxSpeed = 100*PRESCALEVALUE;

#ifdef QUADRO
	settings->PID_rollPitch.PID_kp = 0.6f;
	settings->PID_rollPitch.PID_ki = 0.75f;
	settings->PID_rollPitch.PID_kd = 0.30f;

	settings->PID_yaw.PID_kp = 1.5f;
	settings->PID_yaw.PID_ki = 0.0f;
	settings->PID_yaw.PID_kd = 1.5f;
#endif
#ifdef HEXA
	settings->PID_rollPitch.PID_kp = 0.50f;	//0.65 //0.6l
	settings->PID_rollPitch.PID_ki = 0.20f; //0.12 //0.25
	settings->PID_rollPitch.PID_kd = 0.150f; //0.04//0.30

	settings->PID_yaw.PID_kp = 1.0f;		//2.0
	settings->PID_yaw.PID_ki = 0.0f;		//0.0
	settings->PID_yaw.PID_kd = 0.50f;		//2.0
#endif
}

void CTRLSTATES_init(MulticopterCtrlStates *states)
{
	memset(states, 0, sizeof(MulticopterCtrlStates));
}

