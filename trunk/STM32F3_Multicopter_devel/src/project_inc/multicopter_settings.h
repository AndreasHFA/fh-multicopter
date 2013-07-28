/*
 * multicopter_settings.h
 *
 *  Created on: 04.09.2012
 *      Author: Flasch Franz
 */

#ifndef MULTICOPTER_SETTINGS_H_
#define MULTICOPTER_SETTINGS_H_

#include <stm32f30x.h>
#include <IMU_algorithm.h>


/*
 * Define driver-libraries here:
 */
//#define ITG3200
//#include <ITG3200_Lib.h>
//#define L3G4200D
//#include <L3G4200D_Lib.h>

//#define ADXL345
//#include <ADXL345_Lib.h>

//#define HMC5883L
//#include <HMC5883L_Lib.h>

#define HEXA
//#define QUADRO

#define MAX_GAS 200
#define MAX_ROLL 35
#define MAX_NICK 35
#define MAX_YAW 35

#define sensX 0
#define sensY 1
#define sensZ 2

#define STDEV_THRESHOLD 80

/* USEFUL FLAGS */
#define ARMED_FLAG	0x01 	 /* FLAG set by the RemoteControl*/
#define IMU_READY_FLAG	0x02 /* FLAG set by the IMU Gyro Test*/

#define COPTER_STATUS_ERROR -1
#define COPTER_STATUS_OK 0

typedef struct _sensorDir_
{
	uint8_t sensOrder[3];
	int8_t sensDirection[3];
}sensorDir;

typedef struct _PID_settings_
{
	float PID_kp;
	float PID_ki;
	float PID_kd;
}PID_settings;


typedef struct _MulticopterSettings_
{
	uint16_t gyroSensTreshold;
	uint16_t gyroLowpassValue;
	sensorDir gyroDir;

	uint8_t enableDriftCorrection;

	uint8_t enableACC;
	sensorDir accDir;
	float accMaxGOverflow;

	uint8_t enableMAG;
	sensorDir magDir;
	uint16_t magLowpassValue;

	uint16_t stopSpeed;
	uint16_t minSpeed;
	uint16_t maxSpeed;

	PID_settings PID_rollPitch;
	PID_settings PID_yaw;
}MulticopterSettings;

typedef struct _MulticopterCtrlStates_
{
	int16_t gas;
	int16_t roll;
	int16_t nick;
	int16_t yaw;

	/*
	 * Global Values for the Motor Control
	 *TODO: copterStatus beschreiben
	 */
	uint8_t copterStatus;
	uint8_t watchDogTimVal;

}MulticopterCtrlStates;

void SETTINGS_init(MulticopterSettings *settings);
void CTRLSTATES_init(MulticopterCtrlStates *states);


/* Global variable declarations */
MulticopterSettings GlobalSettings;
MulticopterCtrlStates CtrlStates;

struct XYZ_angles XYZ;

#endif /* MULTICOPTER_SETTINGS_H_ */
