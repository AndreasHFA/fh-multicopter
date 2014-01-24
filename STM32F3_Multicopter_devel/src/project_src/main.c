#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <stm32f30x_it.h>
#include <stm32_configuration.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usart.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <croutine.h>

#include <IMU_algorithm.h>
#include <filter_Lib.h>
#include <multicopter_settings.h>
#include <xBee_Lib.h>
#include <L3GD20_Lib_SPI.h>
#include <ADXL345_Lib.h>
#include <LSM303DLHC_Lib_I2C.h>
#include <MotorPWM.h>
#include <PID_control.h>

#define DEG_TO_RAD 0.017453292f

//#define ESC_CALIBRATE		// Enstufen Kalibrieren
//#define DEBUG				// Motoren werden abgeschalten

#ifndef DEBUG
	#define enableX_AXIS
	#define enableY_AXIS
#endif

//#define USART_REMOTE

/*Max Gyro Error when calibrating*/
#define MAX_GYRO_ERROR 0.15f

/* dt-Time of Main IMU Task*/
#define DT 2
#define DT_S ((float)DT/1000)

#define ACC_MEDIAN_VALUE 5

static void vLedCtrlCoRoutine( xCoRoutineHandle xHandle, unsigned portBASE_TYPE uxIndex );
static void vRemoteCtrlWatchdogCoRoutine( xCoRoutineHandle xHandle, unsigned portBASE_TYPE uxIndex );
static void IMU_Calculation( void *pvParameters );
static void IMU_Print_Values( void *pvParameters );
static void PID_Calculation( void *pvParameters );

uint8_t accBuffer[6];

int xTemp = 0;
int yTemp = 0;
int zTemp = 0;

//
USART_OBJ It_Com;
/************ COM ***************/
int loopcnt = 0;


int main()
{
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	setup_xBeeS6();
	SPI_Configuration();
	I2C_Configuration();
	TIM2_PWM_Configuration();
	TIM3_PWM_Configuration();
	TIM4_Configuration();
	ADC_Configuration();
	EXTI_Configuration();
	DMA_Configuration(&accBuffer[0]);
	//TIM4_PWM_INPUT_Configuration();

	SETTINGS_init(&GlobalSettings);
	CTRLSTATES_init(&CtrlStates);

	xCoRoutineCreate( vLedCtrlCoRoutine, 0, 0 );
	xCoRoutineCreate( vRemoteCtrlWatchdogCoRoutine, 1, 0 );

	xTaskCreate( IMU_Calculation, ( signed char * ) "IMU_Gyro", configMINIMAL_STACK_SIZE, NULL, 4, NULL );
	xTaskCreate( PID_Calculation, ( signed char * ) "PID_Calc", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	xTaskCreate( IMU_Print_Values, ( signed char * ) "Print_Euler", configMINIMAL_STACK_SIZE, NULL, 2, NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();
	while(1);
}

static void IMU_Calculation( void *pvParameters )
{
	portTickType xNextWakeTime;

	uint16_t i = 0;

	struct IMU_values IMU;

	/************ GYRO **************/
	struct gyroValues gyroOffset = {0,0,0};
	struct gyroValues gyroXYZ = {0,0,0};

	floatFilter xGyroFiltered_float;
	floatFilter yGyroFiltered_float;
	floatFilter zGyroFiltered_float;
	memset(&xGyroFiltered_float, 0, sizeof(xGyroFiltered_float));
	memset(&yGyroFiltered_float, 0, sizeof(yGyroFiltered_float));
	memset(&zGyroFiltered_float, 0, sizeof(zGyroFiltered_float));

	/************ ACC ***************/
	struct accValues accXYZ = {0,0,0};
	uint8_t accCount = 0;

	floatFilter xAccFiltered_float;
	floatFilter yAccFiltered_float;
	floatFilter zAccFiltered_float;
	memset(&xAccFiltered_float, 0, sizeof(xAccFiltered_float));
	memset(&yAccFiltered_float, 0, sizeof(yAccFiltered_float));
	memset(&zAccFiltered_float, 0, sizeof(zAccFiltered_float));

	/************ MAG ***************/
	uint8_t magCount = 0;
	struct magValues magXYZ = {0,0,0};
	struct EULER_angles eulerTemp;
	intFilter xMagFiltered = {0,0};
	intFilter yMagFiltered = {0,0};


	/*Initialize the DCM-variables*/
	IMU_init(&IMU, DT_S);
	if(GlobalSettings.enableDriftCorrection)
	{
		IMU_init_drift_correction(&IMU, 0.00020f/*kp-value*/);
	}

	/*
	 * Wait here 2 Seconds to ensure,
	 * that all capacitors are charged
	 * and VCC is stable
	 * */
	vTaskDelay( 2000 );

	while(1)
	{
		Gyro_calibrate(&gyroOffset);
		for (i = 0;i<5000;i++)
		{
			Gyro_readValues(&gyroXYZ);
			gyroXYZ.gyroValueX = ((gyroXYZ.gyroValueX-gyroOffset.gyroValueX));
			gyroXYZ.gyroValueY = ((gyroXYZ.gyroValueY-gyroOffset.gyroValueY));
			gyroXYZ.gyroValueZ = ((gyroXYZ.gyroValueZ-gyroOffset.gyroValueZ));

			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] += ((float)gyroXYZ.gyroValueX*GYROCONVERT)*DT_S;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] += ((float)gyroXYZ.gyroValueX*GYROCONVERT)*DT_S;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] += ((float)gyroXYZ.gyroValueX*GYROCONVERT)*DT_S;
		}
		vTaskDelayUntil( &xNextWakeTime, DT );

		if((IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] > -MAX_GYRO_ERROR) && (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] < MAX_GYRO_ERROR) &&
		   (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] > -MAX_GYRO_ERROR) && (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] < MAX_GYRO_ERROR) &&
		   (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] > -MAX_GYRO_ERROR) && (IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] < MAX_GYRO_ERROR))
		{
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] = 0;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] = 0;
			IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] = 0;
			break;
		}
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] = 0;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] = 0;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] = 0;
		printf("Error Gyro Test!\n\r");
	}

	if(GlobalSettings.enableACC)
	{
		//ADXL345_calibrate(GlobalSettings.accMaxGOverflow, &accXYZ);
		LSM303DLHC_ACC_calibrate(GlobalSettings.accMaxGOverflow, &accXYZ);
	}
	if(GlobalSettings.enableMAG)
	{
		Mag_calibrate();
		vTaskDelay( 200 );
		Mag_readValues(&magXYZ);
		eulerTemp.yaw = atan2((float)(-magXYZ.magValueX),(float)(magXYZ.magValueY))-1.5707;
		CtrlStates.yaw = -eulerTemp.yaw*(180/M_PI);
		/* The yaw value needs a special handling */
		if(CtrlStates.yaw > 180) CtrlStates.yaw -= 360;
		if(CtrlStates.yaw < -180) CtrlStates.yaw += 360;
		IMU_Euler_to_DCM(&IMU, &eulerTemp);
	}

	/* Set the START Flag! */
	CtrlStates.copterStatus |= IMU_READY_FLAG;

	xNextWakeTime = xTaskGetTickCount();

	while(1)
	{
		GPIOC->ODR ^= GPIO_Pin_11;
		Gyro_readValues(&gyroXYZ);
		gyroXYZ.gyroValueX = ((gyroXYZ.gyroValueX-gyroOffset.gyroValueX));
		gyroXYZ.gyroValueY = ((gyroXYZ.gyroValueY-gyroOffset.gyroValueY));
		gyroXYZ.gyroValueZ = ((gyroXYZ.gyroValueZ-gyroOffset.gyroValueZ));

		/* Cut-off the lower values to reduce noise and fail-integration*/
		if(gyroXYZ.gyroValueX > -GlobalSettings.gyroSensTreshold && gyroXYZ.gyroValueX < GlobalSettings.gyroSensTreshold) gyroXYZ.gyroValueX = 0;
		if(gyroXYZ.gyroValueY > -GlobalSettings.gyroSensTreshold && gyroXYZ.gyroValueY < GlobalSettings.gyroSensTreshold) gyroXYZ.gyroValueY = 0;
		if(gyroXYZ.gyroValueZ > -GlobalSettings.gyroSensTreshold && gyroXYZ.gyroValueZ < GlobalSettings.gyroSensTreshold) gyroXYZ.gyroValueZ = 0;

		floatFilterGyro(&xGyroFiltered_float, gyroXYZ.gyroValueX);
		floatFilterGyro(&yGyroFiltered_float, gyroXYZ.gyroValueY);
		floatFilterGyro(&zGyroFiltered_float, gyroXYZ.gyroValueZ);

		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensX]] = GlobalSettings.gyroDir.sensDirection[sensX]*(xGyroFiltered_float.currentValue*GYROCONVERT)*DEG_TO_RAD;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensY]] = GlobalSettings.gyroDir.sensDirection[sensY]*(yGyroFiltered_float.currentValue*GYROCONVERT)*DEG_TO_RAD;
		IMU.velocity_vector[GlobalSettings.gyroDir.sensOrder[sensZ]] = GlobalSettings.gyroDir.sensDirection[sensZ]*(zGyroFiltered_float.currentValue*GYROCONVERT)*DEG_TO_RAD;

		/*Here the ACC-Compensation, we do this in every tenth round*/
		if(GlobalSettings.enableACC)
		{
			if(accCount >= 0)
			{
				//ADXL345_readValues(&accXYZ);
				//LSM303DLHC_ACC_readValues(&accXYZ);
				LSM303DLHC_ACC_readValuesDMA(&accXYZ, &accBuffer[0]);
				printf("%d %d %d\n\r", accXYZ.accValueX,accXYZ.accValueY, accXYZ.accValueZ );
				//ADC_AutoInjectedConvCmd(ADC2,ENABLE);
//				ADC_StartConversion(ADC2);
//				/* wait for ADRDY */
//				while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
//				ADC_StartInjectedConversion(ADC2);
//
//				accXYZ.accValueX = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_2)-1700;
//				accXYZ.accValueZ = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_1)-1700;
//				accXYZ.accValueY = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_3)-1700;

				floatFilterAcc(&xAccFiltered_float, accXYZ.accValueX);
				floatFilterAcc(&yAccFiltered_float, accXYZ.accValueY);
				floatFilterAcc(&zAccFiltered_float, accXYZ.accValueZ);

				if((xAccFiltered_float.currentValue < accXYZ.accGValue) && (xAccFiltered_float.currentValue > -accXYZ.accGValue) &&
				   (yAccFiltered_float.currentValue < accXYZ.accGValue) && (yAccFiltered_float.currentValue > -accXYZ.accGValue) &&
				   (zAccFiltered_float.currentValue < accXYZ.accGValue) && (zAccFiltered_float.currentValue > -accXYZ.accGValue))
				{
					/*Direction setup is the same as above*/
					IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensX]] = GlobalSettings.accDir.sensDirection[sensX]*xAccFiltered_float.currentValue;
					IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensY]] = GlobalSettings.accDir.sensDirection[sensY]*yAccFiltered_float.currentValue;
					IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensZ]] = GlobalSettings.accDir.sensDirection[sensZ]*zAccFiltered_float.currentValue;

					xTemp = (int)IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensX]];
					yTemp = (int)IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensY]];
					zTemp = (int)IMU.accel_vector[GlobalSettings.accDir.sensOrder[sensZ]];
				}
				else
				{
					IMU.kp_vector_ACC[0] = 0;
					IMU.kp_vector_ACC[1] = 0;
					IMU.kp_vector_ACC[2] = 0;

					xTemp = 0;
					yTemp = 0;
					zTemp = 0;
				}

				if(GlobalSettings.enableDriftCorrection)
				{
					IMU_drift_correction_ACC(&IMU);
				}
				accCount = 0;
			}
			accCount++;
		}

		/*Here the ACC-Compensation, we do this in every tenth round*/
		if(GlobalSettings.enableMAG)
		{
			if(magCount >= 45)
			{
				Mag_readValues(&magXYZ);
				update_intFilter(&xMagFiltered, GlobalSettings.magLowpassValue, (magXYZ.magValueX));
				update_intFilter(&yMagFiltered, GlobalSettings.magLowpassValue, (magXYZ.magValueY));

				/*Direction setup is the same as above*/
				IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensX]] = GlobalSettings.magDir.sensDirection[sensX]*xMagFiltered.filtered;
				IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensY]] = GlobalSettings.magDir.sensDirection[sensY]*yMagFiltered.filtered;
				//IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensZ]] = GlobalSettings.magDir.sensDirection[sensZ]*zMagFiltered;
				IMU.mag_vector[GlobalSettings.magDir.sensOrder[sensZ]] = 0;

				if(GlobalSettings.enableDriftCorrection)
				{
					IMU_drift_correction_MAG(&IMU);
				}
				magCount = 0;
				//printf("%d,%d\n\r", (int)xMagFiltered.filtered, (int)yMagFiltered.filtered);
			}
			magCount++;
		}

		/*This functions here do the DCM-Algorithm stuff*/
		IMU_update(&IMU);
		IMU_ortho_adjust(&IMU);
		IMU_normalize(&IMU);
		//Euler_angles(&DCM);
		//IMU_DCM_to_Euler_deg(&IMU, &euler);
		IMU_DCM_to_XYZ(&IMU, &XYZ);

		/*
		 *  KeinePufferung
		 *  CtrlStates.gas, CtrlStates.nick, CtrlStates.roll,
		 */
		/*
		if (loopcnt >= 10) {
			uint32_t Header = 0xFF7F3F1F;
			InttoBuffer(&It_Com.txbuffer[0], Header); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
			InttoBuffer(&It_Com.txbuffer[4], xTemp);		//xTemp);
			InttoBuffer(&It_Com.txbuffer[8], yTemp);		// yTemp);
			InttoBuffer(&It_Com.txbuffer[12], zTemp);		//zTemp);

			loopcnt = 0;
			It_Com.txlen = 16;
			USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		} else {
			loopcnt++;
		}
		*/

		/*
		 *  Mit Pufferung
		 */
		/*
		uint32_t Header = 0xFF7F3F1F;
		InttoBuffer(&It_Com.rxbuffer[(loopcnt + 0)], Header); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
		InttoBuffer(&It_Com.rxbuffer[(loopcnt + 4)], xTemp);
		InttoBuffer(&It_Com.rxbuffer[(loopcnt + 8)], yTemp);
		InttoBuffer(&It_Com.rxbuffer[(loopcnt + 12)],zTemp);

		if (loopcnt > (9 * 16) ) {
			memcpy((void *) &It_Com.txbuffer[0], (void *) &It_Com.rxbuffer[0], (loopcnt));
			It_Com.txlen = loopcnt;
			loopcnt = 0;
			USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		}else{
			loopcnt= loopcnt + 16; // Datenblock
		}
		*/

		vTaskDelayUntil( &xNextWakeTime, DT );
	}
}


/*
 *		PID Calculation - this is far from being finished
 * */

static void PID_Calculation( void *pvParameters )
{
	const portTickType xDelay = 3 / portTICK_RATE_MS;

	float MOT0 = 0;
	float MOT1 = 0;
	float MOT2 = 0;
	float MOT3 = 0;
	float MOT4 = 0;
	float MOT5 = 0;

	PID_Control PID_X1;
	PID_Control PID_X2;
	PID_Control PID_Y1;
	PID_Control PID_Y2;

	PID_Control PID_Z;

	init_PID(0.003f, GlobalSettings.PID_rollPitch.PID_kp, GlobalSettings.PID_rollPitch.PID_ki, GlobalSettings.PID_rollPitch.PID_kd, &PID_X1);
	init_PID(0.003f, GlobalSettings.PID_rollPitch.PID_kp, GlobalSettings.PID_rollPitch.PID_ki, GlobalSettings.PID_rollPitch.PID_kd, &PID_X2);
	init_PID(0.003f, GlobalSettings.PID_rollPitch.PID_kp, GlobalSettings.PID_rollPitch.PID_ki, GlobalSettings.PID_rollPitch.PID_kd, &PID_Y1);
	init_PID(0.003f, GlobalSettings.PID_rollPitch.PID_kp, GlobalSettings.PID_rollPitch.PID_ki, GlobalSettings.PID_rollPitch.PID_kd, &PID_Y2);
	init_PID(0.003f, GlobalSettings.PID_yaw.PID_kp, GlobalSettings.PID_yaw.PID_ki, GlobalSettings.PID_yaw.PID_kd, &PID_Z);

	while(1)
	{
		if(CtrlStates.copterStatus & ARMED_FLAG )
		{
			#ifdef ESC_CALIBRATE
			MOT0 = MOT1 = MOT2 = MOT3 = MOT4 = MOT5 = GlobalSettings.stopSpeed + CtrlStates.gas;

			PWM_Motor(0, (uint16_t)MOT0);
			PWM_Motor(1, (uint16_t)MOT1);
			PWM_Motor(2, (uint16_t)MOT2);
			PWM_Motor(3, (uint16_t)MOT3);
			PWM_Motor(4, (uint16_t)MOT4);
			PWM_Motor(5, (uint16_t)MOT5);
			#else
			//Mischen der Werte:
			#ifdef QUADRO
			calc_PID_YAW(XYZ.z, CtrlStates.yaw, &PID_Z);

			//PID-Regler
			calc_PID(XYZ.y, -CtrlStates.roll, &PID_X1);
			calc_PID(XYZ.x, CtrlStates.nick, &PID_X2);
			calc_PID(XYZ.y, CtrlStates.roll, &PID_Y1);
			calc_PID(XYZ.x, -CtrlStates.nick, &PID_Y2);

			MOT0 = GlobalSettings.minSpeed + CtrlStates.gas + PID_X2.y - PID_X1.y - PID_Z.y;
			MOT2 = GlobalSettings.minSpeed + CtrlStates.gas - PID_X2.y + PID_X1.y - PID_Z.y;
			MOT1 = GlobalSettings.minSpeed + CtrlStates.gas + PID_Y1.y + PID_Y2.y + PID_Z.y;
			MOT3 = GlobalSettings.minSpeed + CtrlStates.gas - PID_Y1.y - PID_Y2.y + PID_Z.y;
			#endif
			#ifdef HEXA
			calc_PID_YAW(XYZ.z, CtrlStates.yaw, &PID_Z);

//			#define MAXPIDZ 50
//			if(PID_Z.y > MAXPIDZ) PID_Z.y = MAXPIDZ;
//			if(PID_Z.y < -MAXPIDZ) PID_Z.y = -MAXPIDZ;

			//PID-Regler
			calc_PID(XYZ.y, -CtrlStates.nick, &PID_X1);
			calc_PID(XYZ.x, CtrlStates.roll, &PID_X2);
			calc_PID(XYZ.y, -CtrlStates.nick, &PID_Y1);
			calc_PID(XYZ.x, CtrlStates.roll, &PID_Y2);

			MOT0 = GlobalSettings.minSpeed + CtrlStates.gas + PID_X1.y + PID_Z.y;
			MOT2 = GlobalSettings.minSpeed + CtrlStates.gas + PID_X2.y - (0.7*PID_X1.y) + PID_Z.y;
			MOT1 = GlobalSettings.minSpeed + CtrlStates.gas + (0.7*PID_Y1.y) + PID_Y2.y - PID_Z.y;
			MOT3 = GlobalSettings.minSpeed + CtrlStates.gas - PID_Y1.y - PID_Z.y;
			MOT4 = GlobalSettings.minSpeed + CtrlStates.gas - PID_X2.y - (0.7*PID_X1.y) + PID_Z.y;
			MOT5 = GlobalSettings.minSpeed + CtrlStates.gas + (0.7*PID_Y1.y) - PID_Y2.y - PID_Z.y;
			#endif

			if(MOT1 < GlobalSettings.minSpeed) MOT1 = GlobalSettings.minSpeed;
			if(MOT1 > GlobalSettings.maxSpeed) MOT1 = GlobalSettings.maxSpeed;

			if(MOT3 < GlobalSettings.minSpeed) MOT3 = GlobalSettings.minSpeed;
			if(MOT3 > GlobalSettings.maxSpeed) MOT3 = GlobalSettings.maxSpeed;

			if(MOT0 < GlobalSettings.minSpeed) MOT0 = GlobalSettings.minSpeed;
			if(MOT0 > GlobalSettings.maxSpeed) MOT0 = GlobalSettings.maxSpeed;

			if(MOT2 < GlobalSettings.minSpeed) MOT2 = GlobalSettings.minSpeed;
			if(MOT2 > GlobalSettings.maxSpeed) MOT2 = GlobalSettings.maxSpeed;

			#ifdef HEXA
			if(MOT4 < GlobalSettings.minSpeed) MOT4 = GlobalSettings.minSpeed;
			if(MOT4 > GlobalSettings.maxSpeed) MOT4 = GlobalSettings.maxSpeed;

			if(MOT5 < GlobalSettings.minSpeed) MOT5 = GlobalSettings.minSpeed;
			if(MOT5 > GlobalSettings.maxSpeed) MOT5 = GlobalSettings.maxSpeed;
			#endif

			taskENTER_CRITICAL();

		#ifdef QUADRO
			#ifdef enableX_AXIS
			PWM_Motor(3, (uint16_t)MOT2);
			PWM_Motor(0, (uint16_t)MOT0);
			#endif /*enableX_AXIS*/
			#ifdef enableY_AXIS
			PWM_Motor(2, (uint16_t)MOT3);
			PWM_Motor(1, (uint16_t)MOT1);
			#endif /*enableY_AXIS*/
		#endif
		#ifdef HEXA
			#ifdef enableX_AXIS
			//PWM_Motor(3, (uint16_t)MOT2);
			PWM_Motor(0, (uint16_t)MOT0);
			PWM_Motor(2, (uint16_t)MOT2);
			PWM_Motor(4, (uint16_t)MOT4);
			#endif /*enableX_AXIS*/
			#ifdef enableY_AXIS
			PWM_Motor(5, (uint16_t)MOT5);
			PWM_Motor(3, (uint16_t)MOT3);
			PWM_Motor(1, (uint16_t)MOT1);
			#endif /*enableY_AXIS*/
		#endif

			taskEXIT_CRITICAL();

			//printf("%d,%d,%d,%d\n\r", (uint16_t)MOT1, (uint16_t)MOT2, (uint16_t)MOT3, (uint16_t)MOT4);
			#endif
		}
		else
		{
			resetIntegralValues_PID(&PID_X1);
			resetIntegralValues_PID(&PID_X2);
			resetIntegralValues_PID(&PID_Y1);
			resetIntegralValues_PID(&PID_Y2);
			resetIntegralValues_PID(&PID_Z);

			PWM_Motor(1, (uint16_t)GlobalSettings.stopSpeed);
			PWM_Motor(3, (uint16_t)GlobalSettings.stopSpeed);
			PWM_Motor(0, (uint16_t)GlobalSettings.stopSpeed);
			PWM_Motor(2, (uint16_t)GlobalSettings.stopSpeed);

			PWM_Motor(4, (uint16_t)GlobalSettings.stopSpeed);
			PWM_Motor(5, (uint16_t)GlobalSettings.stopSpeed);
		}
		//printf("%d %d\n\r", (int16_t)MOT1, (int16_t)MOT3);

		/*
		 *  KeinePufferung
		 *  CtrlStates.gas, CtrlStates.nick, CtrlStates.roll,
		 */
		if (loopcnt >= 10) {
			uint32_t Header = 0xFF7F3F1F;
			InttoBuffer(&It_Com.txbuffer[0], Header); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
			InttoBuffer(&It_Com.txbuffer[4], (uint16_t) MOT1);		//xTemp);
			InttoBuffer(&It_Com.txbuffer[8], (uint16_t) MOT3);		// yTemp);
			InttoBuffer(&It_Com.txbuffer[12], (uint16_t) MOT5);		//zTemp);

			loopcnt = 0;
			It_Com.txlen = 16;
			USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		} else {
			loopcnt++;
		}




		vTaskDelay( xDelay );
	}
}


/*For debug purposes - the print task*/
static void IMU_Print_Values( void *pvParameters )
{
	const portTickType xDelay = 40 / portTICK_RATE_MS;

	while(1)
	{
		//printf("%d,%d,%d,%d %d  ",CtrlStates.gas, CtrlStates.nick, CtrlStates.roll, CtrlStates.yaw, CtrlStates.copterStatus);
		//printf("%d,%d,%d\n",(int)XYZ.y, (int)XYZ.x, (int)XYZ.z);
		printf("test:%d,%d,%d\r\n", (int)(xTemp), (int)(yTemp), (int)(zTemp));
		vTaskDelay( xDelay );
	}
}



void LED_TOGGLE_NOT_READY(void)
{
	GPIOE->ODR ^= GPIO_Pin_8;
	GPIOE->ODR ^= GPIO_Pin_10;
	GPIOE->ODR ^= GPIO_Pin_12;
	GPIOE->ODR ^= GPIO_Pin_14;
}

void LED_TOGGLE_READY(void)
{
	GPIOE->ODR ^= GPIO_Pin_9;
	GPIOE->ODR ^= GPIO_Pin_11;
	GPIOE->ODR ^= GPIO_Pin_13;
	GPIOE->ODR ^= GPIO_Pin_15;
}

void LED_TOGGLE_ALL(void)
{
	if(GPIOE->ODR & GPIO_Pin_9)
	{
		GPIOE->ODR &= ~GPIO_Pin_9;
		GPIOE->ODR &= ~GPIO_Pin_11;
		GPIOE->ODR &= ~GPIO_Pin_13;
		GPIOE->ODR &= ~GPIO_Pin_15;
		GPIOE->ODR |= GPIO_Pin_8;
		GPIOE->ODR |= GPIO_Pin_10;
		GPIOE->ODR |= GPIO_Pin_12;
		GPIOE->ODR |= GPIO_Pin_14;
	}
	else
	{
		GPIOE->ODR |= GPIO_Pin_9;
		GPIOE->ODR |= GPIO_Pin_11;
		GPIOE->ODR |= GPIO_Pin_13;
		GPIOE->ODR |= GPIO_Pin_15;
		GPIOE->ODR &= ~GPIO_Pin_8;
		GPIOE->ODR &= ~GPIO_Pin_10;
		GPIOE->ODR &= ~GPIO_Pin_12;
		GPIOE->ODR &= ~GPIO_Pin_14;
	}
}

void vLedCtrlCoRoutine( xCoRoutineHandle xHandle,
                         unsigned portBASE_TYPE uxIndex )
{
   // Co-routines must start with a call to crSTART().
   crSTART( xHandle );

   for( ;; )
   {
	   // Delay for a fixed period.
	   crDELAY( xHandle, 200 );

	   if(CtrlStates.copterStatus == IMU_READY_FLAG)
	   {
		   LED_TOGGLE_READY();
	   }
	   else if (CtrlStates.copterStatus & ARMED_FLAG)
	   {
		   LED_TOGGLE_ALL();
	   }
	   else
	   {
		   LED_TOGGLE_NOT_READY();
	   }
   }

   // Co-routines must end with a call to crEND().
   crEND();
}

void vRemoteCtrlWatchdogCoRoutine( xCoRoutineHandle xHandle,
                         unsigned portBASE_TYPE uxIndex )
{
   // Co-routines must start with a call to crSTART().
   crSTART( xHandle );

   for( ;; )
   {
	   // Delay for a fixed period.
	   crDELAY( xHandle, 1000 );
	   CtrlStates.watchDogTimVal++;
	   if (CtrlStates.watchDogTimVal >= 2)
	   {
		   CtrlStates.copterStatus &= ~ARMED_FLAG;
		   printf("Watchdog Error!!%d\n", 0);
	   }
   }

   // Co-routines must end with a call to crEND().
   crEND();
}





void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that
	remains unallocated. */
	vCoRoutineSchedule();
}


void vApplicationTickHook( void )
{


}

//***************************************************************************
// *  USART3 Interrupt
//    Dient zur schnellen Übertragung der aktuellen Telemetriedaten
// **************************************************************************
void USART3_IRQHandler(void)
{
  	// Transmit Data Register empty interrupt
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
  	{
  		USART_ClearITPendingBit(USART3, USART_IT_TXE);

  		//if(ModbusData.txcnt <= ModbusData.txlen)
  		if(It_Com.txcnt <= (It_Com.txlen-1))
  		{
            USART_SendData(USART3,It_Com.txbuffer[It_Com.txcnt++]);
  		}
  		else
  		{
  			It_Com.txcnt = 0;
  			It_Com.txready = 0;
  		    //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  		    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  		}
  	}
}
