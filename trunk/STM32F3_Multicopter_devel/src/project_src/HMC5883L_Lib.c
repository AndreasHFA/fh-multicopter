#include <multicopter_settings.h>
#ifdef HMC5883L
#include <HMC5883L_Lib.h>
#include <helperFunctions.h>


void calibrate_MAG(void)
{
	HMC5883L_Write(MAG_ADDR ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	HMC5883L_Write(MAG_ADDR ,0x01 ,0xA0 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	HMC5883L_Write(MAG_ADDR ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
}


void MAG_AverageData(u8 rounds, struct magValues *HMC5883L_values)
{
	u8 i;
	struct magValues HMC5883L_average = {0,0,0};

	HMC5883L_average.magValueX = 0;
	HMC5883L_average.magValueY = 0;
	HMC5883L_average.magValueZ = 0;

	for(i=0;i<rounds;i++)
	{
		HMC5883L_SetRegister(MAG_ADDR, 0x03 );
		HMC5883L_BurstRead(MAG_ADDR, HMC5883L_values);

		HMC5883L_average.magValueX += HMC5883L_values->magValueX;
		HMC5883L_average.magValueY += HMC5883L_values->magValueY;
		HMC5883L_average.magValueZ += HMC5883L_values->magValueZ;

		Delay(INIT_DELAY_TIME/10);
	}
	HMC5883L_values->magValueX = HMC5883L_average.magValueX/rounds;
	HMC5883L_values->magValueY = HMC5883L_average.magValueY/rounds;
	HMC5883L_values->magValueZ = HMC5883L_average.magValueZ/rounds;
}


void MAG_convert(uint8_t I2CRxBuffer[6], struct magValues *HMC5883L_values)
{
	HMC5883L_values->magValueX = (I2CRxBuffer[0] << 8) | (I2CRxBuffer[1]);
	HMC5883L_values->magValueZ = (I2CRxBuffer[2] << 8) | (I2CRxBuffer[3]);
	HMC5883L_values->magValueY = (I2CRxBuffer[4] << 8) | (I2CRxBuffer[5]);
}



void HMC5883L_Write(u8 I2C_Address, u8 Register_Address, u8 Data )
{
	I2C_GenerateSTART(MAG_I2C, ENABLE);

	/* Test on ITG3200_I2C EV5 and clear it */
  	while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_MODE_SELECT));

  	/* Send ITG3200_I2C slave Address for write */
	I2C_Send7bitAddress(MAG_I2C, (I2C_Address<<1), I2C_Direction_Transmitter);

    /* Test on ITG3200_I2C EV6 and clear it */
    while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(MAG_I2C, Register_Address);

    while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(MAG_I2C, Data);

    while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(MAG_I2C, ENABLE);
}


void HMC5883L_SetRegister(u8 I2C_Address, u8 Register_Address )
{
	I2C_GenerateSTART(MAG_I2C, ENABLE);


	/* Test on ITG3200_I2C EV5 and clear it */
  	while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_MODE_SELECT));
  	/* Send ITG3200_I2C slave Address for write */

	//Die I2C_Addresse muss um 1 nach links verschoben werden: (I2C_Address<<1)
	//ist eine Eigenheit der STM32 Library - warum das so ist muss ich erst noch
	//herausfinden. D�rfte aber irgendwas mit dem Write/Read bit zu tun haben
	I2C_Send7bitAddress(MAG_I2C, (I2C_Address<<1), I2C_Direction_Transmitter);


    /* Test on ITG3200_I2C EV6 and clear it */
    while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(MAG_I2C, Register_Address);


    while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(MAG_I2C, ENABLE);
}


void HMC5883L_BurstRead(u8 I2C_Address, struct magValues *HMC5883L_vals)
{
		//Hier die Empfangsroutine:

		u8 Receive_Value[6] = {0,0,0,0,0,0};
		u8 i = 0;

		I2C_GenerateSTART(MAG_I2C, ENABLE);
		/* Test on ITG3200_I2C EV5 and clear it */
		while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_MODE_SELECT));

		/* Send ITG3200_I2C slave Address for write */
		I2C_Send7bitAddress(MAG_I2C, (I2C_Address<<1), I2C_Direction_Receiver);

		/* Test on ITG3200_I2C EV6 and clear it */
		while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));


		for(i = 0;i<5;i++){
			while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));

			Receive_Value[i] = I2C_ReceiveData(MAG_I2C);
		}

		I2C_AcknowledgeConfig(MAG_I2C, DISABLE);

	    I2C_GenerateSTOP(MAG_I2C, ENABLE);

		while(!I2C_CheckEvent(MAG_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));

		Receive_Value[5] = I2C_ReceiveData(MAG_I2C);


		//Decode the received values:
		HMC5883L_vals->magValueX = (Receive_Value[0] << 8) | (Receive_Value[1]);
		//if(L3G4200D->gyroValueX & (0x8000)) L3G4200D->gyroValueX = L3G4200D->gyroValueX - 65536;

		HMC5883L_vals->magValueZ = (Receive_Value[2] << 8) | (Receive_Value[3]);
		//if(L3G4200D->gyroValueY & (0x8000)) L3G4200D->gyroValueY = L3G4200D->gyroValueY - 65536;

		HMC5883L_vals->magValueY = (Receive_Value[4] << 8) | (Receive_Value[5]);
		//if(L3G4200D->gyroValueZ & (0x8000)) L3G4200D->gyroValueZ = L3G4200D->gyroValueZ - 65536;


		//nicht vergessen muss wieder eingeschaltet werden!!!
		I2C_AcknowledgeConfig(MAG_I2C, ENABLE);
}

#endif /*HMC5883L*/

