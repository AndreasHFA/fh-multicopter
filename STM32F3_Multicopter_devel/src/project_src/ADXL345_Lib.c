#include <multicopter_settings.h>
#include <ADXL345_Lib.h>
#include <helperFunctions.h>


#ifdef ADXL345
void ADXL345_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
	/* Test on BUSY Flag */
	while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_BUSY) != RESET)
	{
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(ADXL345_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_TXIS) == RESET)
	{
	}

	/* Send Register address */
	I2C_SendData(ADXL345_I2C, (uint8_t) RegAddr);

	/* Wait until TCR flag is set */
	while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_TCR) == RESET)
	{
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(ADXL345_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	/* Wait until TXIS flag is set */
	while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_TXIS) == RESET)
	{
	}

	/* Write data to TXDR */
	I2C_SendData(ADXL345_I2C, *pBuffer);

	/* Wait until STOPF flag is set */
	while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_STOPF) == RESET)
	{
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(ADXL345_I2C, I2C_ICR_STOPCF);
}


void ADXL345_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  /* Test on BUSY Flag */
  while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_BUSY) != RESET)
  {
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(ADXL345_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_TXIS) == RESET)
  {
  }

  if(NumByteToRead>1)
      RegAddr |= 0x80;


  /* Send Register address */
  I2C_SendData(ADXL345_I2C, (uint8_t)RegAddr);

  /* Wait until TC flag is set */
  while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_TC) == RESET)
  {
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(ADXL345_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Wait until all data are received */
  while (NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_RXNE) == RESET)
    {
    }

    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(ADXL345_I2C);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;

    /* Decrement the read bytes counter */
    NumByteToRead--;
  }

  /* Wait until STOPF flag is set */
  while(I2C_GetFlagStatus(ADXL345_I2C, I2C_ISR_STOPF) == RESET)
  {
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(ADXL345_I2C, I2C_ICR_STOPCF);
}



void ADXL345_AccInit(void)
{
	uint8_t power_ctl = 0x08, dataformat = 0x0B, bw_rate = 0x0B;
	ADXL345_Write((uint8_t)ADXL345_I2C_ADDR, 0x2D, &power_ctl);
	ADXL345_Write((uint8_t)ADXL345_I2C_ADDR, 0x31, &dataformat);
	ADXL345_Write((uint8_t)ADXL345_I2C_ADDR, 0x2C, &bw_rate);
}


void ADXL345_readValues(struct accValues *accXYZ)
{
	uint8_t buffer[6];

	/* Read the register content */
	ADXL345_Read(ADXL345_I2C_ADDR, ADXL345_X_ADDR, buffer, 6);

	accXYZ->accValueX = ((buffer[1] << 8) | (buffer[0])) + X_OFFSET;
	accXYZ->accValueY = ((buffer[3] << 8) | (buffer[2])) + Y_OFFSET;
	accXYZ->accValueZ = (buffer[5] << 8) | (buffer[4]);
}



void ADXL345_calibrate(float maxGOverflow, struct accValues *Acc_values)
{
	u8 i;
	struct accValues ADXL_average = {0,0,0};

	ADXL345_AccInit();

	for(i=0;i<AVG_ROUNDS;i++)
	{
		ADXL345_readValues(Acc_values);

		ADXL_average.accValueX += (Acc_values->accValueX);
		ADXL_average.accValueY += (Acc_values->accValueY);
		ADXL_average.accValueZ += (Acc_values->accValueZ);

		Delay(INIT_DELAY_TIME/10);
	}
	Acc_values->accValueX = (ADXL_average.accValueX/AVG_ROUNDS);
	Acc_values->accValueY = (ADXL_average.accValueY/AVG_ROUNDS);
	Acc_values->accValueZ = (ADXL_average.accValueZ/AVG_ROUNDS);

	Acc_values->accGValue = sqrtf((Acc_values->accValueX*Acc_values->accValueX) +
								  (Acc_values->accValueY*Acc_values->accValueY) +
							      (Acc_values->accValueZ*Acc_values->accValueZ));

	/* Allow some kind of overflow */
	Acc_values->accGValue = Acc_values->accGValue*maxGOverflow;
}



void ACC_convert(uint8_t I2CRxBuffer[6], struct accValues *acc)
{
	acc->accValueX = (I2CRxBuffer[1] << 8) | (I2CRxBuffer[0]);
	acc->accValueY = (I2CRxBuffer[3] << 8) | (I2CRxBuffer[2]);
	acc->accValueZ = (I2CRxBuffer[5] << 8) | (I2CRxBuffer[4]);
}
#endif


