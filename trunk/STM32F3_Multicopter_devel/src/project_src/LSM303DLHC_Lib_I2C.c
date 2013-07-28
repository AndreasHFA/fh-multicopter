#include <LSM303DLHC_Lib_I2C.h>


void LSM303DLHC_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
  /* Test on BUSY Flag */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_BUSY) != RESET)
  {
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)
  {
  }

  /* Send Register address */
  I2C_SendData(LSM303DLHC_I2C, (uint8_t) RegAddr);

  /* Wait until TCR flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TCR) == RESET)
  {
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

  /* Wait until TXIS flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)
  {
  }

  /* Write data to TXDR */
  I2C_SendData(LSM303DLHC_I2C, *pBuffer);

  /* Wait until STOPF flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_STOPF) == RESET)
  {
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(LSM303DLHC_I2C, I2C_ICR_STOPCF);
}


void LSM303DLHC_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  /* Test on BUSY Flag */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_BUSY) != RESET)
  {
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)
  {
  }

  if(NumByteToRead>1)
      RegAddr |= 0x80;


  /* Send Register address */
  I2C_SendData(LSM303DLHC_I2C, (uint8_t)RegAddr);

  /* Wait until TC flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TC) == RESET)
  {
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Wait until all data are received */
  while (NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_RXNE) == RESET)
    {
    }

    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(LSM303DLHC_I2C);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;

    /* Decrement the read bytes counter */
    NumByteToRead--;
  }

  /* Wait until STOPF flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_STOPF) == RESET)
  {
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(LSM303DLHC_I2C, I2C_ICR_STOPCF);
}

void LSM303DLHC_ReadDMA(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  //printf("M1%d ", 0);

//  if(DMA_GetFlagStatus(DMA1_FLAG_TC7))
//  {
//	  printf("unread data!%d\n",0);
//	  return;
//  }

  //DMA_Configuration(pBuffer);
  /* Test on BUSY Flag */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_BUSY) != RESET)
  {
  }


  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)
  {
  }

  if(NumByteToRead>1)
      RegAddr |= 0x80;

  //printf("M2%d ", 0);

  /* Send Register address */
  I2C_SendData(LSM303DLHC_I2C, (uint8_t)RegAddr);

  /* Wait until TXIS flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TC) == RESET)
  {
  }

  //printf("M3%d ", 0);

//  I2C_SlaveAddressConfig(LSM303DLHC_I2C, DeviceAddr);
//  I2C_MasterRequestConfig(LSM303DLHC_I2C, I2C_Direction_Receiver);
//  I2C_GenerateSTART(LSM303DLHC_I2C, ENABLE);
//  I2C_SlaveAddressConfig(LSM303DLHC_I2C, DeviceAddr);
  //I2C_MasterRequestConfig(LSM303DLHC_I2C, I2C_Direction_Receiver);
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Enable DMA1 Channel5 */
  I2C_DMACmd(LSM303DLHC_I2C, I2C_DMAReq_Rx, ENABLE);
DMA_Cmd(DMA1_Channel7, ENABLE);
  /* Wait until all data are received */
//  while (NumByteToRead)
//  {
//    /* Wait until RXNE flag is set */
//    while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_RXNE) == RESET)
//    {
//    }
//
//    /* Read data from RXDR */
//    *pBuffer = I2C_ReceiveData(LSM303DLHC_I2C);
//    /* Point to the next location where the byte read will be saved */
//    pBuffer++;
//
//    /* Decrement the read bytes counter */
//    NumByteToRead--;
//  }

//printf("M4%d ", 0);

  /* DMA1 Channel1 transfer complete test */
  while(!DMA_GetFlagStatus(DMA1_FLAG_TC7));

  //printf("M5%d ", 0);

  DMA_ClearFlag(DMA1_FLAG_GL7);

  I2C_DMACmd(LSM303DLHC_I2C, I2C_DMAReq_Rx, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);

  //GPIO_SetBits(GPIOE,GPIO_Pin_9);

  //I2C_GenerateSTOP(LSM303DLHC_I2C, ENABLE);

  /* Wait until STOPF flag is set */
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_STOPF) == RESET)
  {
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(LSM303DLHC_I2C, I2C_ICR_STOPCF);
}


#ifdef LSM303_ACC
void LSM303DLHC_AccInit(void)
{
  uint8_t ctrl1 = 0x00, ctrl2 = 0x00, ctrl4 = 0x00;

  /* Configure MEMS: data rate, power mode, full scale and axes */
//  ctrl1 |= (uint8_t) (LSM303DLHC_NORMAL_MODE | LSM303DLHC_ODR_200_HZ |
//		  	  	  	  LSM303DLHC_AXES_ENABLE);

    ctrl1 |= (uint8_t) (LSM303DLHC_NORMAL_MODE | LSM303DLHC_ODR_1344_HZ |
  		  	  	  	  	LSM303DLHC_AXES_ENABLE);


//  ctrl4 |= (uint8_t) (LSM303DLHC_BlockUpdate_Continous | LSM303DLHC_BLE_LSB |
//		  	  	  	  LSM303DLHC_FULLSCALE_16G | LSM303DLHC_HR_ENABLE);

  ctrl4 |= (uint8_t) (LSM303DLHC_BlockUpdate_Continous | LSM303DLHC_BLE_LSB |
  		  	  	  	  LSM303DLHC_FULLSCALE_16G | LSM303DLHC_HR_ENABLE);

  //ctrl4 = 0x30;

  /* Write value to ACC MEMS CTRL_REG1 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1);

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &ctrl2);

  /* Write value to ACC MEMS CTRL_REG4 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4);
}


void LSM303DLHC_ACC_calibrate(float maxGOverflow, struct accValues *Acc_values)
{
	u8 i;
	uint32_t tmpX = 0;
	uint32_t tmpY = 0;
	uint32_t tmpZ = 0;

	LSM303DLHC_AccInit();

	for(i=0;i<AVG_ROUNDS;i++)
	{
		LSM303DLHC_ACC_readValues(Acc_values);

		tmpX += (Acc_values->accValueX);
		tmpY += (Acc_values->accValueY);
		tmpZ += (Acc_values->accValueZ);

		Delay(INIT_DELAY_TIME/10);
	}
	Acc_values->accValueX = (tmpX/AVG_ROUNDS);
	Acc_values->accValueY = (tmpY/AVG_ROUNDS);
	Acc_values->accValueZ = (tmpZ/AVG_ROUNDS);

	Acc_values->accGValue = sqrtf((Acc_values->accValueX*Acc_values->accValueX) +
									(Acc_values->accValueY*Acc_values->accValueY) +
									(Acc_values->accValueZ*Acc_values->accValueZ));

	/* Allow some kind of overflow */
	Acc_values->accGValue = Acc_values->accGValue*maxGOverflow;
}


void LSM303DLHC_ACC_readValues(struct accValues *accXYZ)
{
	uint8_t buffer[6], cDivider;

	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

	cDivider=16;

	/* check in the control register4 the data alignment*/
	accXYZ->accValueX = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0])/cDivider;
	accXYZ->accValueY = ((int16_t)((uint16_t)buffer[3] << 8) + buffer[2])/cDivider;
	accXYZ->accValueZ = ((int16_t)((uint16_t)buffer[5] << 8) + buffer[4])/cDivider;
}

void LSM303DLHC_ACC_readValuesDMA(struct accValues *accXYZ, uint8_t *buffer)
{
	uint8_t cDivider;

	/* Read the register content */
	LSM303DLHC_ReadDMA(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

	cDivider=16;

	accXYZ->accValueX = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0])/cDivider;
	accXYZ->accValueY = ((int16_t)((uint16_t)buffer[3] << 8) + buffer[2])/cDivider;
	accXYZ->accValueZ = ((int16_t)((uint16_t)buffer[5] << 8) + buffer[4])/cDivider;
}


//void LSM303DLHC_ACC_readValues(struct accValues *accXYZ)
//{
//  uint8_t ctrlx[2];
//  uint8_t buffer[6], cDivider;
//
//  /* Read the register content */
//  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
//  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);
//
//  if(ctrlx[1]&0x40)
//    cDivider=64;
//  else
//    cDivider=16;
//
//  /* check in the control register4 the data alignment*/
//  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
//  {
//	accXYZ->accValueX = ((int16_t)((uint16_t)buffer[1] << 8) + buffer[0])/cDivider;
//	accXYZ->accValueY = ((int16_t)((uint16_t)buffer[3] << 8) + buffer[2])/cDivider;
//	accXYZ->accValueZ = ((int16_t)((uint16_t)buffer[5] << 8) + buffer[4])/cDivider;
//  }
//  else /* Big Endian Mode */
//  {
//  	accXYZ->accValueX = ((int16_t)((uint16_t)buffer[0] << 8) + buffer[1])/cDivider;
//  	accXYZ->accValueY = ((int16_t)((uint16_t)buffer[2] << 8) + buffer[3])/cDivider;
//  	accXYZ->accValueZ = ((int16_t)((uint16_t)buffer[4] << 8) + buffer[5])/cDivider;
//  }
//}

#endif

void Mag_calibrate(void)
{
	uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;

	/* Configure MEMS: temp and Data rate */
	cra_regm |= (uint8_t) (LSM303DLHC_TEMPSENSOR_DISABLE | LSM303DLHC_ODR_30_HZ);

	/* Configure MEMS: full Scale */
	crb_regm |= (uint8_t) (LSM303DLHC_FS_8_1_GA);

	/* Configure MEMS: working mode */
	mr_regm |= (uint8_t) (LSM303DLHC_CONTINUOS_CONVERSION);

	/* Write value to Mag MEMS CRA_REG regsister */
	LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_regm);

	/* Write value to Mag MEMS CRB_REG regsister */
	LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_regm);

	/* Write value to Mag MEMS MR_REG regsister */
	LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_regm);
}


void Mag_readValues (struct magValues *magXYZ)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 6);
//  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
//  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
//  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
//  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
//  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  magXYZ->magValueX = (float)((int16_t)(((uint16_t)buffer[0] << 8) + buffer[1])*1000)/Magn_Sensitivity_XY;
  magXYZ->magValueZ = (float)((int16_t)(((uint16_t)buffer[2] << 8) + buffer[3])*1000)/Magn_Sensitivity_XY;
  magXYZ->magValueY = (float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}


