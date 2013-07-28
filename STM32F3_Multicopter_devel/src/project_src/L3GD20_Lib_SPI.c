/*
 * L3GD20_Lib_SPI.c
 *
 *  Created on: Nov 17, 2012
 *      Author: franz
 */

#include <L3GD20_Lib_SPI.h>

static uint8_t L3GD20_SendByte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
  }

  /* Send a Byte through the SPI peripheral */
  SPI_SendData8(L3GD20_SPI, byte);

  /* Wait to receive a Byte */
  while (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }

  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_ReceiveData8(L3GD20_SPI);
}



void L3GD20_Init(void)
{
  uint8_t ctrl1 = 0x00, ctrl2 = 0x00, ctrl5 = 0x00, ctrl4 = 0x00;

  /* Configure the low level interface ---------------------------------------*/
  //L3GD20_LowLevel_Init();

  /* Configure MEMS: data rate, power mode, full scale and axes */
//  ctrl1 = (uint8_t) (L3GD20_MODE_ACTIVE | L3GD20_OUTPUT_DATARATE_4  |
//		  	  	  	  L3GD20_AXES_ENABLE | L3GD20_BANDWIDTH_2);

  ctrl1 = (uint8_t) (L3GD20_MODE_ACTIVE | L3GD20_OUTPUT_DATARATE_4  |
		  	  	  	  L3GD20_AXES_ENABLE | L3GD20_BANDWIDTH_4);

  ctrl2 = (uint8_t) (0x34); /* CTRL_REG2   HPF Setup HPF(0,5Hz-4Hz) Autoreset */

  ctrl4 = (uint8_t) (L3GD20_BlockDataUpdate_Single  | L3GD20_BLE_LSB |
		  	  	  	  L3GD20_FULLSCALE_2000);

  ctrl5 = (uint8_t) (0x13); /* CTRL_REG5   low pass filter enable, high pass filter enabled */


  /* Write value to MEMS CTRL_REG1 regsister */
  L3GD20_CS_LOW();
  L3GD20_SendByte(L3GD20_CTRL_REG1_ADDR);
  L3GD20_SendByte(ctrl1);
  L3GD20_CS_HIGH();

  /* Write value to MEMS CTRL_REG2 regsister */
  L3GD20_CS_LOW();
  L3GD20_SendByte(L3GD20_CTRL_REG2_ADDR);
  L3GD20_SendByte(ctrl2);
  L3GD20_CS_HIGH();

  /* Write value to MEMS CTRL_REG4 regsister */
  L3GD20_CS_LOW();
  L3GD20_SendByte(L3GD20_CTRL_REG4_ADDR);
  L3GD20_SendByte(ctrl4);
  L3GD20_CS_HIGH();

  /* Write value to MEMS CTRL_REG2 regsister */
  L3GD20_CS_LOW();
  L3GD20_SendByte(L3GD20_CTRL_REG5_ADDR);
  L3GD20_SendByte(ctrl5);
  L3GD20_CS_HIGH();
}


void L3GD20_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  L3GD20_CS_LOW();

  /* Send the Address of the indexed register */
  L3GD20_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device) */
    *pBuffer = L3GD20_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  L3GD20_CS_HIGH();
}


void Gyro_convert(uint8_t Buffer[6], struct gyroValues *gyro, uint8_t bigEndian)
{
	if(bigEndian)
	{
		gyro->gyroValueX = (Buffer[1] << 8) | (Buffer[0]);
		gyro->gyroValueY = (Buffer[3] << 8) | (Buffer[2]);
		gyro->gyroValueZ = (Buffer[5] << 8) | (Buffer[4]);
	}
	else
	{
		gyro->gyroValueX = (Buffer[0] << 8) | (Buffer[1]);
		gyro->gyroValueY = (Buffer[2] << 8) | (Buffer[3]);
		gyro->gyroValueZ = (Buffer[4] << 8) | (Buffer[5]);
	}
}


void Gyro_readValues (struct gyroValues *gyro)
{
  uint8_t tmpbuffer[6] ={0};
  uint8_t tmpreg = 0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
	Gyro_convert(&tmpbuffer[0], gyro, 1);
  }
  else
  {
   	Gyro_convert(&tmpbuffer[0], gyro, 0);
  }
}



//ACC und GYRO kalibrieren:
void Gyro_calibrate(struct gyroValues *gyroOffset)
{
int i = 0;
int32_t tmpX = 0;
int32_t tmpY = 0;
int32_t tmpZ = 0;

L3GD20_Init();

//discard the first 100 values
for(i=0;i<100;i++)
{
	Gyro_readValues(gyroOffset);
}

gyroOffset->gyroValueX = 0;
gyroOffset->gyroValueY = 0;
gyroOffset->gyroValueZ = 0;

/***************Calibrate the Gyro***************/
for(i=0;i<average_loop_value;i++)
{

	Gyro_readValues(gyroOffset);
	tmpX += gyroOffset->gyroValueX;
	tmpY += gyroOffset->gyroValueY;
	tmpZ += gyroOffset->gyroValueZ;
}

gyroOffset->gyroValueX = tmpX/average_loop_value;
gyroOffset->gyroValueY = tmpY/average_loop_value;
gyroOffset->gyroValueZ = tmpZ/average_loop_value;
}

