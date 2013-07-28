#ifndef ADXL345_Lib_H
#define ADXL345_Lib_H

#define ADXL345_I2C I2C1
#define ADXL345_I2C_ADDR 0x53<<1
#define ADXL345_X_ADDR 0x32

#define AVG_ROUNDS 10

#define X_OFFSET 12
#define Y_OFFSET 20

#ifdef ADXL345
struct accValues{
	int16_t accValueX;
	int16_t accValueY;
	int16_t accValueZ;

	float accGValue;
};

void ADXL345_AccInit(void);
void ADXL345_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer);
void ADXL345_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
void ADXL345_readValues(struct accValues *accXYZ);
void ADXL345_calibrate(float maxGOverflow, struct accValues *Acc_values);
void ACC_convert(uint8_t I2CRxBuffer[6], struct accValues *acc);
#endif

#endif /*ADXL345_Lib_H */


