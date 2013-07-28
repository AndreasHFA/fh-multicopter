/*
 * L3GD20_Lib_SPI.h
 *
 *  Created on: Nov 17, 2012
 *      Author: franz
 */

#ifndef L3GD20_LIB_SPI_H_
#define L3GD20_LIB_SPI_H_

#include <stm32f30x.h>
#include <helperFunctions.h>

#define L3GD20_SPI SPI1

//#define GYROCONVERT 0.017499987f
#define GYROCONVERT 0.07
//#define GYROCONVERT 0.0175

#define average_loop_value 300


#define L3GD20_CS_LOW()       GPIO_ResetBits(GPIOE, GPIO_Pin_3)
#define L3GD20_CS_HIGH()      GPIO_SetBits(GPIOE, GPIO_Pin_3)

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)
#define L3GD20_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_4    ((uint8_t)0xC0)

#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)

#define L3GD20_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GD20_BlockDataUpdate_Single      ((uint8_t)0x80)

#define L3GD20_BLE_LSB                     ((uint8_t)0x00)
#define L3GD20_BLE_MSB	                   ((uint8_t)0x40)

#define L3GD20_FULLSCALE_250               ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500               ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000              ((uint8_t)0x20)

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/ */


struct gyroValues{
	int16_t gyroValueX;
	int16_t gyroValueY;
	int16_t gyroValueZ;
};

void Gyro_calibrate(struct gyroValues *gyroOffset);
void Gyro_readValues (struct gyroValues *gyro);
//uint8_t Gyro_Test(struct gyroValues *gyroOffset);
//void Gyro_convert(uint8_t I2CRxBuffer[6], struct gyroValues *gyro);
//void L3G4200D_Write(u8 I2C_Address, u8 Register_Address, u8 Data );
//void L3G4200D_SetRegister(u8 I2C_Address, u8 Register_Address );
//void L3G4200D_BurstRead(u8 I2C_Address, struct gyroValues *L3G4200D);


#endif /* L3GD20_LIB_SPI_H_ */
