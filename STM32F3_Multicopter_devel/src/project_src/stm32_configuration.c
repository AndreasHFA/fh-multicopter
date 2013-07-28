/*
 * stm32_configuration.c
 *
 *  Created on: Nov 16, 2012
 *      Author: franz
 */
#include <stm32_configuration.h>



void RCC_Configuration(void)
{
	/* Enable I2C1 periph */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1 |
							RCC_APB1Periph_USART2 |
							RCC_APB1Periph_USART3 |
							RCC_APB1Periph_TIM2 |
							RCC_APB1Periph_TIM3 |
							RCC_APB1Periph_TIM4, ENABLE);
	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 |
						   RCC_APB2Periph_SYSCFG, ENABLE);

	/* Enable Periph clock enable, SPI, GPIOs */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA |
						  RCC_AHBPeriph_GPIOB |
						  RCC_AHBPeriph_GPIOC |
						  RCC_AHBPeriph_GPIOD |
						  RCC_AHBPeriph_GPIOE |
						  RCC_AHBPeriph_ADC12 |
						  RCC_AHBPeriph_DMA1, ENABLE);

	/* Configure the ADC clock */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
}




void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
	  /* Set the Vector Table base location at 0x20000000 */
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;	// Je höher desto niedriger
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			// FreeRtos desto höher desto höher
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Config the USART3 Interrupt*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;  // wie war das noch mal mit den Interrupt prioritäten??
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void GPIO_Configuration(void)
{
	/************************GPIO***************************/
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PE14 and PE15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 |
								  GPIO_Pin_10 | GPIO_Pin_11 |
								  GPIO_Pin_12 | GPIO_Pin_13 |
								  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/************************END GPIO***************************/

	/****************************USART****************************/
	/* GPIOA Configuration:  USART2 TX on PA2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	// TX = PA2
	// RX = PA3
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);


	/* GPIOB Configuration:  USART3 RX on PB11 and TX on PB10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect USART3 pins to AF7  */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);


	/*************************END USART***************************/

	/******************************* SPI config ********************************/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure GPIO PIN for Lis Chip select */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	/******************************* END SPI ********************************/

	/****************************** I2C *************************************/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6 , GPIO_AF_4 );
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7 , GPIO_AF_4 );

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* I2C SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C SDA pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/****************************** END I2C **********************************/

	/****************************** TIM3 PWM OUT *****************************/
	/* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_2);
	/****************************** END TIM3 PWM OUT *************************/

	/****************************** TIM2 PWM OUT *****************************/
	/* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_2);
	/****************************** END TIM3 PWM OUT *************************/

	/* EXTI Lines */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* ADC Config */
	/* Configure ADC Channels as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	USART_Cmd(USART2, ENABLE); // enable USART2


	/*
	 * Configure USART 3
	 * */
	//USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

	USART_Cmd(USART3, ENABLE); // enable USART3
}


void SPI_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Configure the RX FIFO Threshold */
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE);
}


void I2C_Configuration(void)
{
	I2C_InitTypeDef  I2C_InitStructure;

	I2C_DeInit(I2C1);

    /* I2C configuration -------------------------------------------------------*/
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_Timing = 0x00902025;

    /* Apply LSM303DLHC_I2C configuration after enabling it */
    I2C_Init(I2C1, &I2C_InitStructure);

    /* LSM303DLHC_I2C Peripheral Enable */
    I2C_Cmd(I2C1, ENABLE);
}


void DMA_Configuration(uint8_t *buffer)
{
	DMA_DeInit(DMA1_Channel7);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->RXDR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)buffer;
    DMA_InitStructure.DMA_BufferSize         = 6;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);

    //DMA_Cmd(DMA1_Channel1, ENABLE);
}


void TIM2_PWM_Configuration(void)
{
	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	static TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / (50000*PRESCALEVALUE)) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}


void TIM3_PWM_Configuration(void)
{
	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	static TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / (50000*PRESCALEVALUE)) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 50*PRESCALEVALUE;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}


void TIM4_Configuration(void)
{
	uint16_t PrescalerValue = 0;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / (50000)) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* TIM3 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}


EXTI_InitTypeDef *getEXTI_Config(void)
{
	static EXTI_InitTypeDef   EXTI_InitStructure;
	return &EXTI_InitStructure;
}

void EXTI_Configuration(void)
{
	EXTI_InitTypeDef   *EXTI_InitStructure = getEXTI_Config();

	/* Connect EXTI0 Line to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource15);

	/* Configure EXTI0 line */
	EXTI_InitStructure->EXTI_Line = EXTI_Line12;
	EXTI_InitStructure->EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure->EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure->EXTI_LineCmd = ENABLE;
	EXTI_Init(EXTI_InitStructure);

	/* Configure EXTI0 line */
	EXTI_InitStructure->EXTI_Line = EXTI_Line13;
	EXTI_InitStructure->EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure->EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure->EXTI_LineCmd = ENABLE;
	EXTI_Init(EXTI_InitStructure);

	/* Configure EXTI0 line */
	EXTI_InitStructure->EXTI_Line = EXTI_Line14;
	EXTI_InitStructure->EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure->EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure->EXTI_LineCmd = ENABLE;
	EXTI_Init(EXTI_InitStructure);

	/* Configure EXTI0 line */
	EXTI_InitStructure->EXTI_Line = EXTI_Line15;
	EXTI_InitStructure->EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure->EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure->EXTI_LineCmd = ENABLE;
	EXTI_Init(EXTI_InitStructure);
}



void ADC_Configuration(void)
{
	__IO uint16_t  calibration_value = 0;

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InjectedInitTypeDef ADC_InjInitStructure;

	ADC_StructInit(&ADC_InitStructure);

	/* Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);

	/* Insert delay equal to 10 µs */
	Delay(10);

	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);

	while(ADC_GetCalibrationStatus(ADC2) != RESET );
	calibration_value = ADC_GetCalibrationValue(ADC2);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_InjSimul;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Enable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 3;
	ADC_Init(ADC2, &ADC_InitStructure);

	ADC_InjInitStructure.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
	ADC_InjInitStructure.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_None;
	ADC_InjInitStructure.ADC_NbrOfInjecChannel = 3;
	ADC_InjInitStructure.ADC_InjecSequence1 = ADC_Channel_5;
	ADC_InjInitStructure.ADC_InjecSequence2 = ADC_Channel_11;
	ADC_InjInitStructure.ADC_InjecSequence3 = ADC_Channel_12;
	ADC_InjectedInit(ADC2, &ADC_InjInitStructure);

	/* ADC1 regular channel7 configuration */
	//ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_5, ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_11,ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_12,ADC_SampleTime_7Cycles5);

	/* Enable ADC1 */
	ADC_Cmd(ADC2, ENABLE);

	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));

	/* Start ADC1 Software Conversion */
	ADC_StartConversion(ADC2);

}


