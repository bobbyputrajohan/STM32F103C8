#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void InitGPIOC()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef test;
	test.GPIO_Mode = GPIO_Mode_Out_PP;
	test.GPIO_Pin = GPIO_Pin_13;
	test.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&test);
}

void InitUsart2()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA10 = floating input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef usart;
	usart.USART_BaudRate = 19200;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &usart);
	USART_Cmd(USART2,ENABLE);
}

void InitPwm()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 999 ;
	TIM_BaseStruct.TIM_Prescaler = 71 ;
	TIM_BaseStruct.TIM_RepetitionCounter = 0 ;
	TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);


	TIM_OCInitTypeDef TIM_OCStruct;
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void InitSPI()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	GPIO_InitTypeDef GPIO;
	GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO);

	GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO);

	SPI_InitTypeDef InitSPI;
	InitSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	InitSPI.SPI_CPHA = SPI_CPHA_2Edge;
	InitSPI.SPI_CPOL = SPI_CPOL_High;
	InitSPI.SPI_DataSize = SPI_DataSize_8b;
	InitSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	InitSPI.SPI_FirstBit = SPI_FirstBit_MSB;
	InitSPI.SPI_Mode = SPI_Mode_Master;
	InitSPI.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_Init(SPI1,&InitSPI);
	SPI_Cmd(SPI1, ENABLE);
}

void InitI2C()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	GPIO_InitTypeDef GPIO;
	GPIO.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO);

	I2C_InitTypeDef Test;
	Test.I2C_Ack = I2C_Ack_Enable;
	Test.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	Test.I2C_ClockSpeed = 100000;
	Test.I2C_DutyCycle = I2C_DutyCycle_16_9;
	Test.I2C_Mode = I2C_Mode_I2C;
	Test.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C1, &Test);
	I2C_Cmd(I2C1, ENABLE);
}

void InitUsart()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	    /* PA10 = floating input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef usart;
	usart.USART_BaudRate = 9600;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &usart);
	USART_Cmd(USART1,ENABLE);
}

int sendData(uint8_t data)
{
	USART_SendData(USART1,data);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return data;
}

int getData()
{
	uint8_t datarx;
	while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==RESET);
	datarx=USART_ReceiveData(USART1);
	return datarx;
}

uint8_t transceive(uint8_t* txData,uint32_t txDataLen, uint8_t* rxBuffer)
{
    for(int i=0; i<txDataLen; i++)
    {
        while(!((SPI1)->SR & SPI_SR_TXE));
        SPI1->DR = txData[i];
        while(!((SPI1)->SR & SPI_SR_RXNE));
        rxBuffer[i] = SPI1->DR;
    }
}

void i2c_start()
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
}

void i2c_stop()
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

void i2CWrite(uint8_t devAddr, uint8_t regAddr, uint8_t* dataOut, uint32_t dataLen)
{
	I2C_AcknowledgeConfig(I2C1, ENABLE); //ack
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
   	I2C_Send7bitAddress(I2C1, devAddr << 1,I2C_Direction_Transmitter); // address
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // ev6
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2C1, regAddr); //data
    for(int i = 0; i < dataLen; i++)
    {
    	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
        I2C_SendData(I2C1, dataOut[i]); //data

    }
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_AcknowledgeConfig(I2C1, DISABLE); //ack
}

uint32_t i2Cread(uint8_t devAddr,uint8_t regAddr, uint8_t* dataIn, uint32_t maxDataLen)
{
	I2C_AcknowledgeConfig(I2C1, ENABLE); //ack
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); //ev5
	I2C_Send7bitAddress(I2C1, devAddr << 1,I2C_Direction_Transmitter); // address
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // ev6
	I2C_SendData(I2C1, regAddr); //data
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //ev8_2
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//ev5
    I2C_Send7bitAddress(I2C1, devAddr << 1 ,I2C_Direction_Receiver);//add
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//ev6
    for(int i = 0; i<maxDataLen; i++)
    {
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); //ev7
        dataIn[i] = I2C_ReceiveData(I2C1);
    }
	I2C_AcknowledgeConfig(I2C1, DISABLE);
}


int main()
{
	//SystemCoreClock
	SystemInit();
	InitGPIOC();
	InitUsart();
	InitUsart2();
	InitPwm();
	InitSPI();
	InitI2C();
	sendData(65);

	uint8_t rx[2], tx[2], data;
	tx[0] = 0x8F;
	tx[1] = 0x00;
	GPIOA -> BRR = GPIO_Pin_4; //mati chip select untuk SPI
	transceive(tx,2,rx); //write data using SPI and read data using SPI
	data = rx[1];
	GPIOA -> BSRR = GPIO_Pin_4; // hidup chip select untuk SPI


	uint8_t rx1[100];
	uint8_t tx1[100];
	tx1[0] = 0x04;
	tx1[1] = 0x05;
	tx1[2] = 0x06;
	i2c_start(); //start communication I2c
	i2CWrite(0x69,0xA0,tx1,3); // write data in I2C Communication
	i2c_stop(); //stop communication I2c
	while(1)
	{

		TIM1->CCR1 = 50;	//PWM
		sendData(rx1[0]);	//Send data using Usart
		sendData(rx1[1]);
		sendData(rx1[2]);
		i2c_start();
		i2Cread(0x69,0xA0,rx1,3);
		i2c_stop();

	}
}
