/*
 * i2c_comm.c
 *
 * Created on: 09-Feb-2020
 * Author: Arun
 */
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "timer_config.h"
#include "i2c_comm.h"
#include "small_printf.h"


/**
 * Initializes the I2C peripheral.
 *
 * @param ClockSpeed Bus communication frequency in Hz
 * @param OwnAddress MCU I2C address
 * @return none.
 */
void I2C_LowLevel_Init(uint32_t ClockSpeed, uint8_t OwnAddress)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);


	/* Configure I2C_EE pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = OwnAddress;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);
}

/**
 * reads a register of the slave device. only works on slave devices
 * with register addresses not longer than 8 bits.
 *
 * @param slaveAddre slave address.
 * @param registerAddr adress of the slave register to be read.
 * @return none.
 */
uint8_t i2c_register_read(uint8_t slaveAddr, uint8_t registerAddr)
{
	uint8_t data;

	timeout_alarm_set(5);

	I2C_AcknowledgeConfig(I2C1,ENABLE);

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(timeout_alarm_status_check() == 1)
			return 0;
	}
	//uart_printf("aa\n");

	I2C_Send7bitAddress(I2C1, (slaveAddr<<1), I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(timeout_alarm_status_check() == 1)
			return 0;
	}
	//uart_printf("bb\n");


	I2C_SendData(I2C1, registerAddr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(timeout_alarm_status_check() == 1)
			return 0;
	}
	//uart_printf("cc\n");

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(timeout_alarm_status_check() == 1)
			return 0;
	}
	//uart_printf("dd\n");

	I2C_Send7bitAddress(I2C1, (slaveAddr<<1), I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(timeout_alarm_status_check() == 1)
			return 0;
	}
	//uart_printf("ee\n");

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		if(timeout_alarm_status_check() == 1)
			return 0;
	}

	I2C_AcknowledgeConfig(I2C1,DISABLE);
	data = I2C_ReceiveData(I2C1);
	//uart_printf("ff\n");

	I2C_GenerateSTOP(I2C1,ENABLE);
	//uart_printf("gg\n");

	timeout_alarm_off();

	return data;
}

/**
 * Reads data from the slave.
 *
 * @param slaveAddr slave address.
 * @param registerAddr starting memory location of slave to start reading from.
 * @param writeBuffer pointer to the buffer to hold the read data.
 * @param bytesNum number of bytes to read from slave.
 * @return 0 on success, -1 on timeout.
 *
 */
int8_t i2c_slave_mem_read(uint8_t slaveAddr, uint8_t registerAddr, uint8_t* writeBuffer, uint8_t bytesNum)
{
	uint8_t i;

	i = bytesNum;
	timeout_alarm_set(5*bytesNum);

	I2C_AcknowledgeConfig(I2C1,ENABLE);

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(timeout_alarm_status_check() == 1)
			return -1;
	}
	//uart_printf("aa\n");

	I2C_Send7bitAddress(I2C1, (slaveAddr<<1), I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(timeout_alarm_status_check() == 1)
			return -1;
	}
	//uart_printf("bb\n");


	I2C_SendData(I2C1, registerAddr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(timeout_alarm_status_check() == 1)
			return -1;
	}
	//uart_printf("cc\n");

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(timeout_alarm_status_check() == 1)
			return -1;
	}
	//uart_printf("dd\n");

	I2C_Send7bitAddress(I2C1, slaveAddr<<1, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if(timeout_alarm_status_check() == 1)
			return -1;
	}
	//uart_printf("ee\n");

	while(i>0)
	{
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			if(timeout_alarm_status_check() == 1)
				return -1;
		}
		if(i==1)
			I2C_AcknowledgeConfig(I2C1,DISABLE);
		writeBuffer[bytesNum-i] = I2C_ReceiveData(I2C1);
		i--;
		//uart_printf("ff\n");
	}
	I2C_GenerateSTOP(I2C1,ENABLE);
	//uart_printf("gg\n");
	timeout_alarm_off();
	return 0;
}

/**
 * write data to slave registers/memory.
 *
 * @param slaveAddr slave address.
 * @param registerAddr starting memory location of slave to start writing to.
 * @param writedata data to be written to the slave register.
 * @return 0 on success, -1 on timeout.
 *
 */
int8_t i2c_slave_mem_write(uint8_t slaveAddr, uint8_t registerAddr, uint8_t writeData)
{
	timeout_alarm_set(20);

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
			if(timeout_alarm_status_check() == 1)
				return -1;
	}

	I2C_Send7bitAddress(I2C1, (slaveAddr<<1), I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
			if(timeout_alarm_status_check() == 1)
				return -1;
	}

	I2C_SendData(I2C1,registerAddr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
			if(timeout_alarm_status_check() == 1)
				return -1;
	}

	I2C_SendData(I2C1,writeData);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
			if(timeout_alarm_status_check() == 1)
				return -1;
	}

	I2C_GenerateSTOP(I2C1,ENABLE);

	return 0;
}
