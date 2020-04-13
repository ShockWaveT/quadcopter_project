/**
 * @file 	uart_comm.c
 * @date 	21-Feb-2020
 * @author 	Arun Cheriyan
 */
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "uart_comm.h"


void USART1_IRQHandler()
{
	// Check if the USART receive interrupt flag was set
	if (USART_GetITStatus(USART1, USART_IT_RXNE))
	{
		USART_ReceiveData(USART1);
	}
}


/************************************************************************//*
 * Initializes the USART1 peripheral for console operation.
 *
 * @param baudRate: uart baud rate
 * @return none.
 **************************************************************************/
void uart_console_init(uint32_t baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable peripheral clocks for USART1 on GPIOA */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_AFIO, ENABLE);

	//----------------------------- interrupt testing ongoing ------------------------------

	/* NVIC Configuration */
	/* Enable the USARTx Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	//--------------------------------------------------------------------------------------


	/* Configure PA9 and PA10 as USART1 TX/RX */
	/* PA9 = alternate function push/pull output */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA10 = floating input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure and initialize usart... */
	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	//	Enable USART receive interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);
}
