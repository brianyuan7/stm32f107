#include "uart.h"

/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
  ***************************************************************************/

void uart_init(int baud)
{
		USART_InitTypeDef  USART_InitStruct;
		NVIC_InitTypeDef   NVIC_InitStructure;
		/* Enable UART clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		USART_InitStruct.USART_BaudRate = baud;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		

		NVIC_InitStructure.NVIC_IRQChannel= USART1_IRQn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;  
		NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;  
		NVIC_Init(&NVIC_InitStructure);		
		/* Connect PXx to USARTx_Tx */
		//GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
		//GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

		/* Connect PXx to USARTx_Rx */
		//GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
		/* USART configuration */
		USART_Init(USART1, &USART_InitStruct);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //enable before UART enable
				 
		/* Enable USART */
		USART_Cmd(USART1, ENABLE);
}
