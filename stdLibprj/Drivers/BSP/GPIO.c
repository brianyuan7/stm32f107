#include "gpio.h"

/*****************************************************************************
  * @brief  
  * @param  None
  * @retval None
  ***************************************************************************/
void gpio_init(void)
{
		GPIO_InitTypeDef     GPIO_InitStructure;    
//LED GPIO init
//LED1 PD2, LED2 PD3, LED3 PD4, LED4 PD7		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
		GPIO_Init(GPIOD, &GPIO_InitStructure);
//According to schimatic when LED pin outputs low, LED ON	
		GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
	
//config USART1 IO TX PA9, RX PA10
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		//Configure USART txpins AF output pull up pull down
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);       

			//Configure USART rx pins, floating input
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
		
}
