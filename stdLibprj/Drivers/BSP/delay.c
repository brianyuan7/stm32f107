#include "delay.h"

void delay_us(int nus)
{
		u32 temp;
//SYSTICK source is HCLK_Div8
		SysTick->LOAD = 72 / 8	*	nus; //Value to load into the SYST_CVR register when the counter is enabled and when it reaches 0 
		SysTick->VAL	=	0x00;//set current systick value to 0
		SysTick->CTRL	=	0x01;//enable systick counter		
		do
		{
			temp	=	SysTick->CTRL;//get current systick ctrl value, loop until systick counter to delay time
		}while((temp & 0x01) && (!(temp & (1<<16))));//bit1 = 1, systick enable, bit16 = 1; systick count to 0
				 
		SysTick->CTRL	=	0x00; //disable systick counter		
		SysTick->VAL =	0x00; //clear counter
}

void delay_ms(int nms)
{
	delay_us(nms * 1000);
}

