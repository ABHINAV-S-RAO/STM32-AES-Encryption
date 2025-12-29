#include "stm32f446xx.h"
#define SYSTICK_LOAD_VAL			16000//default is 16Mhz meaning 16000 cycles in a single millisecond
#define CTRL_ENABLE					(1U<<0)
#define CTRL_CLKSRC					(1U<<2)
#define CTRL_COUNTFLAG				(1U<<16)


void systickDelayMs(int delay)
{
	/*COnfiure systick*/
	//Reload with number of clocks per millisecond
	SysTick->LOAD=SYSTICK_LOAD_VAL;

	//Clear the systick current value register
	SysTick->VAL=0;
	//Enable systick and select internal clk source
	SysTick->CTRL=CTRL_ENABLE|CTRL_CLKSRC;
	for(int i=0;i<delay;i++)
	{
		//Wait until the COUNT FLAG IS SET
		while((SysTick->CTRL & CTRL_COUNTFLAG)==0){}

	}
	SysTick->CTRL=0;
}
