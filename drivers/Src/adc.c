#include "stm32f446xx.h"
#include"adc.h"
#define GPIOAEN					(1U<<0)
#define ADC1EN					(1U<<8)
#define ADC_CH1					(1U<<0)
#define ADC_SEQ_LEN_1			0x00
#define CR2_ADON				(1U<<0)
#define CR2_SWSTART				(1U<<30)
#define SR_EOC					(1U<<1)//EOC==end of conversion 1
/*adc congfigured with 3 channels
 * ch2,ch3,ch5
 * first=ch5 so go to SQ1 and in the 4 bits put 5 as binary which will be 0101
 * second=ch2  sq2---> 0010
 * third=ch3   sq3--->0011
 * */


//0B 0000 0001


void pa1_adc_init(void)
{
	//CONFIGURE THE ADC GPIO PIN

	//Enable clock access to GPIOA
	RCC->AHB1ENR|=GPIOAEN;
	//Set the mode of PA1 to analog in moder
	GPIOA->MODER|=(1U<<2);
	GPIOA->MODER|=(1U<<3);


	//CONFIGURE THE ADC MODULE

	//Enable clock access to ADC
	RCC->APB2ENR|=ADC1EN;

	//configure ADC parameters
	/*Conversion sequence starts*/
	ADC1->SQR3=ADC_CH1;

	/*Conversion sequence length*/
	ADC1->SQR1=ADC_SEQ_LEN_1;
	/*enable the ADC module*/
	ADC1->CR2|=CR2_ADON;
}

void start_conversion(void)
{
	/*Start ADC conversion*/
	ADC1->CR2|=CR2_SWSTART;
}


uint32_t adc_read(void)
{
	//Wait for conversion to be complete
	while(!(ADC1->SR & SR_EOC))
	{

	}//this while loop is where prgm gets stuck in case the SR_EOC is not equal to 1 , i.e it waits for it to become 1 before proceeding
	//Read converted result
	return(ADC1->DR);
}

