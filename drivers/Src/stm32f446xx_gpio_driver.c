/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jul 25, 2025
 *      Author: abhin
 */
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define GPIO_MODE_ANALOG		3
#define ENABLE 1
#define DISABLE	0
//DOCUMENT SECTION PRESENT FOR EACH API
/**************************************************************************************
 * @fn			-
 *
 * @brief 		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return 		-
 *
 * @Note		-
 */

/**************************************************************************************
 * @fn			- 	GPIO_PeriClockControl
 *
 * @brief 		- 	This function enables or disables Peripheral clock for the given GPIO port
 *
 * @param[in]	- 	pointer to Base addres of the GPIO peripheral
 * @param[in]	-	Enable or disable macros
 * @param[in]	-
 *
 * @return 		- 	none
 *
 * @Note		- 	None
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)//by using this function , we can enable/disable the peri clock for a given gpio base address
{
	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx==GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx==GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_DI();
		}

	}
}
//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)//takes pointer to the Handle and thats it. user shld create a variable of "GPIO_Handle_t" type and send its pointer to this function
{
	uint32_t temp=0;
	//1.configure the mode of the gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
	    uint32_t pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	    uint32_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;

	    // Clear the 2 bits for that pin
	    pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pin));

	    // Set the desired mode
	    pGPIOHandle->pGPIOx->MODER |= (mode << (2 * pin));
	}
	else
	{
		//this will be interrupt mode (coded later)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)//falling edge detection
		{
			//1.Configure the FTSR
			EXTI->FTSR|=(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the RTSR bit
			EXTI->RTSR&=~(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)//rising edge detection
		{
			//1.Configure the RTSR
			EXTI->RTSR|=(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing the corresp FTSR
			EXTI->FTSR&=~(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)//both rising and falling
		{
			//1.Configure the FTSR AND RTSR
			EXTI->RTSR|=(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR|=(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2.Configure the GPIO port selection in SYSCFG_EXTICR
		int8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;//to get to know which EXTIx to use[0],[1],[2],[3]
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCK_EN();
		SYSCFG->EXTICR[temp1]=portcode<<(temp2*4);

		//3.Enable the exti interrupt delivery using IMR
		EXTI->IMR|=(1U<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;
	//2.configure the speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;
	//3.configure the pupd setting
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->PUPDR|=temp;
	temp=0;
	//4.configure the otype
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//each pin occupies a single bit so multiplying by 2 is not required
	pGPIOHandle->pGPIOx->OTYPER&=~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIOx->OTYPER|=temp;
	temp=0;
	//5.configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers
		uint32_t temp1,temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1]&=~(0xF<<(4*temp2));//clearing
		pGPIOHandle->pGPIOx->AFR[temp1]|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));

	}
}
//DOCUMENT SECTION PRESENT FOR EACH API
/**************************************************************************************
 * @fn			- GPIO deinitialize
 *
 * @brief 		- used to set and reset the gpio ports immediately
 *
 * @param[in]	- pointer to GPIOx peripheral
 * @param[in]	-
 * @param[in]	-
 *
 * @return 		- void
 *
 * @Note		- we have defined special kind of macros for this using do while loop so that 2 instructions are defined in a single macro defn
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)//should just take base address of the peripheral , so that it can use RCC-peripheral reset registers
{
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}
}

void SYSCFG_PCK_EN(void)
{
	RCC->APB2ENR|=(1U<<14);
}

//Data Read and Write
//DOCUMENT SECTION PRESENT FOR EACH API
/**************************************************************************************
 * @fn			-
 *
 * @brief 		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return 		-
 *
 * @Note		-
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//it will either be high/low so we use uint8_t return type
	uint8_t value;
	value=((pGPIOx->IDR >> PinNumber)& 0x00000001);//right shifting the value in pin no. x so that it comes to pin 0 and anding with 1
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	//16 pins so uint16_t is being used as return type
	uint16_t value;
	value=pGPIOx->IDR;//right shifting the value in pin no. x so that it comes to pin 0 and anding with 1
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)//Value=0 or 1

{
	if (Value)
	    {
	        pGPIOx->BSRR = (1 << PinNumber);           // Set pin
	    }
	    else
	    {
	        pGPIOx->BSRR = (1 << (PinNumber + 16));    // Reset pin
	    }
}
	//no return value.
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	//no return value. uint16_t here cuz 16 pins in each port and 0 or 1
	pGPIOx->ODR=Value;//just copy the value into ODR cuz we are copying for whole port
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR^=(1U<<PinNumber);
}



//IRQ Configuration and ISR Handling- related to interrupts
//void GPIO_IRQConfig(uint IRQNumber,uint8_t IRQPriority,uint8_t EnorDi)

void GPIO_IRQHandling(uint8_t PinNumber)//whenever an interrupt is triggered then this API can be called. irq handling shld know which pin the interrupt is generated
{
	//clear the Exti Pr register corresponding to pin number
	if(EXTI->PR & (1U<<PinNumber))
	{
		//clear
		EXTI->PR|=(1U<<PinNumber);//writing 1 is the way to clear it
	}
}
void GPIOA_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 0); // Set bit 0 for GPIOA
}

void GPIOB_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 1); // Set bit 1 for GPIOB
}

void GPIOC_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 2); // Set bit 2 for GPIOC
}

void GPIOD_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 3); // Set bit 3 for GPIOD
}

void GPIOE_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 4); // Set bit 4 for GPIOE
}

void GPIOF_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 5); // Set bit 5 for GPIOF
}

void GPIOG_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 6); // Set bit 6 for GPIOG
}

void GPIOH_PCLK_EN(void)
{
	RCC->AHB1ENR |= (1 << 7); // Set bit 7 for GPIOH
}

//DISABLE APIs
void GPIOA_PCLK_DI(void)
{
	RCC->AHB1ENR &=~ (1 << 0); // Set bit 0 for GPIOA
}

void GPIOB_PCLK_DI(void)
{
	RCC->AHB1ENR &=~ (1 << 1); // Set bit 1 for GPIOB
}

void GPIOC_PCLK_DI(void)
{
	RCC->AHB1ENR &=~ (1 << 2); // Set bit 2 for GPIOC
}

void GPIOD_PCLK_DI(void)
{
	RCC->AHB1ENR &=~ (1 << 3); // Set bit 3 for GPIOD
}

void GPIOE_PCLK_DI(void)
{
	RCC->AHB1ENR &=~(1 << 4); // Set bit 4 for GPIOE
}

void GPIOF_PCLK_DI(void)
{
	RCC->AHB1ENR &=~ (1 << 5); // Set bit 5 for GPIOF
}

void GPIOG_PCLK_DI(void)
{
	RCC->AHB1ENR &=~(1 << 6); // Set bit 6 for GPIOG
}

void GPIOH_PCLK_DI(void)
{
	RCC->AHB1ENR &=~ (1 << 7); // Set bit 7 for GPIOH
}




//Interrupt COnfigurations

void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi)
{

	if(EnorDi==ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*(NVIC_ISER0)|=(1U<<IRQNumber);
		}
		else if(IRQNumber >=32 && IRQNumber<64)
		{
			//program ISER1 register
			*(NVIC_ISER0)|=(1U<<(IRQNumber)%32);
		}
		else if(IRQNumber >=64 && IRQNumber<96)
		{
			//program ISER2 Register
			*(NVIC_ISER0)|=(1U<<(IRQNumber)%32);
		}

	}
	else
	{
		if(IRQNumber<=31)
		{
			//program ICER0 register
			*(NVIC_ICER0)|=(1U<<IRQNumber);
		}
		else if(IRQNumber >=32 && IRQNumber <64)
		{
			//program ICER1 register
			*(NVIC_ICER0)|=(1U<<(IRQNumber)%32);
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			//program ICER2 register
			*(NVIC_ICER0)|=(1U<<(IRQNumber)%32);
		}
	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx=IRQNumber/4;//finding out the IPR register number
	uint8_t iprx_section=IRQNumber%4;//finding out which section of the register 0,1,2 or 3
	uint8_t shift_amount=(8*iprx_section)+ (8*NO_OF_PRIORITY_BITS_IMPLEMENTED);
	*((NVIC_PR_BASE_ADDR)+iprx*4)|=(IRQPriority<<shift_amount);
}
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

