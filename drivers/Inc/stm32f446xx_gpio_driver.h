/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jul 25, 2025
 *      Author: abhin
 */
#define __IO volatile
#ifndef STM32F446XX_GPIO_DRIVER_H_
#define STM32F446XX_GPIO_DRIVER_H_
#define GPIO_TypeDef GPIO_RegDef_t
#include "stm32f446xx.h"
#define GPIO_BASEADDR_TO_CODE(x)	((x==GPIOA)?0:\
									(x==GPIOB)?1:\
									(x==GPIOC)?2:\
									(x==GPIOD)?3:\
									(x==GPIOE)?4:\
									(x==GPIOF)?5:\
									(x==GPIOG)?6:\
									(x==GPIOH)?7:0)

#define NO_OF_PRIORITY_BITS_IMPLEMENTED		4//in case of ST

/*PROCESSOR (ARM CORTEX M4) SPECIFIC HEADER MACRO DEFINITIONS
 */
//ARM Cortex Mx Processor NVIC ISERx register address
#define NVIC_ISER0				((__IO uint32_t*)0xE000E100)
#define NVIC_ISER1				((__IO uint32_t*)0xE000E104)//ISER0 + 4 bytes
#define NVIC_ISER2				((__IO uint32_t*)0xE000E108)
#define NVIC_ISER2				((__IO uint32_t*)0xE000E10C)

#define NVIC_ICER0				((__IO uint32_t*)0xE000E180)
#define NVIC_ICER1				((__IO uint32_t*)0xE000E184)
#define NVIC_ICER2				((__IO uint32_t*)0xE000E188)


#define NVIC_PR_BASE_ADDR                       ((__IO uint32_t*)0xE000E400)

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
}GPIO_TypeDef;
typedef struct
{
	uint8_t GPIO_PinNumber;/*possible pin numbers from @GPIO_PIN_NUMBER>*/
	uint8_t GPIO_PinMode; /*possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;/*possible output speeds @GPIO_OUTPUT_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct//this one contains the base address pf the gpio port and also the gpio pin config within that port
{
	GPIO_TypeDef *pGPIOx;  //Holds the base address of the GPIO port to which the pin belongs (p is a pointer type variable)
	GPIO_PinConfig_t GPIO_PinConfig;//This holds GPIO pin configuration settings

}GPIO_Handle_t;

/**********************************************************************/
/* ***************APIs supported by this drier ************************/
/**********************************************************************/
//Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);//by using this function , we can enable/disable the peri clock for a given gpio base address

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);//takes pointer to the Handle and thats it. user shld create a variable of "GPIO_Handle_t" type and send its pointer to this function
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);//should just take base address of the peripheral , so that it can use RCC-peripheral reset registers

//Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);//it will either be high/low so we use uint8_t return type
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);//16 pins so uint16_t is being used as return type
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);//no return value. Value=0 or 1
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);//no return value. uint16_t here cuz 16 pins in each port and 0 or 1
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR Handling- related to interrupts
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);//whenever an interrupt is triggered then this API can be called. irq handling shld know which pin the interrupt is generated

void GPIOA_PCLK_EN(void);
void GPIOB_PCLK_EN(void);
void GPIOC_PCLK_EN(void);
void GPIOD_PCLK_EN(void);
void GPIOE_PCLK_EN(void);
void GPIOF_PCLK_EN(void);
void GPIOG_PCLK_EN(void);
void GPIOH_PCLK_EN(void);

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin possible nnumbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15
#define GPIO_PIN_NO_16		16


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4 //interrupt mode
#define GPIO_MODE_IT_RT		5 //interrupt mode
#define GPIO_MODE_IT_RFT	6 //interrupt mode


/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * GPIO pin possible output speeds
 * @GPIO_OUTPUT_SPEEDS
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/*Macros to reset GPIOx peripherals */
#define GPIOA_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<0); RCC->AHB1RSTR&=~(1U<<0);}while(0)//a do while loop is used cuz we have to perform both set and reset
#define GPIOB_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<1); RCC->AHB1RSTR&=~(1U<<1);}while(0)//while using a single macro definition
#define GPIOC_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<2); RCC->AHB1RSTR&=~(1U<<2);}while(0)
#define GPIOD_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<3); RCC->AHB1RSTR&=~(1U<<3);}while(0)
#define GPIOE_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<4); RCC->AHB1RSTR&=~(1U<<4);}while(0)
#define GPIOF_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<5); RCC->AHB1RSTR&=~(1U<<5);}while(0)
#define GPIOG_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<6); RCC->AHB1RSTR&=~(1U<<6);}while(0)
#define GPIOH_REG_RESET()		do{RCC->AHB1RSTR|=(1U<<7); RCC->AHB1RSTR&=~(1U<<7);}while(0)

#endif /* STM32F446XX_GPIO_DRIVER_H_ */
