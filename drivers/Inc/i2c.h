
#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#define FLAG_SET 	1
#define SET			1
#define RESET		0
#define FLAG_RESET	0
#define __IO volatile
typedef struct
{
  __IO uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  __IO uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  __IO uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  __IO uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  __IO uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  __IO uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  __IO uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  __IO uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  __IO uint32_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
}I2C_RegDef_t;
/*
 * Config struct for I2Cx peripheral
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))

#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))

#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * I2C CLK disable
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &=~ (1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &=~ (1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &=~ (1 << 23))

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t	 I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t		*pTxBuffer;
	uint8_t		*pRxBuffer;
	uint32_t	TxLen;
	uint32_t	RxLen;
	uint8_t		TxRxState;
	uint8_t		DevAddr;
	uint32_t	RxSize;
	uint8_t		Sr;
}I2C_Handle_t;

#define I2C1                ((I2C_RegDef_t *) I2C1_BASE)
#define I2C2                ((I2C_RegDef_t *) I2C2_BASE)
#define I2C3                ((I2C_RegDef_t *) I2C3_BASE)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
/*
 * Modes for I2C_SCLSpeed
 */


#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

/*
 * I2C ACK Control
 */

#define I2C_ACK_ENABLE		ENABLE
#define I2C_ACK_DISABLE		DISABLE

/*
 * I2C Duty cycle
 */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C flags
 */

#define I2C_FLAG_TXE   		( I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( I2C_SR1_RXNE)
#define I2C_FLAG_SB			( I2C_SR1_SB)
#define I2C_FLAG_OVR  		( I2C_SR1_OVR)
#define I2C_FLAG_AF   		( I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

/*
 * I2C Application states
 */

#define I2C_READY 			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2


#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR			3
#define I2C_ERROR_ARLO			4
#define I2C_ERROR_AF				5
#define I2C_ERROR_OVR				6
#define I2C_ERROR_TIMEOUT			7
#define I2C_EV_DATA_REQ				8
#define I2C_EV_DATA_RCV				9

//***INCLUDING CMSIS HEADER FILE for mystm32**


/***********************************************************************************************************
 * 												APIs for I2C protocol
 *
 * *********************************************************************************************************
 */


/*
 * Peripheral Clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * Init and De-Init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_Deinit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and recieve
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);//these return the application status
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
void I2C_SlaveReceiveData(I2C_RegDef_t *pI2C, uint8_t *rxBuf, uint32_t len);

/*
 * IRQ and ISR config
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * other APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi );
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void  I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint32_t RCC_GetPCLK1Value(void);


/*
 * Application Callbacks
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);








#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
