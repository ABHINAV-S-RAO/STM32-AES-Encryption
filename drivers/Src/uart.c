/*
 * uart.c
 *
 *  Created on: Jun 27, 2025
 *      Author: abhin
 */
#include "uart.h"
#include <stdint.h>
#include "stm32f446xx.h"
#define UART2EN		(1U<<17)
#define GPIOAEN		(1U<<0)
#define SYS_FREQ	16000000//16Mhz
#define APB1_CLK	SYS_FREQ
#define CR1_TE		(1U<<3)//TE stands for transmitter enable, CR1 stands for Control Register 1
#define CR1_UE		(1U<<13)
#define SR_TXE		(1U<<7)// stands for Status Register--->transmit data register empty
#define UART_BAUDRATE	115200
#define CR1_RE		(1U<<2)
#define SR_RXNE		(1U<<5)


static void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t BaudRate);

void uart2_write(int ch);
void uart_tx_init(void)
{
	//******************Configure the UART GPIO pin************************
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR|=GPIOAEN;
	//Set PA2 to alternate function mode
	GPIOA->MODER|=(1U<<5);
    GPIOA->MODER&=~(1U<<4);
	/* Set PA2 alternate function to UART_TDX (AF07) */
    GPIOA->AFR[0]|=(1U<<8);//AFR is in form of array of 2 bytes. AFR[0] is for the AFRL register , and AFR[1] is for the AFRH register
    GPIOA->AFR[0]|=(1U<<9);
    GPIOA->AFR[0]|=(1U<<10);
    GPIOA->AFR[0]&=~(1U<<11);

	//******************Configure UART Module*****************
	//Enable clock access to UART2
    RCC->APB1ENR|=UART2EN;


	/*Configure UART baudrate*/
    uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);
	// Configure the transfer direction
    USART2->CR1=CR1_TE;// assigning operator used cuz everything else becomes auto 0 and thats what we want . also default stuff needed so we dont have to congfigure parity or start or stop bit

	  //Enable UART module
    USART2->CR1|=CR1_UE;

}
void uart2_rxtx_init(void)
{
	//******************Configure the UART GPIO pin************************
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR|=GPIOAEN;
	//Set PA2 to alternate function mode
	GPIOA->MODER|=(1U<<5);
    GPIOA->MODER&=~(1U<<4);
    /* Set PA2 alternate function to UART_TX (AF07) */
        GPIOA->AFR[0]|=(1U<<8);//AFR is in form of array of 2 bytes. AFR[0] is for the AFRL register , and AFR[1] is for the AFRH register
        GPIOA->AFR[0]|=(1U<<9);
        GPIOA->AFR[0]|=(1U<<10);
        GPIOA->AFR[0]&=~(1U<<11);

    //Set PA3 to alternate function mode
    	GPIOA->MODER|=(1U<<7);
        GPIOA->MODER&=~(1U<<6);


    /* Set PA3 alternate function to UART_RX (AF07) */
       	GPIOA->AFR[0]|=(1U<<12);//AFR is in form of array of 2 bytes. AFR[0] is for the AFRL register , and AFR[1] is for the AFRH register
        GPIOA->AFR[0]|=(1U<<13);
        GPIOA->AFR[0]|=(1U<<14);
        GPIOA->AFR[0]&=~(1U<<15);

	//******************Configure UART Module*****************
	//Enable clock access to UART2
    RCC->APB1ENR|=UART2EN;


	/*Configure UART baudrate*/
    uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);
	// Configure the transfer direction
    USART2->CR1=(CR1_TE|CR1_RE);// assigning operator used cuz everything else becomes auto 0 and thats what we want . also default stuff needed so we dont have to congfigure parity or start or stop bit
    //(CR1_TE|CR1_RE) will enable both the registers .just a short way to write instead of writing in 2 diff lines
	  //Enable UART module
    USART2->CR1|=CR1_UE;

}

int uart2_read_nonblocking(char *out) {
    if (USART2->SR & USART_SR_RXNE) {
        *out = USART2->DR;
        return 1;
    }
    return 0;
}
int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

char uart2_read(void)
{
	/*make sure the recieved data register is not empty*/
	while(!(USART2->SR & SR_RXNE)){}
	/*return data*/
	return USART2->DR;
}
void uart2_write(int ch)
{
	//Make sure the transmit data register is empty---> USART SR-- status register , bit 7 for
	while(!(USART2->SR & SR_TXE)){}//if both are not same then infinite loop . only when both are same the loop will not be accessed
	//Write to transmit data register
	USART2->DR=(ch & 0xFF);
}





static void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR=compute_uart_bd(PeriphClk,BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t BaudRate)
		{
			return((PeriphClk+(BaudRate/2U))/BaudRate);//equation for the baudrate value
		}
