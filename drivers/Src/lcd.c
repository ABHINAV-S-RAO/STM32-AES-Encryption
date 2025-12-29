/*
 * lcd.c
 *
 *  Created on: Jul 26, 2025
 *      Author: abhin
 */
#define GPIO_PIN_RESET 		0
#define GPIO_PIN_SET		1
#include "stm32f446xx.h"
#include "lcd.h"
#include "stm32f446xx_gpio_driver.h"


static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);


static void mdelay(uint32_t cnt)//millisecond delay
{
	for(uint32_t i=0;i<(cnt*1000);i++);
}
static void udelay(uint32_t cnt)//microsecond delay
{
	for(uint32_t i=0;i<(cnt*1);i++);
}
void lcd_send_command(uint8_t cmd)
{
	//RS=0 for LCD command
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_RS, GPIO_PIN_RESET);

	//RW=0 for write
	//GPIO_WriteToOutputPin(LCD_GPIO_PORTB,LCD_GPIO_RW,GPIO_PIN_RESET);

	write_4_bits(cmd>>4);//higher nibble
	write_4_bits(cmd & 0x0F);//lower nibble
}

static void lcd_enable(void)//this enables the LCD to latch on to the values
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB,LCD_GPIO_EN,GPIO_PIN_SET);
	mdelay(1);//udelay(1) changed to mdelay(1)
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB,LCD_GPIO_EN,GPIO_PIN_RESET);

	//any value greater than 37 microseconds
}



void lcd_print_char(uint8_t data)
{
	//RS=1 for LCD user data
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB,LCD_GPIO_RS,GPIO_PIN_SET);

	//Rw =0 for write
	//GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_RW, GPIO_PIN_RESET);
	write_4_bits(data>>4);//higher nibble
	write_4_bits(data & 0x0F);//lower nibble
}


void lcd_print_string(char *message)
{
	do
	{
		lcd_print_char((uint8_t)*message++);
	}while(*message!='\0');
}

void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
	/*
	 * check page number 24 of data sheet
	 * return home command execution wait time is around 2 ms
	 */
}

void lcd_set_cursor(uint8_t row,uint8_t column)
{
	column--;
	switch(row)
	{
	case 1:
	/*Set cursor to 1st row address and add index*/
	lcd_send_command((column|=0x80));
	break;
	case 2:
	/*Set cursor to 2nd row address and add index*/
	lcd_send_command((column|=0xC0));
	break;
	default:
		break;
	}
}
void lcd_display_clear(void)
	{
	//display clear command
	lcd_send_command(LCD_CMD_DIS_CLEAR);

	mdelay(2);
	}

void lcd_init(void)
{
	//1.Configure the GPIO pins which are used for LCD connections

	GPIO_Handle_t lcd_signalA,lcd_signalB,lcd_signalC;

	lcd_signalB.pGPIOx=GPIOB;
	lcd_signalB.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signalB.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_RS;
	lcd_signalB.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalB.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalB.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalB);

	/*lcd_signalB.pGPIOx=GPIOB;
	lcd_signalB.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;

	lcd_signalB.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalB.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalB.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalB);*/

	lcd_signalB.pGPIOx=GPIOB;
	lcd_signalB.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signalB.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_EN;
	lcd_signalB.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalB.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalB.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalB);

	lcd_signalA.pGPIOx=GPIOC;
	lcd_signalA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signalA.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D4;
	lcd_signalA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalA.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalA);


	lcd_signalA.pGPIOx=GPIOB;
	lcd_signalA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signalA.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D5;
	lcd_signalA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalA.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalA);

	lcd_signalA.pGPIOx=GPIOA;
	lcd_signalA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signalA.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D6;
	lcd_signalA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalA.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalA);

	/*GPIOA->MODER|=(1U<<6);
	GPIOA->OSPEEDR|=(1U<<6);
	GPIOA->OSPEEDR|=(1U<<7);  Ignore this*/


	lcd_signalA.pGPIOx=GPIOA;
	lcd_signalA.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_signalA.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D7;
	lcd_signalA.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_signalA.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_signalA.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalA);


	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_D7, GPIO_PIN_RESET);



	//2.Do the LCD initialization
	mdelay(40);

	/*RS=0, for LCD command , have connected to ground */

		lcd_send_command(0x33);
		lcd_send_command(0x32);

		mdelay(5);
		//function set command
		lcd_send_command(LCD_CMD_4DL_2N_5X8F);

		mdelay(5);


		lcd_send_command(LCD_CMD_DIS_CLEAR);

		mdelay(5);


		//disply ON and cursor on

		lcd_send_command(LCD_CMD_DON_CURON);

		mdelay(5);


		//entry mode set
		lcd_send_command(LCD_CMD_INCADD);

		mdelay(5);

}

//Writes 4 bits of data/command to D4,5,6,7 lines
static void write_4_bits(uint8_t value)
	{
			GPIO_WriteToOutputPin(LCD_GPIO_PORTC,LCD_GPIO_D4,(value>>0)&0x01);
			GPIO_WriteToOutputPin(LCD_GPIO_PORTB,LCD_GPIO_D5,(value>>1)&0x01);//bringing all bits to LSB and then sending thru gpio
			GPIO_WriteToOutputPin(LCD_GPIO_PORTA,LCD_GPIO_D6,(value>>2)&0x01);
			GPIO_WriteToOutputPin(LCD_GPIO_PORTA,LCD_GPIO_D7,(value>>3)&0x01);

			lcd_enable();//after writing every nibble we have to instruct the LCD to latch that Data
		}
