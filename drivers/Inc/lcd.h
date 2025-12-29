/*
 * lcd.h
 *
 *  Created on: Jul 26, 2025
 *      Author: abhin
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "stm32f446xx.h"
/*bsp exposed apis*/

void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_print_char(uint8_t data);
void lcd_display_return_home(void);
void lcd_print_string(char *message);
void lcd_set_cursor(uint8_t row,uint8_t column);
void lcd_display_clear(void);

/*Application configurable items*/
#define LCD_GPIO_PORTA 	GPIOA
#define LCD_GPIO_PORTB 	GPIOB
#define LCD_GPIO_PORTC	GPIOC
#define LCD_GPIO_RS		GPIO_PIN_NO_5
#define LCD_GPIO_RW		GPIO_PIN_NO_8
#define LCD_GPIO_EN		GPIO_PIN_NO_4
#define LCD_GPIO_D4		GPIO_PIN_NO_7
#define LCD_GPIO_D5 	GPIO_PIN_NO_8
#define LCD_GPIO_D6		GPIO_PIN_NO_7
#define LCD_GPIO_D7		GPIO_PIN_NO_6
/*LCD Commands*/
#define LCD_CMD_4DL_2N_5X8F		0x28//4 bit mode , 5x8 pixels
#define LCD_CMD_DON_CURON		0x0E//display on and cursor on
#define LCD_CMD_INCADD			0x06
#define LCD_CMD_DIS_CLEAR		0x01
#define LCD_CMD_DIS_RETURN_HOME	0x02


#endif /* INC_LCD_H_ */
