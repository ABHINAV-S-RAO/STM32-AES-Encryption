#include "i2c.h"
#include "lcd.h"
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "aes.h"

#define AES_BLOCK_SIZE 16
#define MY_ADDR 	0x61
#define SLAVE_ADDR	0x68
#define ENABLE		1
#define NVIC_IPR8 (*(volatile uint32_t*)0xE000E420)

static void mdelay(uint32_t cnt)//millisecond delay(approx - too lazy to config timer)
{
	for(uint32_t i=0;i<(cnt*1000);i++);
}
void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++);

}


//PB6-I2C1 SCL
//PB7-I2C1 SDA
//PB3-I2C2 SDA
//PB10-I2C2 SCL
void I2C_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx=GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode=4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//Tx SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//Tx SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

	//Rx SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	//Rx SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_3;
	GPIO_Init(&I2CPins);
}

I2C_Handle_t I2C1Handle,I2C2Handle;
void I2C_Inits(void)
{

	I2C1Handle.pI2Cx=I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl=I2C_ACK_ENABLE;
	//Device address relevant only for slave
	I2C1Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);


	I2C2Handle.pI2Cx=I2C2;
	I2C2Handle.I2C_Config.I2C_ACKControl=I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_DeviceAddress=SLAVE_ADDR;
	I2C2Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C2Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

	I2C_Init(&I2C2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GPIOLED;
	//PA5 LED PC13 Btn
	GPIOBtn.pGPIOx=GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	GPIOLED.pGPIOx=GPIOA;
	GPIOLED.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOLED);
}

void lcd_print_hex(uint8_t *data, uint8_t len)
{
    char buf[3];
    for (uint8_t i = 0; i < len; i++)
    {
        sprintf(buf, "%02X", data[i]);
        lcd_print_string(buf);
    }
}
void lcd_print_raw_chars(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        lcd_print_char((char)data[i]);
    }
}


volatile uint8_t rx_block[16];
volatile uint32_t rx_index = 0;
volatile uint8_t stop_seen = 0;
volatile uint32_t m=0;
void I2C2_EV_IRQHandler(void)
{
	if (I2C2->SR1 & I2C_SR1_ADDR)
	{
	    volatile uint32_t temp;
	    temp = I2C2->SR1;
	    temp = I2C2->SR2;
	    (void)temp;
	}

    if (I2C2->SR1 & I2C_SR1_RXNE)
    {
        uint8_t data = I2C2->DR;

        if (m < 16)
            rx_block[m++] = data;

        //ensure ACK stays asserted
        I2C2->CR1 |= I2C_CR1_ACK;

        return;
    }

    if (I2C2->SR1 & I2C_SR1_STOPF)
    {
        volatile uint32_t temp = I2C2->SR1;
        I2C2->CR1|=I2C_CR1_PE;
        I2C2->CR1|=I2C_CR1_ACK;
        (void)temp;

        m = 0;
        return;
    }
}


void I2C2_ER_IRQHandler(void)
{
    if (I2C2->SR1 & I2C_SR1_AF)
        I2C2->SR1 &= ~I2C_SR1_AF;

    if (I2C2->SR1 & I2C_SR1_BERR)
        I2C2->SR1 &= ~I2C_SR1_BERR;

    if (I2C2->SR1 & I2C_SR1_ARLO)
        I2C2->SR1 &= ~I2C_SR1_ARLO;
}


int main()
{

	GPIO_PeriClockControl(GPIOA, ENABLE);//for LCD
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	lcd_init();
	lcd_display_clear();
	I2C_GPIOInits();
	//Disable I2C
	I2C1->CR1&=~I2C_CR1_PE;

	//Reset I2C peripheral
	RCC->APB1RSTR|=RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR&=~RCC_APB1RSTR_I2C1RST;

	// Re-enable I2C
	I2C1->CR1|=I2C_CR1_PE;



	I2C1->CR1|=I2C_CR1_STOP;
	I2C1->SR1&=~I2C_SR1_ARLO;

	I2C1->CR1|=I2C_CR1_SWRST;
	I2C1->CR1&=~I2C_CR1_SWRST;

	I2C2->CR1|=I2C_CR1_SWRST;
	I2C2->CR1&=~I2C_CR1_SWRST;


	I2C_Inits();
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_PeripheralControl(I2C2, ENABLE);

	I2C_ManageAcking(I2C2, ENABLE);

	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(I2C2_ER_IRQn);
	//i2C IRQ enable

	//NVIC_ISER1|=(1U<<1);//I2C2_EV_IRQn
	//NVIC_ISER1|=(1U<<2);//I2C2_ER_IRQn

	NVIC_SetPriority(I2C2_EV_IRQn, 5);
	NVIC_SetPriority(I2C2_ER_IRQn, 5);
	I2C2->CR2|=I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN|I2C_CR2_ITERREN;

	//enable I2C peripheral

	__enable_irq();   // or asm("cpsie i")
	lcd_set_cursor(0, 1);
	GPIO_ButtonInit();
	//PB6-I2C1 SCL
	//PB7-I2C1 SDA
	//PB11-I2C2 SDA
	//PB10-I2C2 SCL
	//i2c pins init

	//128 bit AES key
	uint8_t key[16] ={
				0x2b, 0x7e, 0x15, 0x16,
				0x28, 0xae, 0xd2, 0xa6,
				0xab, 0xf7, 0x15, 0x88,
				0x09, 0xcf, 0x4f, 0x3c };
	char input[] = ("Abhinav S Rao");

	uint8_t block[16];
	memset(block,' ',16);//Fill wiht spaces
	strncpy((char *)block,input,16);//copy upto 16 chars

	//Printing unencrypted value
	lcd_display_clear();
	lcd_print_string((char *)input);

	//Encrypt
	struct AES_ctx ctx;
	AES_init_ctx(&ctx, key);
	AES_ECB_encrypt(&ctx, block);
	mdelay(2000);

	//print encrypted hex
	lcd_display_clear();
	lcd_print_hex(block, 16);
	mdelay(2000);
	I2C2->CR1 |= I2C_CR1_ACK;

	//Send encrypted block
	I2C_MasterSendData(&I2C1Handle, block, 16, SLAVE_ADDR);




	//decrypt
	struct AES_ctx rx_ctx;
	AES_init_ctx(&rx_ctx, key);
	AES_ECB_decrypt(&rx_ctx, rx_block);
	lcd_display_clear();
	mdelay(2000);


	rx_block[15]='\0';
	lcd_display_clear();
	lcd_print_string((char *)rx_block);
	return 0;
}




