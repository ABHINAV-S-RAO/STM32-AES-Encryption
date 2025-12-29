/*
 * i2c.c
 *
 *  Created on: Aug 7, 2025
 *      Author: abhin
 */

#include "i2c.h"
#include "stm32f446xx.h"
#define ENABLE 				1
#define DISABLE				0
uint16_t AHB_PreScalar[8]={2,4,8,16,64,128,256,512};//holds the prescalar values basically
uint16_t APB1_PreScalar[4]={2,4,8,16};
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);


void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1|=(1U<<I2C_CR1_START);//bit 8
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1|=(I2C_CR1_STOP);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	(void)I2C1->SR1;//reads SR1
	//in this function we send the 7 bit address and 1 bit of r/w
	SlaveAddr=SlaveAddr<<1;//shifting the slave address to make space for r/w bit
	SlaveAddr&=~(1);//clears the 0th bit only , while keeping all other bits intact
	pI2Cx->DR=SlaveAddr;//SlaveAddr=Slave address(7bit)+ R/W bit (total 8)-> R/W is in the LSB
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	//in this function we send the 7 bit address and 1 bit of r/w
	SlaveAddr=SlaveAddr<<1;//shifting the slave address to make space for r/w bit
	SlaveAddr|=(1);//clears the 0th bit only , while keeping all other bits intact
	pI2Cx->DR=SlaveAddr;//SlaveAddr=Slave address(7bit)+ R/W bit (total 8)-> R/W is in the LSB
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead=pI2Cx->SR1;
	dummyRead=pI2Cx->SR2;
	(void)dummyRead;
}


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi==ENABLE)
	{
		pI2Cx->CR1|=(I2C_CR1_PE);
		//pI2C_BaseAddress->CR1|= I2C_CR1_PE_Bit_Mask;
	}
	else
	{
		pI2Cx->CR1&=~(1U<<0);
	}
}
/**********************************************************************************************************************************************/

/**************************************************************************************
 * @fn			- PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	//based on what the user gives, disable that particular gpio peripheral
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();

		}


	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();

		}
	}


}
/**********************************************************************************************************************************************/

/**************************************************************************************
 * @fn			- I2C_init
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
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint8_t trise;
	uint32_t tempreg=0;
	//enable the clock for i2c peripheral

	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//1.Enabling the automatic ACK for master CR1
	tempreg|=pI2CHandle->I2C_Config.I2C_ACKControl<<10;//I2CAckControl holds Value like 1 , which is same as using (1U<<10)
	pI2CHandle->pI2Cx->CR1=tempreg;

	//2.Configure The FREQ field of CR2
	tempreg=0;
	tempreg=RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2= tempreg&0x3F;//(0x3F is 11111)in tempreg only 5 bits should be valid and rest shld be masked out

	//3.Configuring the OAR( own address register ) if slave I2C_OAR1
	//2 configs available -> 7 bit and 10 bit, but we use 7 usually
	//store 7 bit address in I2C_OAR-> ADD[7:1]
	tempreg=pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;//left shift by 1 cuzz we dont use ADD[0] bit
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//Clock configurations

	//4. CCR Calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is std mode so bit 15 is 0

		ccr_value = RCC_GetPCLK1Value()/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ;
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//mode is fast mode so bit 15 is 1
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if( pI2CHandle->I2C_Config.I2C_FMDutyCycle ==I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value()/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ;
		}
		else if( pI2CHandle->I2C_Config.I2C_FMDutyCycle ==I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value()/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ;
		}
		tempreg |= (ccr_value & 0xFFF);

	}
	pI2CHandle->pI2Cx->CCR = tempreg;
	//Trise configuration


	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is std mode

		uint8_t trise;
		trise = (RCC_GetPCLK1Value()/1000000U) + 1 ; //+1 acc to ref manual


	}
	else
	{
		//mode is fast mode

		trise = (RCC_GetPCLK1Value()* 300 /1000000000U) + 1 ; //+1 acc to ref manual
	}
	pI2CHandle->pI2Cx->TRISE = (trise & 0x3F);


}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}








void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1.Generate the START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.Confirm that the start generation is completed by checking the SB flag in the SR1
	//Note:Until SB is cleared SCL will be stretched(Pulled to LoW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));




	//3.Send the address of the slave with r/nw bit set to w(0) (Total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4.Confirm that the address phase is completed by checking the ADDR flag in the SR1
	while(!	I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5.Clear the ADDR flag acc toits software sequence
	volatile uint32_t temp;
	temp = I2C1->SR1;
	temp = I2C1->SR2;
	(void)temp;

	//Note: Until ADDR is cleared SCL will be stretched(pulled to LOW)
	//I2C_ClearADDRFlag(pI2CHandle->pI2Cx);//reads the SR1 and SR2 which is required
	    if (!(I2C1->SR2 & I2C_SR2_MSL))
	    {
	        while(1);// Not master
	    }

	    if (!(I2C1->SR2 & I2C_SR2_TRA))
	    {
	       while(1); // Not transmitter
	    }

	//6.Send the data until len=0 now to the data register , but before that check the TxE
	while(Len>1)
	{
		while(!	((pI2CHandle->pI2Cx->SR1) & I2C_SR1_TXE));//wait till TXE is set
		pI2CHandle->pI2Cx->DR=*pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//For the last byte [spl way of flag set]
	while(!(pI2CHandle->pI2Cx->SR1 & I2C_SR1_TXE));
	pI2CHandle->pI2Cx->DR=*pTxBuffer;

	//Wait only once for BTF
	while(!	(pI2CHandle->pI2Cx->SR1 & I2C_FLAG_BTF));//Wait until BTF is set
	//7.WHen Len becomes zero , we wait for TXE=1 and BTF=0 before generating the STOP condition
	//Note: TXE=1 ,BTF=1 means that both SR And DR are empty and the next transmission should begin
	//when BTF=1 SCL will be stretched/pulled to LOW
	//while(!	I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE ));//wait until TxE is set
	//while(!	I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));//Wait until BTF is set

	//8.Generating STOP condition and master need not to wait for the completion of STOP condition
	//Note:generating STOP , automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber %32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}
		else if( IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if( IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}


	}


}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. first find the IPR of the register
	uint8_t iprx = IRQNumber/ 4;
	uint8_t iprx_section = IRQNumber %4;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8- NO_OF_PRIORITY_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


//void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
//{
	//Interrupt handling for both master and slave mode of a device


	//1.Handle for Interrupt generated by SB event
	//NOTE: SB flag is only applicable in master mode

	//2. Handle for interrupt generated
//}

void I2C_EV_IRQHandling1(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint8_t temp1, temp2, temp3;

	//check if the interrupt event flags are actually set?

	temp1 = pI2CHandle->pI2Cx->CR2 & (I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_SB);
	//1. Handle for interrupt by SB
	//note : SB is only applicable in master mode
	if( temp1 && temp3)
	{
		//1. Generated due to SB event

		//2. Execute the address phase
	    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
	    {
	    	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	    }
	    else if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
	    {
	    	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	    }
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_ADDR);

	//2. Handle for interrupt generated by ADDR event
	//note: When master mode : Addr is sent
	//		When slave mode  : Addr is matched with own addr

	if( temp1 && temp3)
	{
		//interrupt for ADDR flag set, clock stretch until further action
		//Imp to first clear the I2C addr flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF ( Byte Transfer Finished) event
	if( temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//Make sure TXE also set
			if(pI2CHandle->pI2Cx->SR1 & (I2C_SR1_TXE))
			{
				// BTF, TXE both are 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. Generate STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset all member elem of handle struct
					I2C_CloseSendData();

					//3. Notify applic about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
	//note: stop detection flag only in slave mode
	if( temp1 && temp3)
	{
		//STOPF flag is set
		//clear the stop flag
		// read SR1, write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the appplicaition that stop is detected

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_TXE);
	//5. Handle for interrupt by TXE event
	if(temp1 && temp2 && temp3 )
	{
		//check if device is master
		if(pI2CHandle->pI2Cx->SR2 & (I2C_SR2_MSL))
		{


			//TXE flag is set
			//We do data transmit only if busy in TX
			if(pI2CHandle->TxRxState == 1U<<I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen >0)
				{
					//1. load the data into DR

					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					//2. Decrement Tx Length
					pI2CHandle->TxLen--;

					//3. Increase buffer address
					pI2CHandle->pTxBuffer++;

				}
			}
		}

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_FLAG_RXNE);

	//6. Handle for interrupt by RXNE event

	if(temp1 && temp2 && temp3 )
	{
		//RXNE flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//We have to recieve data
			if(pI2CHandle->RxSize == 1)
			{

			}

			if(pI2CHandle->RxSize > 1)
			{

			}

		}
	}

}






uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	//the function will give a 1 or 0

	// Get the status reg bit and check its value
	if(pI2Cx->SR1 & FlagName)//WE are reading the SR1 so as to clear the SB in SR1. #procedure
	{
		return FLAG_SET;
	}

	else
	{
		return FLAG_RESET;
	}
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
		{
			if(EnorDi==I2C_ACK_ENABLE)
			{
				//Enable ack
				//10th bit in CR1 , set and cleared by software when PE=0
				pI2Cx->CR1|=(I2C_CR1_ACK);
			}
			else
			{
				//disable ack
				pI2Cx->CR1&=~(I2C_CR1_ACK);
			}
		}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t SystemClk;
	uint32_t pclk1;
	uint8_t clksrc=((RCC->CFGR>>2)* 0x3);//bring those 2 bits to the lsb position and mask out all other bits
	uint8_t temp,ahbp,apb1p;//ahbp stands for ahb pre scalar
	// System clock switch status done by hardware
	if(clksrc==0)//System clock is HSI which is 16 Mhz
	{
		SystemClk=16000000;
	}else if(clksrc==1)//Sys clk is HSE which is 8 Mhz
	{
		SystemClk=80000000;
	}else if(clksrc==2)
	{
		SystemClk=RCC_GetPLLOutputClock();
	}
	// FOR reading the AHB PRESCALAR
	temp=(RCC->CFGR>>4) & 0xF;//gets the value to the extreme right position and then mask all other bits other than first four

	if(temp<8)
	{
		ahbp=1;
	}else
	{
		ahbp=AHB_PreScalar[temp-8];//8-8=0 so we get division factor as 2 , and so on
	}

	//for apb prescalar
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 8)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScalar[temp-4];
	}

	pclk1= (SystemClk / ahbp)/apb1p ;//final pclk calculated

	return pclk1;
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1> Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is compleete by checking the SB flag in the SR1
	//UNTIL SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));


	//3. Send the address of the slave with R/W bit set to 1(total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Wait until the address phase is complete (check the ADDR flag)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	//procedure to read only 1 byte

	if(Len==1)
	{
		//Disable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Wait until RxNE becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));


		//read data into buffer
		*pRxBuffer=pI2CHandle->pI2Cx->DR;
		return;

	}
	//procedure to read the data from slave when Len>1
	else if(Len>1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Read the data until Len becomes 0
		for(uint32_t i=Len;i>0;i--)
		{
			//Wait until RxNE becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

			if(i==2)//if last 2 bytes are remaining
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//read the data from data register into the buffer
			*pRxBuffer=pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;//only 1 byte increment , so address ++
		}

	}
	//re-enable acking
	I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);

}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR=data;
}

void I2C_SlaveReceiveData(I2C_RegDef_t *pI2C, uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;

    while (i < len)
    {
        // Wait for data
        while (!(pI2C->SR1 & I2C_SR1_RXNE));

        buf[i++] = pI2C->DR;

        // Check STOP condition
        if (pI2C->SR1 & I2C_SR1_STOPF)
        {
            volatile uint32_t temp = pI2C->SR1;
            pI2C->CR1 |= 0; // clear STOPF
            break;
        }
    }
}


uint32_t RCC_GetPLLOutputClock(void)
{


	return 0;
}

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;
	//Make sure the bus isnt busy
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		//Update the buffer and values
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |=(I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2|=(I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2|=(I2C_CR2_ITERREN);

	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))//when both SR and DR are not busy
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITERREN);
	}

	return busystate;
}








