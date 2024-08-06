#include "I2C.h"
#include "RCC.h"

static void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx);
static void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (1 <<I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->RxTxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// FIRST disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR Flag( read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//clear the ADDR flag(read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//device is in salve mode
		//clear the ADDR Flag( read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//1 load the data in to  DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2 decrement the TxLen
		pI2CHandle->TxLen--;

		//3 increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->RxTxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl ==  I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// we have to do the data reception
	if(pI2CHandle->RxLen == 1)
	{
		*pI2CHandle->pRxBuffer == pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

		//read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//close the I2C data reception and notify the application

		//1 generate tHe stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 close the I2C RX
		I2C_CloseReceiveData(pI2CHandle);

		//3 Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE);
	}
}

void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
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
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// enable ACK
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//FREQ CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// OAR1 7bit or 10bit
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);// should always be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//CCR
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else //fast mode
	{
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			 ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;

	//TRISE
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		tempreg = (RCC_GetPCLK1Value() /1000000U)+ 1;
	}
	else
	{
		//fast mode
		tempreg = ((RCC_GetPCLK1Value()*300) /1000000U)+ 1;
	}
	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x3F);
}

uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_GenerateStopCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// checking the SB flag in the SR1, SCL = LOW
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	// send address	data and bit read/write(8 bit)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	// clear the ADDR flag, SCL = LOW
	I2C_ClearADDRFlag(pI2CHandle);

	//send the data until LEN = 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//when LEN  = 0 wait for TXE = 0 and BTF = 1 before generating the STOP condition
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//generate STOP condition
	//Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/ no write bit set to R(1)( total 8 bit)
	I2C_ExecuteAddressPhaseRead   (pI2CHandle->pI2Cx, SlaveAddr);

	//checking the ADDR flag in the SR1
	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//READ only 1 byte from driver
	if(Len == 1)
	{
		//disable ACKING
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		// generation STOP condition
		if(Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//when LEN > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until LEN becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until RNXE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//disable ACKING
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generation STOP condition
				if(Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				//read the data from data register in to buffer
				*pRxBuffer = pI2CHandle->pI2Cx->DR;
				pRxBuffer++;
			}
		}
	}
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
	        if (IRQNumber <= 31)
	        {
	            *NVIC_ISER0 |= (1 << IRQNumber);
	        }
	        else if (IRQNumber >= 32 && IRQNumber < 64)
	        {
	            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
	        }
	        else if (IRQNumber >= 64 && IRQNumber < 96)
	        {
	            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
	        }
	  }
	  else
	  {
	        if (IRQNumber <= 31)
	        {
	            *NVIC_ICER0 |= (1 << IRQNumber);
	        }
	        else if (IRQNumber >= 32 && IRQNumber < 64)
	        {
	            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
	        }
	        else if (IRQNumber >= 64 && IRQNumber < 96)
	        {
	            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
	        }
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;
	*(NVIC_PR_BASE_ADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->RxTxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->RxTxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->RxTxState;
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->RxTxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//implement the code to disable ITEVTEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->RxTxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//SB FLAG
	if(temp1 && temp3)
	{
		if(pI2CHandle->RxTxState ==I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->RxTxState ==I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//ADDR event
	//when master mode: Address is sent
	//when slave mode : Address is similar with individual address
	if(temp1 && temp3)
	{
		// Interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//Byte transfer finished
	if(temp1 && temp3)
	{
		//BTF is set
		if(pI2CHandle->RxTxState == I2C_BUSY_IN_TX)
		{
			// make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF,TXE =1
				if(pI2CHandle->TxLen ==0)
				{
					//1 generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2 reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					//3 notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->RxTxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOP flag is set
		//clear the STOPF

		pI2CHandle->pI2Cx->CR1 |= 0x0000;


		//notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//transmit empty
	if(temp1 && temp2 && temp3)
	{
		//check  for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//we have  to do the data transmission
			if(pI2CHandle->RxTxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//slave
			//make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1  << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		//check  for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			//device is master

			//RXNE flag is set
			if(pI2CHandle->RxTxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(pI2CHandle->pI2Cx->SR2 & (1  << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint8_t temp1, temp2;

	//ITERREN control bit
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITEVTEN);

	//check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//this is bus error

		//implement the code to clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	//check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		//this is arbitration lost error

		//implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//check for ACK failure error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		//this is ACK failure error

		//implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//check for Overrun error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		//this is Overrun error

		//implement the code to clear the Overrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	//check for Time out error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		//this is Time out error

		//implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t ApEv)
{

}
