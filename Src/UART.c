#include "UART.h"
#include "RCC.h"

void USART_SetBaudRate(USART_TypeDef *pUSARTx,uint32_t BaudRate);
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		//remain usart6
	}
}

void USART_Init(USART_Handle_t *pUSART_Handle)
{
	uint32_t tempreg = 0;

	//enable the clock
	USART_PeriClockControl(pUSART_Handle->pUARTx,ENABLE);

	//MODE
	if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the Transmitter and Receiver bit field
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	//wordLenght
	tempreg |= pUSART_Handle->USART_Config.USART_WordLength <<USART_CR1_M;

	//parity control
	if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);
	}
	else if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Implement the code to enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		////Implement the code to enable the parity control odd
		tempreg |= (1 << USART_CR1_PS);
	}

	pUSART_Handle->pUARTx->CR1 |= tempreg;
	// end CR1

	//begin CR2
	tempreg = 0;
	tempreg |= pUSART_Handle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	pUSART_Handle->pUARTx->CR2 |= tempreg;
	//end CR2

	//begin CR3
	tempreg = 0;
	if(pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CLRT_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if(pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CLRT_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if(pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CLRT_CTS_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
		tempreg |= (1 << USART_CR3_CTSE);
	}

	pUSART_Handle->pUARTx->CR3 |= tempreg;
	//end CR3

	USART_SetBaudRate(pUSART_Handle->pUARTx,pUSART_Handle->USART_Config.USART_Baud);
}

void USART_SetBaudRate(USART_TypeDef *pUSARTx,uint32_t BaudRate)
{
	uint32_t PCLKx;
	uint32_t usartdiv;
	uint32_t M_part,F_part;
	uint32_t tempreg=0;

	// get clock
	if(pUSARTx == USART1)//REmain USART6
	{
		//usart1 and usart6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//check for over8(over sample)
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//over8 = 1, over sampling by 8
		usartdiv = ((25*PCLKx)/(2*BaudRate));
	}
	else
	{
		//over sampling by 16
		usartdiv = ((25*PCLKx)/(4*BaudRate));
	}

	// Mantissa
	M_part = usartdiv/100;
	tempreg |= M_part << 4;

	F_part = (usartdiv - (M_part*100));

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//over8 = 1, over sampling by 8
		F_part = (((F_part*8)+50)/100)&((uint8_t)0x07);
	}
	else
	{
		// over sampling by 16
		F_part = (((F_part*8)+50)/100)&((uint8_t)0x0F);
	}

	tempreg |= F_part;

	pUSARTx->BRR = tempreg;
}

void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

uint8_t USART_GetFlag_Status(USART_TypeDef *pUSARTx, uint32_t StatusFlagName)
{
	uint8_t status = RESET;
	if(pUSARTx->SR & StatusFlagName)
	{
		status =  SET;
	}
	return status;
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
	for(uint32_t i = 0; i< Len;i++)
	{
		//wait until TXE flag is set in SR
		while(!USART_GetFlag_Status(pUSARTHandle->pUARTx,USART_FLAG_TXE));

		//CHECK the USART WordLength item for 9BIT or 8BI in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//load the DR with 2bytes
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUARTx->DR = (*pdata & (uint16_t)0x01FF);

			//CHECK for USART parityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else //PARITY ENABLE, sent 8 bit
			{
				pTxBuffer++;
			}
		}
		else //USART WordLength item for 8bit
		{
			//this is 8bits data transfer
			pUSARTHandle->pUARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			// increment the buffer address
			pTxBuffer++;
		}
	}

	//wait till TC flag is set in the SR
	while(!USART_GetFlag_Status(pUSARTHandle->pUARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	for(uint32_t i=0;i< Len;i++)
	{
		//wait until RXNE set, when the buffer not enough the data and wait
		while(!USART_GetFlag_Status(pUSARTHandle->pUARTx,USART_FLAG_RXNE));

		//check word length 8bit or 9bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//check parity
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUARTx->DR & (uint16_t)0x01FF);

				pRxBuffer++;
				pRxBuffer++;
			}
			else //parity enable => 9BIT ( 8BIT + 1BIT PARITY)
			{
				*pRxBuffer = (pUSARTHandle->pUARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else //8bit
		{
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				*pRxBuffer = (pUSARTHandle->pUARTx->DR & (uint8_t)0xFF);
			}
			else //8bit (7bit + 1 bit parity)
			{
				*pRxBuffer = (pUSARTHandle->pUARTx->DR & (uint8_t)0x07F);
			}
			pRxBuffer++;
		}
	}
}

void USART_ClearFlag(USART_TypeDef *pUSARTx, uint32_t StatusFlagName)
{
	pUSARTx->SR &= ~ (StatusFlagName);
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
{
    if (EnorDis == ENABLE)
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t ipr = IRQNumber / 4;
	uint8_t irq = IRQNumber % 4;
	*(NVIC_PR_BASE_ADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}

uint8_t USART_SendData_IT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint8_t Len)
{
	uint8_t txstate = pUSART_Handle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSART_Handle->TxLen = Len;
		pUSART_Handle->pTxBuffer = pTxBuffer;
		pUSART_Handle->TxBusyState = USART_BUSY_IN_TX;

		//enable interrupt for TXE
		pUSART_Handle->pUARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//enable interrupt for TC
		pUSART_Handle->pUARTx->CR1 |= (1 << USART_CR1_TCIE);
		}
	return txstate;
}

uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint8_t Len)
{
	uint8_t rxstate = pUSART_Handle->RxBusyState;
	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSART_Handle->RxLen = Len;
		pUSART_Handle->pRxBuffer = pRxBuffer;
		pUSART_Handle->TxBusyState = USART_BUSY_IN_RX;

		(void)pUSART_Handle->pUARTx->DR;

		//enable interrupt for RXNE
		pUSART_Handle->pUARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

void USART_IRQHandling(USART_Handle_t *pUSART_Handle)
{
	uint32_t temp1, temp2;
	uint16_t *pdata;

	//TC flag
	//check the state of TC bit in the SR
	temp1 = pUSART_Handle->pUARTx->SR &(1 << USART_SR_TC);
	//check the state of TCEIE bit
	temp2 = pUSART_Handle->pUARTx->CR1 & (1 << USART_CR1_TCIE);
	if(temp1 && temp2)
	{
		//interrupt is because of TC
		//if TXLEN is zero, close transmit and call application
		if(pUSART_Handle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(!pUSART_Handle->TxLen)
			{
				//clear the TC flag
				pUSART_Handle->pUARTx->SR &= ~(1 << USART_SR_TC);
				//clear the TCIE control bit
				pUSART_Handle->pUARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				pUSART_Handle->TxBusyState = USART_READY;
				pUSART_Handle->pTxBuffer = NULL;
				pUSART_Handle->TxLen = 0;

				//call application
				USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	//TXE flag
	//check the state of TXE bit in the SR
	temp1 =pUSART_Handle->pUARTx->SR & (1 << USART_SR_TXE);
	//check the state of TXEIE bit in the CR1
	temp2 =pUSART_Handle->pUARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2)
	{
		//interrupt is because of TXE
		if(pUSART_Handle->TxBusyState ==USART_BUSY_IN_TX)
		{
			if(pUSART_Handle->RxLen > 0 )
			{
				if(pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					pdata = (uint16_t*)pUSART_Handle->pTxBuffer;
					pUSART_Handle->pUARTx->DR = (*pdata & (uint16_t)0x01FF);

					if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						pUSART_Handle->pTxBuffer++;
						pUSART_Handle->pTxBuffer++;
						pUSART_Handle->TxLen-=2;
					}
					else
					{
						pUSART_Handle->pTxBuffer++;
						pUSART_Handle->TxLen-=1;
					}
				}
				else
				{
					pUSART_Handle->pUARTx->DR =(*pUSART_Handle->pTxBuffer & (uint8_t)0xFF);
					pUSART_Handle->pTxBuffer++;
					pUSART_Handle->TxLen-=1;
				}
			}
			if(pUSART_Handle->TxLen == 0)
			{
				//clear the TXEIE
				pUSART_Handle->pUARTx->CR1 &= ~(1<< USART_CR1_TXEIE);
			}
		}
	}
	// RXNE flag
	temp1 = pUSART_Handle->pUARTx->DR & (1 << USART_SR_RXNE);
	temp2 = pUSART_Handle->pUARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		if(pUSART_Handle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSART_Handle->RxLen > 0 )
			{
				if(pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*((uint16_t*)pUSART_Handle->pRxBuffer)= pUSART_Handle->pUARTx->DR & (uint16_t)0x01FF;
						pUSART_Handle->pRxBuffer++;
						pUSART_Handle->pRxBuffer++;
						pUSART_Handle->RxLen-=2;
					}
					else
					{
					*pUSART_Handle->pRxBuffer = ( pUSART_Handle->pUARTx->DR & (uint8_t)0x0FF);
					pUSART_Handle->pRxBuffer++;
					pUSART_Handle->RxLen-=1;
					}
				}
				else
				{
					if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						*pUSART_Handle->pRxBuffer = (uint8_t)(pUSART_Handle->pUARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						*pUSART_Handle->pRxBuffer = (uint8_t)(pUSART_Handle->pUARTx->DR & (uint8_t)0x7F);
					}
					pUSART_Handle->pRxBuffer++;
					pUSART_Handle->RxLen-=1;

				}
			}
			if(!pUSART_Handle->RxLen)
			{
				//disable the RXNE
				pUSART_Handle->pUARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE);
				pUSART_Handle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSART_Handle,USART_EVENT_RX_CMPLT);
			}
		}
	}

	// CTS flag (not use for UART4 and UART5)
	//implement the code to check the status of CTS bit in SR
	temp1 = pUSART_Handle->pUARTx->SR & (1 << USART_SR_CTS);
	//implement the code to check the state of CTSE bit in CR3
	temp2 =  pUSART_Handle->pUARTx->CR1 & (1 << USART_CR3_CTSE);

	if(temp1 && temp2)
	{
		//implement the code to clear the CTS flag in SR
		pUSART_Handle->pUARTx->SR &= ~( 1 << USART_SR_CTS);
		//this interrupt is because of CTS
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_CTS);
	}

	//IDLE detection flag
	//implement the code to check the status of IDLE flag bit in SR
	temp1 = pUSART_Handle->pUARTx->SR & (1<<USART_SR_IDLE);
	//implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSART_Handle->pUARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		//implement the code to clear the IDLE flag
		temp1 = pUSART_Handle->pUARTx->SR  &= ~( 1 << USART_SR_IDLE);
		//this interrupt is because of IDLE
		USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_IDLE);
	}

	//OVERRUN detection flag
	//implement the code to the check the status of ORE flag in the SR
	temp1 = pUSART_Handle->pUARTx->SR & USART_SR_ORE;
	//implement the code to check the status of RXNEIE bit in the CR1
	temp2 = pUSART_Handle->pUARTx->CR1 & USART_CR1_RXNEIE;

	if(temp1 && temp2)
	{
		//this interrupt is because of OVERRUN error
		USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_ORE);
	}

	//ERROR flag
	temp2 = pUSART_Handle->pUARTx->CR3 & (1 << USART_CR3_EIE);
	if(temp2)
	{
		temp1 = pUSART_Handle->pUARTx->SR;
		if(temp1 & (1 <<USART_SR_FE))
		{
			USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_FE);
		}

		if(temp1 & (1 <<USART_SR_NE))
		{
			USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_NE);
		}

		if(temp1 & (1 <<USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_ORE);
		}
	}
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}
