#include "SPI.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
        	SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
        	SPI3_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
        	SPI1_PCLK_DIS();
        }
        else if (pSPIx == SPI2)
        {
        	SPI2_PCLK_DIS();
        }
        else if (pSPIx == SPI3)
        {
        	SPI3_PCLK_DIS();
        }
    }
}

void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
    SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);
    uint32_t tempreg = 0;

    tempreg |= pSPI_Handle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;
    if (pSPI_Handle->SPI_Config.SPI_Busconfig == SPI_BUS_CONFIG_FD)
    {
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPI_Handle->SPI_Config.SPI_Busconfig == SPI_BUS_CONFIG_HD)
    {
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPI_Handle->SPI_Config.SPI_Busconfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }
    tempreg |= pSPI_Handle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;
    tempreg |= pSPI_Handle->SPI_Config.SPI_DFF << SPI_CR1_DFF;
    tempreg |= pSPI_Handle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;
    tempreg |= pSPI_Handle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;
    tempreg |= pSPI_Handle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

    pSPI_Handle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_TypeDef *pSPIx)
{
    // Implementation for DeInit
}

uint8_t SPI_GetFlag_Status(SPI_TypeDef *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // Wait until TXE is set
        while (!(pSPIx->SR & (1 << SPI_SR_TXE)));

        // Check the DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len -= 2;
            pTxBuffer += 2;
        }
        else
        {
            // 8-bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // Wait until RXNE is set
        while (!(pSPIx->SR & (1 << SPI_SR_RXNE)));

        // Check the DFF bit in CR1
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            // 8-bit DFF
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 <<SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_TypeDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 <<SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_TypeDef *pSPIx,uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4;
    *(NVIC_PR_BASE_ADDR + ipr) |= (IRQPriority << (8 * irq + 4));
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	//TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//handle txe
		spi_txe_interrupt_handle(pHandle);
	}
	//RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
		{
		//handle rxne
		spi_rxne_interrupt_handle(pHandle);
	}
	// ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//handle ovr
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1 save the TX buffer address and Length in4 in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		// 2 mARK the SPI state as a busy transmission so that
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// 3 Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		// WHEN, transmit will be handled at ISR code
	}
	return state;
}

uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	uint8_t state = pSPIHandle->RxStaTe;
	if(state != SPI_BUSY_IN_RX)
	{
		// 1 save the TX buffer address and Length in4 in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		// 2 mARK the SPI state as a busy transmission so that
		pSPIHandle->RxStaTe = SPI_BUSY_IN_RX;
		// 3 Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		// WHEN, transmit will be handled at ISR code
	}
	return state;
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16-bit DFF
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen -= 2;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    }
    else
    {
        // 8-bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (pSPIHandle->TxLen == 0)
    {
        SPI_CloseTransmission(pSPIHandle);
        SPI_AplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16-bit DFF
        *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer += 2;
    }
    else
    {
        // 8-bit DFF
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (pSPIHandle->RxLen == 0)
    {
        SPI_CloseReception(pSPIHandle);
        SPI_AplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1 clear ovr flag
	if(pSPIHandle->RxStaTe != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2 inform application
	SPI_AplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_AplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // Custom implementation to handle SPI events
    if (AppEv == SPI_EVENT_TX_CMPLT) {
        // Handle transmission complete event
    } else if (AppEv == SPI_EVENT_RX_CMPLT) {
        // Handle reception complete event
    } else if (AppEv == SPI_EVENT_OVR_ERR) {
        // Handle overrun error event
    }
}

