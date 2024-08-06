#include "GPIO.h"

void GPIO_PeripheralClockControl(GPIO_TypeDef *GPIOx, uint8_t EnorDis)
{
    if (EnorDis == ENABLE)
    {
        if (GPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (GPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (GPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (GPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (GPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (GPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (GPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (GPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (GPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if (GPIOx == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if (GPIOx == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if (GPIOx == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
        else if (GPIOx == GPIOD)
        {
            GPIOD_PCLK_DIS();
        }
        else if (GPIOx == GPIOE)
        {
            GPIOE_PCLK_DIS();
        }
        else if (GPIOx == GPIOF)
        {
            GPIOF_PCLK_DIS();
        }
        else if (GPIOx == GPIOG)
        {
            GPIOG_PCLK_DIS();
        }
        else if (GPIOx == GPIOH)
        {
            GPIOH_PCLK_DIS();
        }
        else if (GPIOx == GPIOI)
        {
            GPIOI_PCLK_DIS();
        }
    }
}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. Configure mode for GPIO
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->GPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->GPIOx->MODER |= temp;
	}
	else // interrupt mode
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
		    EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Configure Falling trigger
		    EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear Rising trigger
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
		    EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Configure Rising trigger
		    EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear Falling trigger
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
		    EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Configure Rising trigger
		    EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Configure Falling trigger
		}

		// Configure EXTI interrupt system
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->GPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2 * 4)); // Clear previous configuration
		SYSCFG->EXTICR[temp1] |= (code << (temp2 * 4)); // Configure EXTI line

		// Enable interrupt on the pin
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->OSPEEDR |= temp;

	// 3. Configure pull-up/pull-down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->PUPDR |= temp;

	// 4. Configure output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->GPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->GPIOx->OTYPER |= temp;

	// 5. Configure alternate function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->GPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->GPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2));
	}
}


void GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        RCC->AHB1RSTR |= (1 << 0);
        RCC->AHB1RSTR &= ~(1 << 0);
    }
    else if (GPIOx == GPIOB)
    {
        RCC->AHB1RSTR |= (1 << 1);
        RCC->AHB1RSTR &= ~(1 << 1);
    }
    else if (GPIOx == GPIOC)
    {
        RCC->AHB1RSTR |= (1 << 2);
        RCC->AHB1RSTR &= ~(1 << 2);
    }
    else if (GPIOx == GPIOD)
    {
        RCC->AHB1RSTR |= (1 << 3);
        RCC->AHB1RSTR &= ~(1 << 3);
    }
    else if (GPIOx == GPIOE)
    {
        RCC->AHB1RSTR |= (1 << 4);
        RCC->AHB1RSTR &= ~(1 << 4);
    }
    else if (GPIOx == GPIOF)
    {
        RCC->AHB1RSTR |= (1 << 5);
        RCC->AHB1RSTR &= ~(1 << 5);
    }
    else if (GPIOx == GPIOG)
    {
        RCC->AHB1RSTR |= (1 << 6);
        RCC->AHB1RSTR &= ~(1 << 6);
    }
    else if (GPIOx == GPIOH)
    {
        RCC->AHB1RSTR |= (1 << 7);
        RCC->AHB1RSTR &= ~(1 << 7);
    }
    else if (GPIOx == GPIOI)
    {
        RCC->AHB1RSTR |= (1 << 8);
        RCC->AHB1RSTR &= ~(1 << 8);
    }
}


uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((GPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

uint16_t GPIO_ReadPort(GPIO_TypeDef *GPIOx)
{
    uint16_t value;
    value = (uint16_t)(GPIOx->IDR);
    return value;
}

void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        GPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        GPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber)
{
    GPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
{
    uint8_t ipr = IRQNumber / 4;
    uint8_t irq = IRQNumber % 4;
    *(NVIC_PR_BASE_ADDR + ipr) |= (Priority << (8 * irq + 4));
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);
    }
}

