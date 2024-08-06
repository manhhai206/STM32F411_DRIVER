#include "RCC.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}

	// AHB
	temp = ((RCC->CFGR >> 4) & 0x0F);
	if(temp < 0x08 )
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// APB1
	temp = ((RCC->CFGR >>10 ) & 0x07);
	if(temp < 0x04)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;
	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2,SystemClk=0;
	uint8_t clksrc,temp,ahbp,apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}

	// AHB
	temp = ((RCC->CFGR >> 4) & 0x0F);
	if(temp < 0x08 )
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// APB1
	temp = ((RCC->CFGR >>13 ) & 0x07);
	if(temp < 0x04)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB1_PreScaler[temp-4];
	}

	pclk2 = (SystemClk / ahbp) / apb2p;
	return pclk2;

}



