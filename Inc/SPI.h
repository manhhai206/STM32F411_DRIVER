#ifndef DRIVERS_INC_SPI_H_
#define DRIVERS_INC_SPI_H_

#include "stm32f4.h"
#include "stdio.h"

typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_Busconfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct
{
    SPI_TypeDef *pSPIx;
    SPI_Config_t SPI_Config;
    uint8_t 	*pTxBuffer;
    uint8_t		*pRxBuffer;
    uint32_t 	TxLen;
    uint32_t	RxLen;
    uint8_t		TxState;
    uint8_t		RxStaTe;
} SPI_Handle_t;

#define SPI_EVENT_TX_CMPLT			   1
#define SPI_EVENT_RX_CMPLT			   2
#define SPI_EVENT_OVR_ERR			   3
#define SPI_EVENT_CRC_ERR		   	   4

#define SPI_READY					   0
#define SPI_BUSY_IN_RX 				   1
#define SPI_BUSY_IN_TX				   2

#define SPI_DEVICE_MODE_MASTER         1
#define SPI_DEVICE_MODE_SLAVE          0

#define SPI_BUS_CONFIG_FD              1
#define SPI_BUS_CONFIG_HD              2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY  3

#define SPI_SCLK_SPEED_DIV2            0
#define SPI_SCLK_SPEED_DIV4            1
#define SPI_SCLK_SPEED_DIV8            2
#define SPI_SCLK_SPEED_DIV16           3
#define SPI_SCLK_SPEED_DIV32           4
#define SPI_SCLK_SPEED_DIV64           5
#define SPI_SCLK_SPEED_DIV128          6
#define SPI_SCLK_SPEED_DIV256          7

#define SPI_DFF_8BITS                  0
#define SPI_DFF_16BITS                 1

#define SPI_CPOL_HIGH                  1
#define SPI_CPOL_LOW                   0

#define SPI_CPHA_HIGH                  1
#define SPI_CPHA_LOW                   0

#define SPI_SSM_EN                     1
#define SPI_SSM_DI                     0

#define SPI_CR1_CPHA                   0
#define SPI_CR1_CPOL                   1
#define SPI_CR1_MSTR                   2
#define SPI_CR1_BR                    3
#define SPI_CR1_SPE                    6
#define SPI_CR1_LSBFIRST               7
#define SPI_CR1_SSI                    8
#define SPI_CR1_SSM                    9
#define SPI_CR1_RXONLY                 10
#define SPI_CR1_DFF                    11
#define SPI_CR1_CRCNEXT                12
#define SPI_CR1_CRCEN                  13
#define SPI_CR1_BIDIOE                 14
#define SPI_CR1_BIDIMODE               15

#define SPI_CR2_RXDMAEN                0
#define SPI_CR2_TXDMAEN                1
#define SPI_CR2_SSOE                   2
#define SPI_CR2_ERRIE                  5
#define SPI_CR2_RXNEIE                 6
#define SPI_CR2_TXEIE                  7

#define SPI_SR_RXNE                    0
#define SPI_SR_TXE                     1
#define SPI_SR_CHSIDE                  2
#define SPI_SR_UDR                     3
#define SPI_SR_CRCERR                  4
#define SPI_SR_MODF                    5
#define SPI_SR_OVR                     6
#define SPI_SR_BSY                     7
#define SPI_SR_FRE                     8

#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_BUSY_FLAG				(1 << SPI_SR_BSY)

void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_TypeDef *pSPIx);

void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_GetFlag_Status(SPI_TypeDef *pSPIx, uint32_t FlagName);
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t Len);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Len);

void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_TypeDef *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_TypeDef *pSPIx,uint8_t EnorDis);

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_AplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif // DRIVERS_INC_SPI_H_
