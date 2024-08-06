#ifndef DRIVERS_INC_UART_H_
#define DRIVERS_INC_UART_H_
// 1 Set Bit M in USART_CR1 to Choose 8-bit or 9-bit Word Length
// 2 Choose the Number of Stop Bits in USART_CR2
// 3 Set Baud Rate in USART_BRR
// 4 Set Bit UE in USART_CR1 to Enable USART
// 5 Enable Receiver by Setting RE in USART_CR1
// 6 When a Character is Received, Wait Until RXNE is Set and Read Data from Data Register
// 7 Enable RXNEIE to Receive Interrupts When Data is Available

#include "stm32f4.h"

#define USART_SR_PE     0
#define USART_SR_FE     1
#define USART_SR_NE     2
#define USART_SR_ORE    3
#define USART_SR_IDLE   4
#define USART_SR_RXNE   5
#define USART_SR_TC     6
#define USART_SR_TXE    7
#define USART_SR_LBD    8
#define USART_SR_CTS    9

#define USART_CR1_SBK     0
#define USART_CR1_RWU     1
#define USART_CR1_RE      2
#define USART_CR1_TE      3
#define USART_CR1_IDLEIE  4
#define USART_CR1_RXNEIE  5
#define USART_CR1_TCIE    6
#define USART_CR1_TXEIE   7
#define USART_CR1_PEIE    8
#define USART_CR1_PS      9
#define USART_CR1_PCE     10
#define USART_CR1_WAKE    11
#define USART_CR1_M       12
#define USART_CR1_UE      13
#define USART_CR1_OVER8   15

#define USART_CR2_ADD      0
#define USART_CR2_LBDL     5
#define USART_CR2_LBDIE    6
#define USART_CR2_LBCL     8
#define USART_CR2_CPHA     9
#define USART_CR2_CPOL     10
#define USART_CR2_CLKEN    11
#define USART_CR2_STOP     12
#define USART_CR2_LINEN    14

#define USART_CR3_EIE      0
#define USART_CR3_IREN     1
#define USART_CR3_IRLP     2
#define USART_CR3_HDSEL    3
#define USART_CR3_NACK     4
#define USART_CR3_SCEN     5
#define USART_CR3_DMAR     6
#define USART_CR3_DMAT     7
#define USART_CR3_RTSE     8
#define USART_CR3_CTSE     9
#define USART_CR3_CTSIE    10
#define USART_CR3_ONEBIT   11

typedef struct {
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

typedef struct
{
	USART_TypeDef *pUARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

#define USART_MODE_ONLY_TX 		0
#define USART_MODE_ONLY_RX 		1
#define USART_MODE_TXRX	   		2

#define USART_STD_BAUD_1200     1200
#define USART_STD_BAUD_2400     2400
#define USART_STD_BAUD_9600     9600
#define USART_STD_BAUD_19200    19200
#define USART_STD_BAUD_38400    38400
#define USART_STD_BAUD_56000    56000
#define USART_STD_BAUD_57600    57600
#define USART_STD_BAUD_115200   115200
#define USART_STD_BAUD_230400   230400
#define USART_STD_BAUD_460800   460800
#define USART_STD_BAUD_921600   921600
#define USART_STD_BAUD_2000000  2000000
#define USART_STD_BAUD_3000000  3000000

#define USART_PARITY_EN_ODD			2
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_DISABLE		0

#define USART_WORDLEN_8BITS			0
#define USART_WORDLEN_9BITS			1

#define USART_STOPBITS_1			0
#define USART_STOPBITS_0_5			1
#define USART_STOPBITS_2			2
#define USART_STOPBITS_1_5 			3

#define USART_HW_FLOW_CLRT_NONE		0
#define USART_HW_FLOW_CLRT_CTS		1
#define USART_HW_FLOW_CLRT_RTS		2
#define USART_HW_FLOW_CLRT_CTS_RTS	3

#define USART_FLAG_TXE				(1 << USART_SR_TXE)
#define USART_FLAG_RXNE				(1 << USART_SR_RXNE)
#define USART_FLAG_TC				(1 << USART_SR_TC)

#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2
#define USART_READY					0

#define USART_EVENT_TX_CMPLT		0
#define USART_EVENT_RX_CMPLT		1
#define USART_EVENT_IDLE			2
#define USART_EVENT_CTS				3
#define USART_EVENT_PE				4
#define USART_ERR_FE				5
#define USART_ERR_NE				6
#define USART_ERR_ORE				7

/*
 * Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi);

/*
 * Initial and DeInit
 */
void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_DeInit(USART_Handle_t *pUSART_Handle);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendData_IT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint8_t Len);
uint8_t USART_ReceiveData_IT(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint8_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSART_Handle);

/*
 *  Other Peripheral Control APIs
 */
uint8_t USART_GetFlag_Status(USART_TypeDef *pUSARTx, uint32_t StatusFlagName);
void USART_ClearFlag(USART_TypeDef *pUSARTx, uint32_t StatusFlagName);
void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnorDi);
void USART_SetBaudRate(USART_TypeDef *pUSARTx,uint32_t BaudRate);

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event);

#endif /* DRIVERS_INC_UART_H_ */
