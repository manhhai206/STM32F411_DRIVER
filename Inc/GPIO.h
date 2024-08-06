#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f4.h"

typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFuncMode;
} GPIO_PinConfig_t;

typedef struct {
    GPIO_TypeDef *GPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

// GPIO pin numbers
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15

// GPIO pin modes
#define GPIO_MODE_INPUT         0x00
#define GPIO_MODE_OUTPUT        0x01
#define GPIO_MODE_AF            0x02
#define GPIO_MODE_ANALOG        0x03

// GPIO pin output types
#define GPIO_OP_TYPE_PP         0x00
#define GPIO_OP_TYPE_OD         0x01

// GPIO pin speeds
#define GPIO_SPEED_LOW          0x00
#define GPIO_SPEED_MEDIUM       0x01
#define GPIO_SPEED_FAST         0x02
#define GPIO_SPEED_HIGH         0x03

// GPIO pin pull-up/pull-down
#define GPIO_NO_PUPD            0x00
#define GPIO_PIN_PU             0x01
#define GPIO_PIN_PD             0x02

// GPIO interrupt modes
#define GPIO_MODE_IT_FT     0x04
#define GPIO_MODE_IT_RT     0x08
#define GPIO_MODE_IT_RFT    0x0C

// GPIO pull configurations
#define GPIO_NOPULL         0x00000000u
#define GPIO_PULLUP         0x00000001u
#define GPIO_PULLDOWN       0x00000002u

// Function prototypes
void GPIO_PeripheralClockControl(GPIO_TypeDef *GPIOx, uint8_t EnorDis);
void GPIO_Init(GPIO_Handle_t *GPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *GPIOx);

uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_TypeDef *GPIOx);
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void GPIO_IRQPriorityConfig(uint8_t IRQHNumber, uint32_t Priority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_GPIO_H_ */
