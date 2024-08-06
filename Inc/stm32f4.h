#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include "stdint.h"
#include "stddef.h"

/* NVIC ISERx Registers */
#define NVIC_ISER0         ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1         ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2         ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3         ((volatile uint32_t *)0xE000E10C)

/* NVIC ICERx Registers */
#define NVIC_ICER0         ((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1         ((volatile uint32_t *)0XE000E184)
#define NVIC_ICER2         ((volatile uint32_t *)0XE000E188)
#define NVIC_ICER3         ((volatile uint32_t *)0XE000E18C)

#define NVIC_PR_BASE_ADDR  ((volatile uint32_t *)0xE000E400)

/* Memory Base Addresses */
#define FLASH_BASE_ADDR         0x08000000U
#define SRAM1_BASE_ADDR         0x20000000U
#define SRAM2_BASE_ADDR         0x2001C000U
#define ROM_BASE_ADDR           0x1FFF0000U

/* Peripheral Base Addresses */
#define PERIPH_BASE             0x40000000U
#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE         (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE         (PERIPH_BASE + 0x10000000U)

/* APB1 Peripherals */
#define TIM2_BASE_ADDR          (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE_ADDR          (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE_ADDR          (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE_ADDR          (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE_ADDR          (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE_ADDR          (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE_ADDR         (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE_ADDR         (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE_ADDR         (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE_ADDR           (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE_ADDR          (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE_ADDR          (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE_ADDR          (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE_ADDR          (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE_ADDR        (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE_ADDR        (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE_ADDR         (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE_ADDR         (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE_ADDR          (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE_ADDR          (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE_ADDR          (APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASE_ADDR          (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE_ADDR          (APB1PERIPH_BASE + 0x6800)
#define PWR_BASE_ADDR           (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE_ADDR           (APB1PERIPH_BASE + 0x7400)

/* APB2 Peripherals */
#define TIM1_BASE_ADDR          (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE_ADDR          (APB2PERIPH_BASE + 0x0400)
#define USART1_BASE_ADDR        (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE_ADDR        (APB2PERIPH_BASE + 0x1400)
#define ADC1_BASE_ADDR          (APB2PERIPH_BASE + 0x2000)
#define ADC2_BASE_ADDR          (APB2PERIPH_BASE + 0x2100)
#define ADC3_BASE_ADDR          (APB2PERIPH_BASE + 0x2200)
#define SDIO_BASE_ADDR          (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE_ADDR          (APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASE_ADDR        (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE_ADDR          (APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASE_ADDR          (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE_ADDR         (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE_ADDR         (APB2PERIPH_BASE + 0x4800)

/* AHB1 Peripherals */
#define GPIOA_BASE_ADDR         (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE_ADDR         (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE_ADDR         (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE_ADDR         (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE_ADDR         (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE_ADDR         (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE_ADDR         (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE_ADDR         (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE_ADDR         (AHB1PERIPH_BASE + 0x2000)
#define RCC_BASE_ADDR           (AHB1PERIPH_BASE + 0x3800)
#define FLASHITF_BASE_ADDR      (AHB1PERIPH_BASE + 0x3C00)
#define DMA1_BASE_ADDR          (AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASE_ADDR          (AHB1PERIPH_BASE + 0x6400)

/* AHB2 Peripherals */
#define USB_OTG_FS_BASE_ADDR    (AHB2PERIPH_BASE + 0x0000)

/* Peripheral Register Definition Structures */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_TypeDef;

typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    uint32_t RESERVED[2];
    volatile uint32_t CMPCR;
} SYSCFG_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_TypeDef;


#define RCC                 ((RCC_TypeDef *)RCC_BASE_ADDR)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE_ADDR)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE_ADDR)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE_ADDR)
#define GPIOD               ((GPIO_TypeDef *)GPIOD_BASE_ADDR)
#define GPIOE               ((GPIO_TypeDef *)GPIOE_BASE_ADDR)
#define GPIOF               ((GPIO_TypeDef *)GPIOF_BASE_ADDR)
#define GPIOG               ((GPIO_TypeDef *)GPIOG_BASE_ADDR)
#define GPIOH               ((GPIO_TypeDef *)GPIOH_BASE_ADDR)
#define GPIOI               ((GPIO_TypeDef *)GPIOI_BASE_ADDR)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE_ADDR)
#define SYSCFG              ((SYSCFG_TypeDef *)SYSCFG_BASE_ADDR)
#define USART1              ((USART_TypeDef *)USART1_BASE_ADDR)
#define USART2              ((USART_TypeDef *)USART2_BASE_ADDR)
#define USART3              ((USART_TypeDef *)USART3_BASE_ADDR)
#define UART4               ((USART_TypeDef *)UART4_BASE_ADDR)
#define UART5               ((USART_TypeDef *)UART5_BASE_ADDR)
#define USART6				((USART_TypeDef *)USART6_BASE_ADDR)
#define SPI1                ((SPI_TypeDef *)SPI1_BASE_ADDR)
#define SPI2                ((SPI_TypeDef *)SPI2_BASE_ADDR)
#define SPI3                ((SPI_TypeDef *)SPI3_BASE_ADDR)
#define I2C1                ((I2C_TypeDef *)I2C1_BASE_ADDR)
#define I2C2                ((I2C_TypeDef *)I2C2_BASE_ADDR)
#define I2C3                ((I2C_TypeDef *)I2C3_BASE_ADDR)

/* Clock Enable Macros */
#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()    (RCC->AHB1ENR |= (1 << 8))

/* Clock Disable Macros */
#define GPIOA_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 8))

#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= (1 << 23))

#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()   (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()   (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()    (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()    (RCC->APB1ENR |= (1 << 20))

#define SPI1_PCLK_EN()     (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()     (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()     (RCC->APB1ENR |= (1 << 15))

#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= (1 << 14))


#define I2C1_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 23))

#define USART1_PCLK_DIS()  (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS()  (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DIS()  (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 20))

#define SPI1_PCLK_DIS()    (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 15))

#define SYSCFG_PCLK_DIS()  (RCC->APB2ENR &= ~(1 << 14))

/* GPIO Code for EXTI Configuration */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : \
                                  (x == GPIOB) ? 1 : \
                                  (x == GPIOC) ? 2 : \
                                  (x == GPIOD) ? 3 : \
                                  (x == GPIOE) ? 4 : \
                                  (x == GPIOF) ? 5 : \
                                  (x == GPIOG) ? 6 : \
                                  (x == GPIOH) ? 7 : \
                                  (x == GPIOI) ? 8 : 0)

/* IRQ Numbers */
#define IRQ_EXTI0        6
#define IRQ_EXTI1        7
#define IRQ_EXTI2        8
#define IRQ_EXTI3        9
#define IRQ_EXTI4        10
#define IRQ_EXTI9_5      23
#define IRQ_EXTI15_10    40

#define IRQ_NO_SPI1		35
#define IRQ_NO_SPI2		36
#define IRQ_NO_SPI3		51

#define IRQ_NO_USART1	37
#define IRQ_NO_USART2	38
#define IRQ_NO_USART3	39
#define IRQ_NO_UART4	52
#define IRQ_NO_UART5	53
#define IRQ_NO_USART6	71

#define IRQ_NO_I2C1_EV	31
#define IRQ_NO_I2C1_ER	32
#define IRQ_NO_I2C2_EV	33
#define IRQ_NO_I2C2_ER	34
#define IRQ_NO_I2C3_EV	72
#define IRQ_NO_I2C3_ER	73

/* Generic Macros */
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET
#define FLAG_RESET       RESET
#define FLAG_SET         SET

#endif /* INC_STM32F4XX_H_ */
