/*
 * STM32f446xx_RCC.h
 *
 *  Created on: Jan 23, 2023
 *      Author: Asmod
 */
#include "STM32f446xx.h"

#ifndef STM32F446XX_RCC_H_
#define STM32F446XX_RCC_H_

#include <stdint.h>

#define __vo volatile

/**
 * @brief This file contains the macro for the pointer to 
 *        the Reset and Clock Control (RCC) peripheral, 
 *        its bus structure struct, and functional macros to
 *        enable, disable, and reset the various peripherals
 *        and their clocks.
 * 
 */

/* RCC Register definition structure */
typedef struct {

    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
         uint32_t RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
         uint32_t RESERVED1;
         uint32_t RESERVED2;
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
         uint32_t RESERVED3;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
         uint32_t RESERVED4;
         uint32_t RESERVED5;
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
         uint32_t RESERVED6;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
         uint32_t RESERVED7;
         uint32_t RESERVED8;
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
         uint32_t RESERVED9;
         uint32_t RESERVED10;
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
    __vo uint32_t CKGATENR;
    __vo uint32_t DCKCFGR2;

} RCC_RegDef_t;


/* Explicit RCC peripheral definition */
#define RCC                      ((RCC_RegDef_t *) RCC_BASEADDR)

/* RCC Macros */

/* Enable/Disable GPIOx clocks */
#define GPIOA_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLOCK_EN()         (RCC->AHB1ENR |= (1 << 7))

#define GPIOA_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLOCK_DI()         (RCC->AHB1ENR &= ~(1 << 7))

/* Enable/Disable I2C clocks */
#define I2C1_CLOCK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_CLOCK_EN()          (RCC->APB1ENR |= (1 << 22))
#define I2C3_CLOCK_EN()          (RCC->APB1ENR |= (1 << 23))

#define I2C1_CLOCK_DI()         (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLOCK_DI()         (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLOCK_DI()         (RCC->APB1ENR &= ~(1 << 23))

/* Enable/Disable U(S)ART clocks */
#define USART1_CLOCK_EN()        (RCC->APB2ENR |= (1 << 4) )
#define USART2_CLOCK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART3_CLOCK_EN()        (RCC->APB1ENR |= (1 << 18))
#define UART4_CLOCK_EN()         (RCC->APB1ENR |= (1 << 19))
#define UART5_CLOCK_EN()         (RCC->APB1ENR |= (1 << 20))

#define USART1_CLOCK_DI()        (RCC->APB2ENR &= ~(1 << 4) )
#define USART2_CLOCK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLOCK_DI()        (RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLOCK_DI()         (RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLOCK_DI()         (RCC->APB1ENR &= ~(1 << 20))

/* Enable/Disable SPI clocks */
#define SPI1_CLOCK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_CLOCK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_CLOCK_EN()          (RCC->APB1ENR |= (1 << 15))
#define SPI4_CLOCK_EN()          (RCC->APB2ENR |= (1 << 13))

#define SPI1_CLOCK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLOCK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLOCK_DI()          (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_CLOCK_DI()          (RCC->APB2ENR &= ~(1 << 13))

/* Enable/Disable ADC clocks */
#define ADC1_CLOCK_EN()          (RCC->APB2ENR |= (1 << 8) )
#define ADC2_CLOCK_EN()          (RCC->APB2ENR |= (1 << 9) )
#define ADC3_CLOCK_EN()          (RCC->APB2ENR |= (1 << 10))

#define ADC1_CLOCK_DI()          (RCC->APB2ENR &= ~(1 << 8) )
#define ADC2_CLOCK_DI()          (RCC->APB2ENR &= ~(1 << 9) )
#define ADC3_CLOCK_DI()          (RCC->APB2ENR &= ~(1 << 10))

/* Enable/Disable SYSCFG Clock */
#define SYSCFG_CLOCK_EN()        (RCC->APB2ENR |= (1 << 14))

#define SYSCFG_CLOCK_DI()        (RCC->APB2ENR &= ~(1 << 14))


/* GPIO Reset Macros */
#define GPIOA_RESET()            do{(RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_RESET()            do{(RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_RESET()            do{(RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_RESET()            do{(RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_RESET()            do{(RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_RESET()            do{(RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_RESET()            do{(RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_RESET()            do{(RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

/* SPI Reset Macros */
#define SPI1_RESET()            do{(RCC->APB2RSTR |= (1 << 12));  (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_RESET()            do{(RCC->APB1RSTR |= (1 << 14));  (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_RESET()            do{(RCC->APB1RSTR |= (1 << 15));  (RCC->APB1RSTR &= ~(1 << 15));} while(0)
#define SPI4_RESET()            do{(RCC->APB2RSTR |= (1 << 13));  (RCC->APB2RSTR &= ~(1 << 13));} while(0)

/* I2C Reset Macros */
#define I2C1_RESET()            do{(RCC->APB1RSTR |= (1 << 21));  (RCC->APB1RSTR &= ~(1 << 21));} while(0)
#define I2C2_RESET()            do{(RCC->APB1RSTR |= (1 << 22));  (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C3_RESET()            do{(RCC->APB1RSTR |= (1 << 23));  (RCC->APB1RSTR &= ~(1 << 23));} while(0)

#endif /* STM32F446XX_RCC_H_ */
