/*
 * STM32f446xx_GPIO.h
 *
 *  Created on: Jan 23, 2023
 *      Author: Asmod
 */

#include "STM32f446xx.h"

#ifndef STM32F446XX_GPIO_H_
#define STM32F446XX_GPIO_H_


#include <stdint.h>

#define __vo volatile

/***************************************************************************/
/* GPIO API Header */
/***************************************************************************/

/**
 * @brief This files contains the few basic structures for API implementation,
 *        macros for the base addresses of the GPIO peripherals, macros to define
 *        the various configuration and mode settings, and lastly, the beloved 
 *        function prototypes.
 * 
 */

/* GPIO Register definition structure */
typedef struct {

    __vo uint32_t MODER;
    __vo uint32_t OTYPER;
    __vo uint32_t OSPEEDR;
    __vo uint32_t PUPDR;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t LCKR;
    __vo uint32_t AFRL;
    __vo uint32_t AFRH;

} GPIO_RegDef_t;

/* GPIO Pin Configuration structure */
typedef struct {

    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPUPDCtl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunc;

} GPIO_PinConfig_t;

/* GPIO Handle structure */
typedef struct {

    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;



/* Explicit GPIO peripheral definitions */
#define GPIOA                    ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB                    ((GPIO_RegDef_t *) GPIOB_BASEADDR) 
#define GPIOC                    ((GPIO_RegDef_t *) GPIOC_BASEADDR) 
#define GPIOD                    ((GPIO_RegDef_t *) GPIOD_BASEADDR) 
#define GPIOE                    ((GPIO_RegDef_t *) GPIOE_BASEADDR) 
#define GPIOF                    ((GPIO_RegDef_t *) GPIOF_BASEADDR) 
#define GPIOG                    ((GPIO_RegDef_t *) GPIOG_BASEADDR) 
#define GPIOH                    ((GPIO_RegDef_t *) GPIOH_BASEADDR)

/* Generic macros */
/* State Macros */
#define ENABLE                   1
#define DISABLE                  0
#define SET                      ENABLE
#define RESET                    DISABLE

/* @GPIO_PIN_MODES */
/* Mode Macros */
#define GPIO_MODE_IN             0
#define GPIO_MODE_OUT            1
#define GPIO_MODE_ALTFUNC        2
#define GPIO_MODE_ANALOG         3
#define GPIO_MODE_INT_FT         4
#define GPIO_MODE_INT_RT         5   
#define GPIO_MODE_INT_FTRT       6

/* Pin Output Types */
#define GPIO_OUTPUT_PP           0
#define GPIO_OUTPUT_OD           1

/* Pin Speeds */
#define GPIO_SPEED_LO            0
#define GPIO_SPEED_MED           1
#define GPIO_SPEED_FA            2
#define GPIO_SPEED_HS            3

/* Pull-Up/Pull-Down Resistor Settings */
#define GPIO_PUPD_NN             0
#define GPIO_PUPD_PU             1
#define GPIO_PUPD_PD             2

/* Function Macro */
#define GPIO_BASEADDR_TO_CODE(x)     (x == GPIOA) ? 0 :\
                                     (x == GPIOB) ? 1 :\
                                     (x == GPIOC) ? 2 :\
                                     (x == GPIOD) ? 3 :\
                                     (x == GPIOE) ? 4 :\
                                     (x == GPIOF) ? 5 :\
                                     (x == GPIOG) ? 6 :\
                                     (x == GPIOH) ? 7 :\

/***************************************************************************/
/* GPIO API Function Prototypes */
/***************************************************************************/

/* Peripheral clock setup */
void GPIO_ClockCtl (GPIO_RegDef_t *pGPIOx, uint8_t state);

/* Init/De-init */
void GPIO_Init   (GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx);

/* I/O */
uint8_t GPIO_ReadPin   (GPIO_RegDef_t *pGPIOx, uint8_t pin);

uint16_t GPIO_ReadPort (GPIO_RegDef_t *pGPIOx);

void GPIO_WritePin     (GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t val);

void GPIO_WritePort    (GPIO_RegDef_t *pGPIOx, uint16_t val);
void GPIO_TogglePin    (GPIO_RegDef_t *pGPIOx, uint8_t pin);

/* Interrput Handling */
void GPIO_IRQConfig         (uint8_t IRQNumber, uint8_t state);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling       (uint8_t pin);

void delay (uint32_t val);

#endif /* STM32F446XX_GPIO_H_ */
