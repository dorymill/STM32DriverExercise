/*
 * STM32f446xx_EXTI.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Asmod
 */


#include "STM32f446xx.h"

#ifndef STM32F446XX_EXTI_H_
#define STM32F446XX_EXTI_H_

#include <stdint.h>

#define __vo volatile

/**
 * @brief This files contains a macro for the pointer to the external
 *        interrupt bus, as well as the register structure for the bus
 *        and basic state macros.
 * 
 */


/* EXTI Register Structure */
typedef struct {

    __vo uint32_t IMR;
    __vo uint32_t EMR;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;

} EXTI_RegDef_t;


/* EXTI explicit definition */
#define EXTI                     ((EXTI_RegDef_t *) EXTI_BASEADDR)

/* State Macros */
#define ENABLE                   1
#define DISABLE                  0
#define SET                      ENABLE
#define RESET                    DISABLE

#endif /* STM32F446XX_EXTI_H_ */
