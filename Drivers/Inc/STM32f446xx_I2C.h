/*
 * STM32f446xx_I2C.h
 *
 *  Created on: Apr 5, 2023
 *      Author: Asmod
 */

#include "STM32f446xx.h"

#ifndef STM32F446XX_I2C_H_
#define STM32F446XX_I2C_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/***************************************************************************/
/* I2C API Header */
/***************************************************************************/

/**
 * @brief This files contains the few basic structures for API implementation,
 *        macros for the base addresses of the I2C peripherals, macros to define
 *        the various configuration and mode settings, and lastly, the beloved 
 *        function prototypes.
 * 
 */

/* I2C Register strcuture */
typedef struct {

    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t OAR1;
    __vo uint32_t OAR2;
    __vo uint32_t DR;
    __vo uint32_t SR1;
    __vo uint32_t SR2;
    __vo uint32_t CCR;
    __vo uint32_t TRISE;
    __vo uint32_t FLTR;

} I2C_RegDef_t;

/* I2C Configuration structure */
typedef struct {

    I2C_RegDef_t *i2Cx;
    I2C_Config_t  I2C_Config;

} I2C_Config_t;

/* I2C Handle structure */

/* Explicit I2C peripheral definitions */
#define I2C1                     ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2                     ((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3                     ((I2C_RegDef_t *) I2C3_BASEADDR)


#endif /* STM32F446XX_I2C_H_ */
