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

/* Configuration Register 1 shorthands*/
#define I2C_PE                   0
#define I2C_SMBUS                1
#define I2C_SMBTYPE              3
#define I2C_ENARP                4
#define I2C_ENPEC                5
#define I2C_ENGC                 6
#define I2C_NOSTRETCH            7
#define I2C_START                8
#define I2C_STOP                 9
#define I2C_ACK                  10
#define I2C_POS                  11
#define I2C_CR_PEC                  12
#define I2C_ALERT                13
#define I2C_SWRST                15

/* Configuration Register 2 shorthands */
#define I2C_FREQ                 0
#define I2C_ITERREN              8
#define I2C_ITEVTEN              9
#define I2C_ITBUFEN              10
#define I2C_DMAEN                11
#define I2C_LAST                 12

/* Own-Address Register 1 shorthands */
#define I2C_ADD0                 0
#define I2C_ADD1                 1
#define I2C_ADD3                 8
#define I2C_ADDMODE              15

/* Own-Address Register 2 shorthands */
#define I2C_ENDUAL               0
#define I2C_ADD2                 1

/* Data Register shorthands */
#define I2C_DR                   0

/* Status Register 1 shorthands */
#define I2C_SB                   0
#define I2C_ADDR                 1
#define I2C_BTF                  2
#define I2C_ADD10                3
#define I2C_STOPF                4
#define I2C_RxNE                 6
#define I2C_TxE                  7
#define I2C_BERR                 8
#define I2C_ARLO                 9
#define I2C_AF                   10
#define I2C_OVR                  11
#define I2C_PECERR               12
#define I2C_TIMEOUT              14
#define I2C_SMBALERT             15

/* Status Register 2 shorthands */
#define I2C_MSL                  0
#define I2C_BUSY                 1
#define I2C_TRA                  2
#define I2C_GENCALL              4
#define I2C_SMBDEFAUL            5
#define I2C_SMBHOST              6
#define I2C_DUALF                7
#define I2C_SRPEC                8

/* Clock Control REgister shorthands */
#define I2C_CCR                  0
#define I2C_DUTY                 14
#define I2C_FS                   15

/* Rise Time Register shorthand */
#define I2C_TRISE                0

/* FIlter Register Shorthands */
#define I2C_DNF                  0
#define I2C_ANOFF                4


#endif /* STM32F446XX_I2C_H_ */
