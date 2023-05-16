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
 * @brief This file contains the few basic structures for API implementation,
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

    uint32_t CLKSPD;
    uint8_t  ADDR;
    uint8_t  ACKCTL;
    uint16_t FMDTYCYC;

} I2C_Config_t;

/* I2C Handle structure */
typedef struct {

    I2C_RegDef_t *pI2Cx;
    I2C_Config_t  I2C_Config;

} I2C_Handle_t;

/* Explicit I2C peripheral definitions */
#define I2C1                     ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2                     ((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3                     ((I2C_RegDef_t *) I2C3_BASEADDR)

/* Configuration Register 1 shorthands*/
#define I2C_CR1_PE               0
#define I2C_CR1_SMBUS            1
#define I2C_CR1_SMBTYPE          3
#define I2C_CR1_ENARP            4
#define I2C_CR1_ENPEC            5
#define I2C_CR1_ENGC             6
#define I2C_CR1_NOSTRETCH        7
#define I2C_CR1_START            8
#define I2C_CR1_STOP             9
#define I2C_CR1_ACK              10
#define I2C_CR1_POS              11
#define I2C_CR1_CR_PEC           12
#define I2C_CR1_ALERT            13
#define I2C_CR1_SWRST            15

/* Configuration Register 2 shorthands */
#define I2C_CR2_FREQ             0
#define I2C_CR2_ITERREN          8
#define I2C_CR2_ITEVTEN          9
#define I2C_CR2_ITBUFEN          10
#define I2C_CR2_DMAEN            11
#define I2C_CR2_LAST             12

/* Own-Address Register 1 shorthands */
#define I2C_OAR1_ADD0            0
#define I2C_OAR1_ADD1            1
#define I2C_OAR1_ADD3            8
#define I2C_OAR1_ADDMODE         15

/* Own-Address Register 2 shorthands */
#define I2C_OAR2_ENDUAL          0
#define I2C_OAR2_ADD2            1

/* Data Register shorthands */
#define I2C_DR                   0

/* Status Register 1 shorthands */
#define I2C_SR1_SB               0
#define I2C_SR1_ADDR             1
#define I2C_SR1_BTF              2
#define I2C_SR1_ADD10            3
#define I2C_SR1_STOPF            4
#define I2C_SR1_RXNE             6
#define I2C_SR1_TXE              7
#define I2C_SR1_BERR             8
#define I2C_SR1_ARLO             9
#define I2C_SR1_AF               10
#define I2C_SR1_OVR              11
#define I2C_SR1_PECERR           12
#define I2C_SR1_TIMEOUT          14
#define I2C_SR1_SMBALERT         15

/* Status Register 2 shorthands */
#define I2C_SR2_MSL              0
#define I2C_SR2_BUSY             1
#define I2C_SR2_TRA              2
#define I2C_SR2_GENCALL          4
#define I2C_SR2_SMBDEFAUL        5
#define I2C_SR2_SMBHOST          6
#define I2C_SR2_DUALF            7
#define I2C_SR2_SRPEC            8

/* Clock Control Register shorthands */
#define I2C_CCR_CCR              0
#define I2C_CCR_DUTY             14
#define I2C_CCR_FS               15

/* Rise Time Register shorthand */
#define I2C_TRISE                0

/* FIlter Register shorthands */
#define I2C_FLT_DNF              0
#define I2C_FLT_ANOFF            4

/* CI shorthands */
#define I2C_SCLK_SM              100000 /* 100 kHz */
#define I2C_SCLK_FM              400000 /* 400 kHz */
#define I2C_SCLK_FM200           200000 /* 200 kHz */

#define I2C_ACKCTL_ENABLE        1
#define I2C_ACKCTL_DISABLE       0 /* ACK is disabled by default!!! */

#define I2C_FMDTY_2              0
#define I2C_FMDTY_16_9           1

/* Flag sate macros */
#define I2C_FLAG_TXE              (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE             (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB               (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR             (1 << I2C_SR1_ADDR)

/* I2C Max Rise Times */
#define I2C_RISE_FMMAXHZ           1000000000U /* Base in Hz. The 300 comes in multiplication */
#define I2C_RISE_SMMAXHZ           1000000U 


/***************************************************************************/
/* I2C API Function Prototypes */
/***************************************************************************/

/* Peripheral clock setup */
void I2C_ClockCtl (I2C_RegDef_t *pI2Cx, uint8_t state);

/* Init/De-init */
void I2C_Init   (I2C_Handle_t *pI2CHandle);
void I2C_DeInit (I2C_RegDef_t *pI2Cx);

/* Master API */
void I2C_MasterTx (I2C_Handle_t *pI2CHandle, 
                        uint8_t *pTxBuffer, 
                        uint32_t len, 
                        uint8_t slaveAddr);

void I2C_MasterRx (void);

/* Slave API */
void I2C_SlaveTx (void);
void I2C_SlaveRx (void);

/* Interrput Handling */
void I2C_IRQConfig         (uint8_t IRQNumber, uint8_t state);
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);

/* Helper functions */
uint32_t I2C_PLLClockValue(void);
uint8_t  I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag);

static void     I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void     I2C_GenerateStop(I2C_RegDef_t *pI2Cx);
static void     I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void     I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);


/* Application callback interface */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);

#endif /* STM32F446XX_I2C_H_ */
