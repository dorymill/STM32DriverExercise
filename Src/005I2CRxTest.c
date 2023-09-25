/*
 * 005I2CRxTest.c
 *
 *  Created on: Sep 12, 2023
 *      Author: Asmod
 */


#include <QMC5883L.h>
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"
#include "STM32f446xx.h"
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <malloc.h>

/**
 * @brief Testing I2C Master Rx transactions!
 * 
 */

/* Globals */
#define ARDUINO_SLAVE 0x68
#define CMD_GET_LEN   0x51
#define CMD_GET_DAT   0x52

/* Static helper functions */
static void initGPIO   (GPIO_Handle_t *pGPIOHandle);
static void initI2C    (I2C_Handle_t  *pI2CHandle);
static void initClocks (GPIO_Handle_t *pGPIOHandle, 
                        I2C_Handle_t  *pI2CHandle);

int main(void)
{
    /*
    I2C2 GPIO Pins (in Alternate Function Mode)
    AF4 on all pins
    SCL  -> PB10
    SDA  -> PB3

 */

    /* Create peripheral handles */
    GPIO_Handle_t hgpiob;
    memset(&hgpiob, 0 , sizeof(hgpiob));
    hgpiob.pGPIOx = GPIOB;

    I2C_Handle_t hi2c2;
    memset(&hi2c2, 0 , sizeof(hi2c2));
    hi2c2.pI2Cx = I2C2;

    /* Initialize Clocks */
    initClocks (&hgpiob, &hi2c2);

    /* Initialize GPIO */
    initGPIO (&hgpiob);

    /* Initialize I2C */
    initI2C(&hi2c2);

    /* Main logic */

    /* Key vars */
    uint8_t msgLen = 0;
    uint8_t txBuffer =  CMD_GET_LEN;

    /* Send Command Code 0x51 to get message length */
    I2C_MasterTx (&hi2c2, &txBuffer, 1, ARDUINO_SLAVE);
    I2C_MasterRx (&hi2c2, &msgLen, 1, ARDUINO_SLAVE);

    /* Now allocate an rxBuffer that's the right size. */
    uint8_t *rxBuffer = (uint8_t *) malloc(msgLen*sizeof(uint8_t));

    /* Now send the command to get the data and receive it. */
    txBuffer = CMD_GET_DAT;
    I2C_MasterTx (&hi2c2, &txBuffer, 1, ARDUINO_SLAVE);
    I2C_MasterRx (&hi2c2, rxBuffer, msgLen, ARDUINO_SLAVE);

}

static 
void initGPIO   (GPIO_Handle_t *pGPIOHandle)
{
    /* Base configuration */
    pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunc = 4;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinMode    = GPIO_MODE_ALTFUNC;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  = GPIO_OUTPUT_OD;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDCtl = GPIO_PUPD_PU; /* NN | External Pull-Up: 2.2 kΩ */
    pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed   = GPIO_SPEED_FA;

    /* SCLK */
    pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = 10;
    GPIO_Init(pGPIOHandle);

    /* SDA */
    pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = 3;
    GPIO_Init(pGPIOHandle);

}

static 
void initI2C    (I2C_Handle_t  *pI2CHandle)
{
    /* Init the I2C handle */
    pI2CHandle->I2C_Config.CLKSPD   = I2C_SCLK_SM;
    pI2CHandle->I2C_Config.FMDTYCYC = I2C_FMDTY_2;
    pI2CHandle->I2C_Config.ADDR     = 0x65; /* Dummy addr */

    I2C_Init(pI2CHandle);

    /* Enable acking */
    //I2C_ACKControl(pI2CHandle->pI2Cx, I2C_ACKCTL_ENABLE);
}

static 
void initClocks (GPIO_Handle_t *pGPIOHandle, 
                        I2C_Handle_t  *pI2CHandle)
{
    /* Wind the clocks */
    GPIO_ClockCtl(pGPIOHandle->pGPIOx, ENABLE);
    I2C_ClockCtl(pI2CHandle->pI2Cx, ENABLE);

}
