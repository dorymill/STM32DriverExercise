/*
 * 004I2CTxTest.c
 *
 *  Created on: May 15, 2023
 *      Author: Asmod
 */


#include <QMC5883L.h>
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"
#include "STM32f446xx.h"
#include <string.h>
#include <unistd.h>

/**
 * @brief Testing I2C Master Tx transactions!
 * 
 */

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

    CompassHandle_t hcompass;

    GPIO_ClockCtl(hgpiob.pGPIOx, ENABLE);
    I2C_ClockCtl(hi2c2.pI2Cx, ENABLE);


    /* Base configuration */
    hgpiob.GPIO_PinConfig.GPIO_PinAltFunc = 4;
    hgpiob.GPIO_PinConfig.GPIO_PinMode    = GPIO_MODE_ALTFUNC;
    hgpiob.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OUTPUT_OD;
    hgpiob.GPIO_PinConfig.GPIO_PinPUPDCtl = GPIO_PUPD_PU;
    hgpiob.GPIO_PinConfig.GPIO_PinSpeed   = GPIO_SPEED_FA;

    /* SCLK */
    hgpiob.GPIO_PinConfig.GPIO_PinNumber = 10;
    GPIO_Init(&hgpiob);

    /* SDA */
    hgpiob.GPIO_PinConfig.GPIO_PinNumber = 3;
    GPIO_Init(&hgpiob);

    /* Init the I2C handle */
    hi2c2.I2C_Config.ACKCTL   = I2C_ACKCTL_ENABLE;
    hi2c2.I2C_Config.CLKSPD   = I2C_SCLK_FM;
    hi2c2.I2C_Config.FMDTYCYC = I2C_FMDTY_2;
    hi2c2.I2C_Config.ADDR     = 0x61; /* Dummy addr */

    I2C_Init(&hi2c2);

    /* Set compass configuration structure. */
    hcompass.compConfig.averaging = COMPASS_MA_AVG1;
    hcompass.compConfig.output_rate = COMPASS_DSO_15Hz;
    hcompass.compConfig.bias_control = COMPASS_MS_NORMAL;
    hcompass.compConfig.gain_control = COMPASS_GAIN_P1P9G;
    hcompass.compConfig.op_mode      = COMPASS_MODE_SNGL;
    hcompass.compConfig.hsbit        = 0;

    InitCompass(&hi2c2, &hcompass, hcompass.compConfig);

    return 0;
}
