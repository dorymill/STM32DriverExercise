/*
 * SPITxTest.c
 *
 *  Created on: Feb 21, 2023
 *      Author: Asmod
 */


/*

@brief: Testing SPI transmission!

*/

#include "STM32f446xx.h"
#include "STM32f446xx_RCC.h"
#include "STM32f446xx_SPI.h"
#include "STM32f446xx_GPIO.h"
#include <string.h>
#include <unistd.h>


int main(void)
{

 /*
    SPI2 GPIO Pins (in Alternate Function Mode)
    AF5 on all pins
    MOSI  -> PB15
    MISO  -> PB14
    SCLK  -> PB13
    NSS   -> PB12
 */

    /* Create peripheral handles */
    GPIO_Handle_t hgpiob;
    memset(&hgpiob, 0 , sizeof(hgpiob));
    hgpiob.pGPIOx = GPIOB;

    SPI_Handle_t hspi2;
    memset(&hspi2, 0 , sizeof(hspi2));
    hspi2.pSPIx = SPI2;

    /* 
        Divide the clocks of APB1 and AHB1 to 
        get their clock speeds to < 12 MHz
        for a 24 MHz Logic Analyzer.
    */
   /* APB1 */
    RCC->CFGR |= (0b110 << 10);

    /* AHB1 */
    RCC->CFGR |= (0b1010 << 4);

    //SPI_SSI_Config(hspi2.pSPIx, SET);

    GPIO_ClockCtl(hgpiob.pGPIOx, ENABLE);
    SPI_ClockCtl(hspi2.pSPIx, ENABLE);

    /* Configure GPIO Handle (SPI2 is on GPIOB) */


    /* Base configuration */
    hgpiob.GPIO_PinConfig.GPIO_PinAltFunc = 5;
    hgpiob.GPIO_PinConfig.GPIO_PinMode    = GPIO_MODE_ALTFUNC;
    hgpiob.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OUTPUT_PP;
    hgpiob.GPIO_PinConfig.GPIO_PinPUPDCtl = GPIO_PUPD_NN;
    hgpiob.GPIO_PinConfig.GPIO_PinSpeed   = GPIO_SPEED_FA;

    /* SCLK */
    hgpiob.GPIO_PinConfig.GPIO_PinNumber = 13;
    GPIO_Init(&hgpiob);

    /* MOSI */
    hgpiob.GPIO_PinConfig.GPIO_PinNumber = 15;
    GPIO_Init(&hgpiob);

    /* MISO */
    hgpiob.GPIO_PinConfig.GPIO_PinNumber = 14;
    GPIO_Init(&hgpiob);

    /* NSS */
    hgpiob.GPIO_PinConfig.GPIO_PinNumber = 12;
    hgpiob.GPIO_PinConfig.GPIO_PinMode    = GPIO_MODE_OUT;
    hgpiob.GPIO_PinConfig.GPIO_PinPUPDCtl = GPIO_PUPD_PU; /* If we bit bang via hardware */
    GPIO_Init(&hgpiob);


    /* Configure SPI Handle*/ 
    hspi2.SPI_Config.MSTR     = SPI_MSTR_MASTER;
    hspi2.SPI_Config.DFF      = SPI_DFF_8BIT;
    hspi2.SPI_Config.BIDIMODE = SPI_BIDIMODE_FD;
    hspi2.SPI_Config.SSM      = SPI_SSM_HW;
    hspi2.SPI_Config.SSI      = SPI_SSM_HW;
    hspi2.SPI_Config.SPE      = SPI_SPE_EN;
    
    //SSOE Enable
    hspi2.pSPIx->CR2 |= (ENABLE << 2);

    SPI_Init(&hspi2);

    /* Send data! */
    char * message = "In the sky there is nobody asleep. Nobody, nobody. Nobody is asleep. The creatures of the moon sniff and prowl about their cabins.";

    GPIO_WritePin(hgpiob.pGPIOx, 12, 0);
    SPI_Write(hspi2.pSPIx, (uint8_t *) message, (uint32_t) strlen(message));
    delay(1);
    GPIO_WritePin(hgpiob.pGPIOx, 12, 1);

    while(1);
     
    return 0;
}



