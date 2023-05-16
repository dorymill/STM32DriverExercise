/*
 * STM32f446xx_I2C.c
 *
 *  Created on: Apr 5, 2023
 *      Author: Asmod
 */

#include "STM32f446xx_I2C.h"
#include "STM32f446xx_RCC.h"
#include "STM32f446xx_EXTI.h"
#include "STM32f446xx_SYSCFG.h"

/***************************************************************************/
/* I2C API Function Implementations */
/***************************************************************************/

/**
 * @brief Initaliztion structures
 * 
 */

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t  APB_Prescaler[4] = {2,4,8,16};


/**
 * @brief Initialization functions
 * 
 */

/**
 * @brief Enable and disable the clock for the provided I2C
 *        peripheral.
 * 
 * @param pI2Cx I2C register pointer
 * @param state Clock state
 */
void I2C_ClockCtl (I2C_RegDef_t *pI2Cx, uint8_t state)
{
    if(state == ENABLE){

        if(pI2Cx == I2C1) {
            I2C1_CLOCK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_CLOCK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_CLOCK_EN();
        }

    } else {

        if(pI2Cx == I2C1) {
            I2C1_CLOCK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_CLOCK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_CLOCK_DI();
        } 
    }
}

uint32_t I2C_PLLClockValue(void)
{
    /* Using the RCC registers, we can calculate the 
       clock speed and configure the return the required
       parameter to the init API. */
    
    uint32_t result, sysclock = 0;

    uint8_t clcksrc, temp, ahbpr, apbpr = 0;

    /* Check RCC_CFGR Sytem clock switch status */
    /* Determine clock source */
    clcksrc = (RCC->CFGR >> 2) & 0x03;

    if (clcksrc == 0) /* HSI selected */
    {
        sysclock = 16000000;
    } 
    else if (sysclock == 1) /* HSE selected */
    {
        sysclock = 8000000;
    } 
    else if (sysclock == 2) /* PLL_P selected */
    {
        // Not implemented
        //sysclock = RCC_GetPLLOutputClock();
        
    } 
    else if (sysclock == 3) /* PLL_R selected */
    {
        // Not implemented
        //sysclock = RCC_GetPLLOutputClock();
    }

    /* Determine AHB Clock Prescaler */
    temp = (RCC->CFGR >> 4) & 0x0F;

    if (temp < 8)
    {
        ahbpr = 1;
    } 
    else 
    {
        ahbpr = AHB_Prescaler[temp - 8];
    }

    /* Determine APB1 Clock Prescaler */
    temp = (RCC->CFGR >> 10) & 0x07;

    if(temp < 4)
    {
        apbpr = 1;
    }
    else
    {
        apbpr = APB_Prescaler[temp -4];
    }

    /* Calculate final clock value to pass to I2C register */
    result = (sysclock / ahbpr) / apbpr;

    return result;
}

void I2C_Init   (I2C_Handle_t *pI2CHandle)
{

    /* Set configuration registers */
    uint32_t temp = 0;

    /* ACK Control */
    temp |= (pI2CHandle->I2C_Config.ACKCTL << 10);
    pI2CHandle->pI2Cx->CR1 = temp;

    /* Set FREQ */
    temp = 0;
    temp |= I2C_PLLClockValue() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (temp & 0x3F) << 0;

    /* Set OAR (10 bit mode not implemented)*/
    temp = 0;
    temp |= (pI2CHandle->I2C_Config.ADDR << 1);
    temp |= (1 << 14); /* Required per page 784 (24.6.3) of reference manual. */
    pI2CHandle->pI2Cx->OAR1 = temp;

    /* Calculated and set CCR Field */
    uint16_t ccr = 0;

    temp = 0;

    if(pI2CHandle->I2C_Config.CLKSPD <= I2C_SCLK_SM) /* Standard mode */
    {
        ccr = I2C_PLLClockValue() / (2 * pI2CHandle->I2C_Config.CLKSPD);
        temp |= (ccr & 0xFFF);
    } 
    else /* Fast mode */
    {
        temp |= (1 << 15);
        temp |= (pI2CHandle->I2C_Config.FMDTYCYC << 14);

        if(pI2CHandle->I2C_Config.FMDTYCYC == I2C_FMDTY_2)
        {
            ccr = (I2C_PLLClockValue() / (3 * pI2CHandle->I2C_Config.CLKSPD));
        }
        else
        {
            ccr = (I2C_PLLClockValue() / (25 * pI2CHandle->I2C_Config.CLKSPD));
        }

        temp |= (ccr & 0xFFF);
    }

    pI2CHandle->pI2Cx->CCR = temp;

    /* Set Rise-Time register */
    /* To be implemented later. */

}

void I2C_DeInit (I2C_RegDef_t *pI2Cx)
{
        if(pI2Cx == I2C1){
    		I2C1_RESET ();
    	} else if (pI2Cx == I2C2) {   
            I2C2_RESET ();
        } else if (pI2Cx == I2C3) {
            I2C3_RESET ();
        }
}