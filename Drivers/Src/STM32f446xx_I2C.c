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

void I2C_Init   (I2C_Handle_t *pI2CHandle)
{

    /* Set configuration registers */
    uint32_t temp = 0;


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