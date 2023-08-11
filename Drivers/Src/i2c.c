/*
 * STM32f446xx_I2C.c
 *
 *  Created on: Apr 5, 2023
 *      Author: Asmod
 */

#include <exti.h>
#include <i2c.h>
#include <rcc.h>
#include <syscfg.h>

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
 * @brief Static function protoypes
 * 
 */
static void     I2C_ACKControl   (I2C_RegDef_t *pI2Cx, uint8_t ACKNACK);
static void     I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
static void     I2C_GenerateStop (I2C_RegDef_t *pI2Cx);
static void     I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);
static void     I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, 
                                        uint8_t slaveAddr, 
                                        uint8_t RxTx);

/**
 * @brief Initialization functions
 * 
 */
 
/**
 * @brief Enable and disable the clock for the provided I2C
 *        peripheral.
 * 
 * @param pI2C I2C register pointer
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

/**
 * @brief Calculates the pll clock frequency required to
 *        configure the I2C CCR.
 * 
 * @return uint32_t Clock frequency
 */
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
    temp = ((RCC->CFGR >> 4) & 0x0F);

    if (temp < 8)
    {
        ahbpr = 1;
    } 
    else 
    {
        ahbpr = AHB_Prescaler[temp - 8];
    }

    /* Determine APB1 Clock Prescaler */
    temp = ((RCC->CFGR >> 10) & 0x07);

    if(temp < 4)
    {
        apbpr = 1;
    }
    else
    {
        apbpr = APB_Prescaler[temp - 4];
    }

    /* Calculate final clock value to pass to I2C register */
    result = (sysclock / ahbpr) / apbpr;

    return result;
}

/**
 * @brief Initializes the I2C peripheral.
 * 
 * @param pI2CHandle Handle for the I2C peripheral 
 */
void I2C_Init   (I2C_Handle_t *pI2CHandle)
{

    /* Set configuration registers */
    uint32_t temp  = 0;

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
    /* The resistance range of the pull-up resistors needed
       can be calculated as
       Rmin = (Vcc - Vol(max) )/ Iol 
       Rmax = t_rise / (0.8473*Cb)  
       Where, Vcc=power line voltage, Vol(max) is
       line low voltage maximum output, Iol is low level
       output current, t_rise is the rise time, and Cb is
       the bus capacitve load. 
       */
    if(pI2CHandle->I2C_Config.CLKSPD <= I2C_SCLK_SM) /* Standard mode */
    {
        temp = (I2C_PLLClockValue() / I2C_RISE_SMMAXHZ) + 1;
    } 
    else /* Fast mode */
    {
        temp = ((I2C_PLLClockValue() * 300) / I2C_RISE_FMMAXHZ) + 1;
    }

    pI2CHandle->pI2Cx->TRISE = temp & 0x3F;

    /* Enable peripheral */ 
    pI2CHandle->pI2Cx->CR1 = (1 << 0);

}

/**
 * @brief De-initializes the I2C peripheral.
 * 
 * @param pI2C I2C register pointer 
 */
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

/**
 * @brief Transmits data from device as master.
 * 
 * @param pI2CHandle Handle for the I2C peripheral
 * @param pTxBuffer  Tx buffer
 * @param len        Number of bytes to send
 * @param slaveAddr  Address of slave device
 */
void I2C_MasterTx (I2C_Handle_t *pI2CHandle, 
                        uint8_t *pTxBuffer, 
                        uint32_t len, 
                        uint8_t slaveAddr)
{

    /* Retry counter */
    // uint8_t retry = 0;

    /* Generate START condition */
    I2C_GenerateStart(pI2CHandle->pI2Cx);

    /* Confirm start by checking SB flag */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    /* Send slave address */
    I2C_ExecAddrPhase(pI2CHandle->pI2Cx, slaveAddr, TX);

    /* Confirm SR1 address field */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) {
        
        // Add retry handling here otherwise we'll hang.

    }

    /* Clear addr flag according to SW sequence */
    I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

    /* Send data until length is zero */
    while(len > 0)
    {
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        len--;

        // Make sure that very last byte gets out.
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    }

    /* Generate STOP condition */
    I2C_GenerateStop(pI2CHandle->pI2Cx);

}


void I2C_MasterRx (I2C_Handle_t *pI2CHandle, 
                        uint8_t *pRxBuffer, 
                        uint32_t len, 
                        uint8_t  slaveAddr)
{
    /* Generate START condition */
    I2C_GenerateStart(pI2CHandle->pI2Cx);

    /* Confirm via checking SB flag */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    /* Send slave address with R/W bit, read is 1 */
    I2C_ExecAddrPhase(pI2CHandle->pI2Cx, slaveAddr, RX);

    /* Wait for ADDR flag */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    /* Single Byte Read */
    if (len == 1) {

        /* Disable ACK */
        I2C_ACKControl(pI2CHandle->pI2Cx, NACK);

        /* Generate STOP */
        I2C_GenerateStop(pI2CHandle->pI2Cx);

        /* Clear ADDR flag */
        I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

        /* Wait for RXNE */
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        /* Read data */
        pRxBuffer = (uint8_t *) pI2CHandle->pI2Cx->DR;

    }

    /* Multi byte read */
    if (len > 1)
    {
        /* Clear ADDR flag */
        I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

        /* Read! */
        for(uint32_t iter = len; iter > 0; iter--)
        {
            /* Wait for RXNE to go active */
            while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

            if(iter == 2)
            {
                /* Disable ACK */
                I2C_ACKControl(pI2CHandle->pI2Cx, NACK);

                /* Generate STOP */
                I2C_GenerateStop(pI2CHandle->pI2Cx);
            }

            /* Read data */
            pRxBuffer = (uint8_t *) pI2CHandle->pI2Cx->DR;
            pRxBuffer++;
            len--;

        }
    }

    /* Re-enable ACK */
    if(pI2CHandle->I2C_Config.ACKCTL == ACK)
    {
        I2C_ACKControl(pI2CHandle->pI2Cx, ACK);
    }
    else
    {
        I2C_ACKControl(pI2CHandle->pI2Cx, NACK);
    }
}

 

/**
 * @brief Helper functions
 * 
 */

/**
 * @brief Generates start condition
 * 
 * @param pI2C I2C register pointer 
 */
void     I2C_GenerateStart(I2C_RegDef_t *pI2Cx)
{
    /* Generate start condition */
    pI2Cx->CR1 |= (1 << I2C_CR1_START);

}

/**
 * @brief Generates stop condition
 * 
 * @param pI2C I2C register pointer 
 */
void     I2C_GenerateStop(I2C_RegDef_t *pI2Cx)
{
    /* Generate start condition */
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}

/**
 * @brief Checks flag status and returns true if it's on.
 * 
 * @param pI2C I2C register pointer 
 * @param flag Flag to check
 * @return uint8_t 
 */
uint8_t  I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag)
{
    if(pI2Cx->SR1 & flag)
    {
        return SET;
    }

    return RESET;
}

/**
 * @brief Executes the address phase of I2C communication.
 * 
 * @param pI2C      I2C register pointer 
 * @param slaveAddr Address of slave device
 * @param RxTx      Receive or transmit flag
 */
void     I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, 
                                 uint8_t slaveAddr, 
                                 uint8_t RxTx)
{
    /* Make space for the R/W bit! */
    slaveAddr = slaveAddr << 1;

    /* Write bit is 0 */
    if(RxTx == TX)
    {
        slaveAddr &= ~(1);
    } else {
        slaveAddr |= 1;
    }

    /* Send it! */
    pI2Cx->DR = slaveAddr;
}

/**
 * @brief Executes the clear address phase of I2C communication.
 * 
 * @param pI2C I2C register pointer 
 */
void     I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
    uint32_t dummy = pI2Cx->SR1;
             dummy = pI2Cx->SR2;

    (void) dummy;
}

/**
 * @brief 
 * 
 * @param pI2Cx 
 * @param ACKNACK 
 */
void     I2C_ACKControl   (I2C_RegDef_t *pI2Cx, uint8_t ACKNACK)
{
    if(ACKNACK == ACK)
    {
        pI2Cx->CR1 |= (ACK << 10);
    } 
    else
    {
        pI2Cx->CR1 |= (NACK << 10);
    }
}
