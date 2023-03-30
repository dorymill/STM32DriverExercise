/*
 * STM32f446xx_SPI.c
 *
 *  Created on: Feb 19, 2023
 *      Author: Asmod
 */

#include "STM32f446xx_SPI.h"
#include "STM32f446xx_RCC.h"
#include "STM32f446xx_EXTI.h"
#include "STM32f446xx_SYSCFG.h"


/***************************************************************************/
/* SPI API Function Implementations */
/***************************************************************************/


/* Peripheral clock setup */
void SPI_ClockCtl (SPI_RegDef_t *pSPIx, uint8_t state)
{
    if(state == ENABLE){

        if(pSPIx == SPI1) {
            SPI1_CLOCK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_CLOCK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_CLOCK_EN();
        } else if (pSPIx == SPI4) {
            SPI4_CLOCK_EN();
        }


    } else {

        if(pSPIx == SPI1) {
            SPI1_CLOCK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_CLOCK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_CLOCK_DI();
        } else if (pSPIx == SPI4) {
            SPI4_CLOCK_DI();
        }

    }
}

/* Init/De-init */
void SPI_Init   (SPI_Handle_t *pSPIHandle)
{

    /* Set configuration registers! */
    uint32_t temp = 0;

    /* Device Mode (Master/Slave) */
    temp |= (pSPIHandle->SPI_Config.MSTR << SPI_MSTR);

    /* Set proper duplexing mode */
    if (pSPIHandle->SPI_Config.BIDIMODE == SPI_BIDIMODE_FD)
    {
        /* Clear BIDIMODE bit */
        temp &= ~(0x1U << SPI_BIDIMODE);


    } else if (pSPIHandle->SPI_Config.BIDIMODE == SPI_BIDIMODE_HD)
    {
        /* Set BIDIMODE*/
        temp |= (0x1U << SPI_BIDIMODE);

    } else if (pSPIHandle->SPI_Config.BIDIOE == SPI_BIDIOE_RXONLY)
    {
        /* Clear BIDIMODE*/
        temp &= ~(0x1U << SPI_BIDIMODE);
        
        /* Set RXONLY */
        temp |= (0x1U << SPI_RXONLY);
    }

    /* CPHA Setting */
    temp |= (pSPIHandle->SPI_Config.CPHA << SPI_CPHA);

    /* CPOL Setting */
    temp |= (pSPIHandle->SPI_Config.CPOL << SPI_CPOL);

    /* Baud Rate Setting */
    temp |= (pSPIHandle->SPI_Config.BR << SPI_BR);

    /* SPE Setting */
    temp |= (pSPIHandle->SPI_Config.SPE << SPI_SPE);

    /* LSBFIRST Setting */
    temp |= (pSPIHandle->SPI_Config.LSBFIRST << SPI_LSBFIRST);

    /* SSI Setting */
    temp |= (pSPIHandle->SPI_Config.SSI << SPI_SSI);

    /* SSM Setting */
    temp |= (pSPIHandle->SPI_Config.SSM << SPI_SSM);

    /* Data Frame Format Setting */
    temp |= (pSPIHandle->SPI_Config.DFF << SPI_DFF);

    /* CRC Transfer Next Setting */
    temp |= (pSPIHandle->SPI_Config.CRCNEXT << SPI_CRCNXT);

    /* CRC Hardware Enable Setting */
    temp |= (pSPIHandle->SPI_Config.CRCEN << SPI_CRCEN);


    /* Apply all settings at once to CR1 */
    pSPIHandle->pSPIx->CR1 |= temp;

    /* TODO */
    /* Repeat for CR2 and Status registers */

}

void SPI_DeInit (SPI_RegDef_t *pSPIx)
{
        if(pSPIx == SPI1){
    		SPI1_RESET ();
    	} else if (pSPIx == SPI2) {   
            SPI2_RESET ();
        } else if (pSPIx == SPI3) {
            SPI3_RESET ();
        } else if (pSPIx == SPI4) {
            SPI4_RESET ();
        }
}

/* I/O  */
/* Blocking I/O */
/*
    @brief: This method operates by taking in a pointer to the SPI 
    peripheral, a pointer to a buffer of singular bytes--or a byte array,
    and the length of the latter. It then sequentially waits until the
    transmit buffer is empty, places 1 or 2 bytes in, decreases the number
    of bytes required to be transferred, and increases the position of the
    pointer to the transmit buffer data.
*/
void SPI_Write  (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{

    if(length == 0){
        return;
    }

    while(length > 0)
    {
        /* Wait until TXE is set */
        while(!(pSPIx->SR & (1 << 1)));

        /* Drive Slave Select low */
        SPI_SSI_Config(pSPIx, RESET);

        /* Check DFF bit */
        if(pSPIx->CR1 & (1 << SPI_DFF))
        {
            /* 16 bit implementation */
            *((volatile uint16_t *)&pSPIx->DR) = *((uint16_t *) pTxBuffer); /* Cast to uint16_t then dereference to get the data */
            length--;
            length--;
            (uint16_t *) pTxBuffer++;

        } else {
        
            /* 8 bit implementation */
            *((volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
            length--;
            pTxBuffer++;

        }

        /* Drive SS high again. */
        SPI_SSI_Config(pSPIx, SET);

    }


}

void SPI_Read   (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{

}

/* Interrput Handling */
void SPI_IRQConfig         (uint8_t IRQNumber, uint8_t state)
{

}

void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling       (SPI_Handle_t *pSPIHandle)
{

}

/* Utility functions */
uint8_t SPI_TXE_STATUS  (SPI_RegDef_t *pSPIx, uint32_t flag)
{
    if((pSPIx->SR & 0x02U) & flag)
    {
        return SET;
    }

    return RESET;
}

void SPI_SSI_Config     (SPI_RegDef_t *pSPIx, uint32_t flag)
{
    /* Configure SSI Bit for SW slave management */

    if(flag == SET) {
        pSPIx->CR1 |= (1 << SPI_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_SSI);
    }
}
