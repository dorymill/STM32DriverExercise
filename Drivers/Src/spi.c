/*
 * STM32f446xx_SPI.c
 *
 *  Created on: Feb 19, 2023
 *      Author: Asmod
 */

#include <exti.h>
#include <rcc.h>
#include <spi.h>
#include <syscfg.h>


/***************************************************************************/
/* SPI API Function Implementations */
/***************************************************************************/

/**
 * @brief Initialization functions
 * 
 */

/**
 * @brief Enable and disables the clock for the provided SPI 
 *        peripheral.
 * 
 * @param pSPIx SPI register pointer
 * @param state Clock state
 */
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

/**
 * @brief Initialize the SPI peripheral associated with
 *        the passed handle and configuration structure
 *        therein.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
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

/**
 * @brief Deinitialize the SPI device.
 * 
 * @param pSPIx SPI register pointer
 */
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


/**
 * @brief I/O Functions 
 */

/**
 * @brief This method operates by taking in a pointer to the SPI 
 *        peripheral, a pointer to a buffer of singular bytes--or a byte array,
 *        and the length of the latter. It then sequentially waits until the
 *        transmit buffer is empty, places 1 or 2 bytes in, decreases the number
 *        of bytes required to be transferred, and increases the position of the
 *        pointer to the transmit buffer data.
 * 
 * @param pSPIx     SPI register pointer
 * @param pTxBuffer Pointer to user Tx data buffer
 * @param length    Amount of data to send
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

    }

}

/**
 * @brief Performs the same fundamental operations as the write function
 *        however, reads from the DR instead of writing to it.
 * 
 * @param pSPIx      SPI register pointer
 * @param pRxBuffer  Pointer to user Rx data buffer
 * @param length     Amount of data to receive
 */
void SPI_Read   (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
    if(length == 0){
        return;
    }

    while(length > 0)
    {
        /* Wait until RXNE is not set */
        while(!(pSPIx->SR & (1 << 0)));

        /* Check DFF bit */
        if(pSPIx->CR1 & (1 << SPI_DFF))
        {
            /* 16 bit implementation */
            *((volatile uint16_t *)&pRxBuffer) = *((uint16_t *) pSPIx->DR); /* Cast to uint16_t then dereference to get the data */
            length--;
            length--;
            (uint16_t *) pRxBuffer++;

        } else {
        
            /* 8 bit implementation */
            *((volatile uint8_t *)&pRxBuffer) = pSPIx->DR;
            length--;
            pRxBuffer++;

        }

    }
}

/**
 * @brief This method triggers a SPI write transaction via interrupt.
 *        We check the state of the internal SPI handle, set the internal buffer 
 *        pointer to the data to transmit along with the length parameter,
 *        then trigger the interrupt via setting the TXEIE bit in the SPI
 *        device config register referenced in pSPIHandle.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 * @param pTxBuffer  Pointer to user Tx data buffer
 * @param length     Amount of data to send
 * @return uint8_t   Current state of internal SPI handle @param hTxState
 */
uint8_t SPI_Int_Write    (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length)
{

    uint8_t state = pSPIHandle->hTxState;

    if(state != SPI_BUSY_RX) {

        /* Save TxBuffer and length to handle variables. */
        pSPIHandle->phTxBuffer = pTxBuffer;
        pSPIHandle->hTxLen     = length;

        /* Mark SPI state busy -> semaphore */
        pSPIHandle->hTxState   = SPI_BUSY_TX;

        /* Enable TXEIE control bit */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_TXEIE);

    }

    /* Data Tx handled by ISR */

    return state;
}

/**
 * @brief Does the same as the interrupt write routine, utilizing the RXNEIE 
 *        interrupt bit instead.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 * @param pRxBuffer  Pointer to user Rx data buffer
 * @param length     Amount of data to receive
 * @return uint8_t   Current state of internal SPI handle @param hRxState
 */
uint8_t SPI_Int_Read     (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length)
{

    uint8_t state = pSPIHandle->hRxState;

    if(state != SPI_BUSY_TX) {

        /* Save RxBuffer and length to handle variables. */
        pSPIHandle->phRxBuffer = pRxBuffer;
        pSPIHandle->hRxLen     = length;

        /* Mark SPI state busy -> semaphore */
        pSPIHandle->hRxState   = SPI_BUSY_RX;

        /* Enable RXNEIE control bit */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_RXNEIE);

    }

    /* Data Rx handled by ISR */

    return state;

}


/**
 * @brief Interrupt handling functionality.
 */

/* Static function prototypes */
static void SPI_TXE_ISR    (SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_ISR   (SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ISR    (SPI_Handle_t *pSPIHandle);

/**
 * @brief Sets the appropriate NVIC state from IRQNumber.
 * 
 * @param IRQNumber NVIC IRQ position
 * @param state     IRQ state
 */
void SPI_IRQConfig    (uint8_t IRQNumber, uint8_t state)
{
    if(state == ENABLE) {

        if(IRQNumber <= 31) {

            (*NVIC_ISER0) |= (1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber < 64) {
            
            (*NVIC_ISER1) |= (1 << (IRQNumber % 32));

        } else if (IRQNumber >= 64 && IRQNumber < 96) {

            (*NVIC_ISER2) |= (1 << (IRQNumber % 64));
        }

    } else {

        if(IRQNumber <= 31) {

            (*NVIC_ICER0) &= ~(1 << IRQNumber);

        } else if (IRQNumber > 31 && IRQNumber < 64) {

            (*NVIC_ICER1) &= ~(1 << (IRQNumber % 32));

        } else if (IRQNumber >= 64 && IRQNumber < 96) {

            (*NVIC_ICER2) &= ~(1 << (IRQNumber % 64));

        }
    }
}

/**
 * @brief Sets the priority of the given IRQNumber.
 * 
 * @param IRQNumber   NVIC IRQ position
 * @param IRQPriority Interrupt priority
 */
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* Figure out register */
    uint8_t iprx         = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amnt   = (8 * iprx_section) + (8 - NVIC_IPR_BITS_IMP);

    /* Write to it */
    *(NVIC_IPR + iprx) |= (IRQPriority << (shift_amnt));

}

/**
 * @brief Interrupt service routine for SPI peripherals.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
void SPI_ISR       (SPI_Handle_t *pSPIHandle)
{
    /* Since there are multiple types of interrupt, we must handle
    them case by case */
    uint8_t temp1, temp2;

    /* Checking and handling of TXE */
    temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_TXEIE);

    if (temp1 && temp2) {
        SPI_TXE_ISR(pSPIHandle);
    }

    /* Checking and handling of RXNE */
    temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_RXNEIE);

    if (temp1 && temp2) {
        SPI_RXNE_ISR(pSPIHandle);
    }

    /* CRC not implemented in this course, but straightforward to do. Same with MODF. 
       we mainly care about read and write. Implement as an exercise.
    */
    // /* Checking and handling of CRC Error */
    // temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_CRCERR);
    // temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_ERRIE);

    // if (temp1 && temp2) {
    //     SPI_CRCERR_ISR();
    // }

    /* Checking and handling of OVR */
    temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_ERRIE);

    if (temp1 && temp2) {
        SPI_OVR_ISR(pSPIHandle);
    }

}

/**
 * @brief Takes in the SPI Handle and executes the SPI_Read
 *        operation using the data buffers and length fields
 *        internal to the pSPIHandle structure, at the behest
 *        of the processor interrupt and provides and application
 *        callback method.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
static void SPI_TXE_ISR    (SPI_Handle_t *pSPIHandle)
{

        /* Check DFF bit */
        if(pSPIHandle->pSPIx->CR1 & (1 << SPI_DFF))
        {
            /* 16 bit implementation */
            /* Cast to uint16_t then dereference to get the data */
            *((volatile uint16_t *)&pSPIHandle->pSPIx->DR) = *((uint16_t *) pSPIHandle->phTxBuffer); 
            
            pSPIHandle->hTxLen--;
            pSPIHandle->hTxLen--;
            
            pSPIHandle->phTxBuffer++;
            pSPIHandle->phTxBuffer++;

        } else {
        
            /* 8 bit implementation */
            *((volatile uint8_t *)&pSPIHandle->pSPIx->DR) = *pSPIHandle->phTxBuffer;
            
            pSPIHandle->hTxLen--;
            
            pSPIHandle->phTxBuffer++;

        }

        if(!pSPIHandle->hTxLen) {

            SPI_Tx_Abort(pSPIHandle);
            
            SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
        }

}

/**
 * @brief Takes in the SPI Handle and executes the SPI_Write
 *        operation on interrupt.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
static void SPI_RXNE_ISR   (SPI_Handle_t *pSPIHandle)
{
        /* Check DFF bit */
        if(pSPIHandle->pSPIx->CR1 & (1 << SPI_DFF))
        {
            /* 16 bit implementation */
            *((volatile uint16_t *)&pSPIHandle->phRxBuffer) = *((uint16_t *) pSPIHandle->pSPIx->DR); /* Cast to uint16_t then dereference to get the data */
            
            pSPIHandle->hRxLen--;
            pSPIHandle->hRxLen--;
            
            pSPIHandle->phRxBuffer++;
            pSPIHandle->phRxBuffer++;

        } else {
        
            /* 8 bit implementation */
            *((volatile uint8_t *)&pSPIHandle->phRxBuffer) = pSPIHandle->pSPIx->DR;
            
            pSPIHandle->hRxLen--;
            
            pSPIHandle->phRxBuffer++;

        }

        if(!pSPIHandle->hRxLen) {

            SPI_Rx_Abort(pSPIHandle);
            
            SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
        }
}

/**
 * @brief Handles overrun fault by read to the SPI data and 
 *        status registers respectively.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
static void SPI_OVR_ISR    (SPI_Handle_t *pSPIHandle)
{

    uint8_t temp;

    /* Read from the DR followed by the SR. */
    if(pSPIHandle->hTxState != SPI_BUSY_TX) {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }

    (void) temp;
    
    /* Notify the application. */
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_CMPLT);
}

/**
 * @brief Utility functions to handle minor bit flips.
 */

/**
 * @brief Acquires the pSPIx TXE bit state and compares it to the
 *        flag, returning 1 or 0 if they match or do not.
 * 
 * @param pSPIx    SPI register pointer
 * @param flag     Flag to compare to
 * @return uint8_t 1 for match, 0 if not
 */
uint8_t SPI_TXE_STATUS  (SPI_RegDef_t *pSPIx, uint32_t flag)
{
    if((pSPIx->SR & 0x02U) & flag)
    {
        return SET;
    }

    return RESET;
}

/**
 * @brief Configures the pSPIx SSI bit.
 * 
 * @param pSPIx SPI register pointer
 * @param flag  SSI state
 */
void SPI_SSI_Config     (SPI_RegDef_t *pSPIx, uint32_t flag)
{

    if(flag == SET) {
        pSPIx->CR1 |= (1 << SPI_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_SSI);
    }
}

/**
 * @brief Configures the pSPIx SSOE bit.
 * 
 * @param pSPIx SPI register pointer
 * @param flag  SSOE state
 */
void SPI_SSOE_Config    (SPI_RegDef_t *pSPIx, uint32_t flag)
{

    if(flag == SET) {
        pSPIx->CR2 |= (1 << SPI_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_SSOE);
    }
}

/**
 * @brief Configures the pSPIx SPE bit, used for NSS.
 * 
 * @param pSPIx SPI register pointer
 * @param flag  SPE state (0 for NSS high and 1 for NSS low in SSM HW mode)
 */
void SPI_SPE_Config     (SPI_RegDef_t *pSPIx, uint32_t flag)
{

    if(flag == SET) {
        pSPIx->CR1 |= (1 << SPI_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_SPE);
    }
}

/**
 * @brief Disengage SPI TXE interrupt and sets internal handle
 *        Tx buffers and states to ready.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
void SPI_Tx_Abort       (SPI_Handle_t *pSPIHandle)
{
    /* Flip TXEIE control bit to disengage the interrupt. */
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_TXEIE);
    
    pSPIHandle->phTxBuffer = NULL;
    pSPIHandle->hTxLen     = 0;
    pSPIHandle->hTxState   = SPI_READY;

}

/**
 * @brief Disengage SPI RXNE interrupt and sets internal
 *        handle Rx buffers and states to ready.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 */
void SPI_Rx_Abort       (SPI_Handle_t *pSPIHandle)
{
    /* Flip RXNEIE control bit to disengage interrupt. */
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_RXNEIE);
    
    pSPIHandle->phRxBuffer = NULL;
    pSPIHandle->hRxLen     = 0;
    pSPIHandle->hRxState   = SPI_READY;

}

/**
 * @brief Application callback to be overridden by application.
 * 
 * @param pSPIHandle Handle for the SPI peripheral
 * @param event      Event type, BUSY, TX, RX, READY
 * @return __weak 
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
    // To be implemented by the application.
}
