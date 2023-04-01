/*
 * STM32f446xx_GPIO.c
 *
 *  Created on: Jan 23, 2023
 *      Author: Asmod
 */

#include "STM32f446xx_GPIO.h"
#include "STM32f446xx_RCC.h"
#include "STM32f446xx_EXTI.h"
#include "STM32f446xx_SYSCFG.h"


/***************************************************************************/
/* GPIO API Function Implementations */
/***************************************************************************/

/**
 * @brief Initialization functions
 * 
 */

/**
 * @brief Sets the state of the clock for the GPIO register passed
 *        via @param pGPIOx based on @param state.
 * 
 * @param pGPIOx GPIO register pointer
 * @param state  Enable or disable
 */
void GPIO_ClockCtl (GPIO_RegDef_t *pGPIOx, uint8_t state)
{
    if(state == ENABLE){

    	if(pGPIOx == GPIOA){
    		GPIOA_CLOCK_EN ();
    	} else if (pGPIOx == GPIOB) {   
            GPIOB_CLOCK_EN ();
        } else if (pGPIOx == GPIOC) {
            GPIOC_CLOCK_EN ();
        } else if (pGPIOx == GPIOD) {
            GPIOD_CLOCK_EN ();
        } else if (pGPIOx == GPIOE) {
            GPIOE_CLOCK_EN ();
        } else if (pGPIOx == GPIOF) {
            GPIOF_CLOCK_EN ();
        } else if (pGPIOx == GPIOG) {
            GPIOG_CLOCK_EN ();
        }

    } else {
        if(pGPIOx == GPIOA){
    		GPIOA_CLOCK_DI ();
    	} else if (pGPIOx == GPIOB) {   
            GPIOB_CLOCK_DI ();
        } else if (pGPIOx == GPIOC) {
            GPIOC_CLOCK_DI ();
        } else if (pGPIOx == GPIOD) {
            GPIOD_CLOCK_DI ();
        } else if (pGPIOx == GPIOE) {
            GPIOE_CLOCK_DI ();
        } else if (pGPIOx == GPIOF) {
            GPIOF_CLOCK_DI ();
        } else if (pGPIOx == GPIOG) {
            GPIOG_CLOCK_DI ();
        }
    }

}

/**
 * @brief Initialzies the GPIO device by writing parameters
 *        found in the GPIO_PinConfig structure of the
 *        @param pGPIOHandle to the GPIO configuration
 *        registers.
 * 
 * @param pGPIOHandle Handle containing GPIO device and 
 *                    configuration parameters
 */
void GPIO_Init   (GPIO_Handle_t *pGPIOHandle)
{

    uint32_t temp = 0;
    uint8_t  pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    /* Set Mode */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

        /* Regular modes */
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  << (2 * pin));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3U << pin);
        pGPIOHandle->pGPIOx->MODER |= temp;
        
    } else {

        /* Interrupt modes */
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT) {
            /* Configure FTSR */
            EXTI->FTSR &= ~( 1 << pin);
            EXTI->RTSR |=  ( 1 << pin);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT) {
            /* Configure RTSR*/
            EXTI->RTSR &= ~( 1 << pin);
            EXTI->FTSR |=  ( 1 << pin);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FTRT) {
            /* Configure both */
            EXTI->RTSR |= ( 1 << pin);
            EXTI->FTSR |= ( 1 << pin);

        }

        /* Configure SYSCFG_EXTICR */
        uint8_t temp1 = pin / 8;
        uint8_t temp2 = pin % 8;

        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx)

        SYSCFG_CLOCK_EN ();

        SYSCFG->EXTICR[temp1] |= portcode  << (temp2 * 4);

        /* Enable Interrupt Deilvery using IMR */
        EXTI->IMR |= ( 1 << pin);

    }
    
    temp = 0;

    /* Set speed */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pin));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3U << pin);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;

    /* Set resistors */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDCtl << (2 *pin));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3U << pin);
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    /* Configure optype */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT){
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pin));
        pGPIOHandle->pGPIOx->OTYPER &= (~0x1U << pin);
        pGPIOHandle->pGPIOx->OTYPER |= temp;
        temp = 0;
    }

    /* Set alt func (opt) */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC) {

        uint8_t afno = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunc;

        if (pin > 7) {

            pGPIOHandle->pGPIOx->AFRH &= ~(0xFU << ((pin & 7) * 4));
            pGPIOHandle->pGPIOx->AFRH |= (afno << ((pin & 7) * 4));

        } else {

            pGPIOHandle->pGPIOx->AFRL &= (~0xFU << ((pin & 7) * 4));
            pGPIOHandle->pGPIOx->AFRL |= (afno << (((pin & 7) * 4)));

        }

    }

}

/**
 * @brief Resets the GPIO peripheral back to default state.
 * 
 * @param pGPIOx GPIO register pointer
 */
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx)
{
        if(pGPIOx == GPIOA){
    		GPIOA_RESET ();
    	} else if (pGPIOx == GPIOB) {   
            GPIOB_RESET ();
        } else if (pGPIOx == GPIOC) {
            GPIOC_RESET ();
        } else if (pGPIOx == GPIOD) {
            GPIOD_RESET ();
        } else if (pGPIOx == GPIOE) {
            GPIOE_RESET ();
        } else if (pGPIOx == GPIOF) {
            GPIOF_RESET ();
        } else if (pGPIOx == GPIOG) {
            GPIOG_RESET ();
        }
}

/**
 * @brief I/O Functions 
 */

/**
 * @brief Reads and returns the state of the passed GPIO
 *        @param pin on the passed @param pGPIOx bus.
 * 
 * @param pGPIOx    GPIO register pointer
 * @param pin       GPIO pin number
 * @return uint8_t  State of GPIO pin
 */
uint8_t GPIO_ReadPin   (GPIO_RegDef_t *pGPIOx, uint8_t pin)
{
    return (uint8_t) ((pGPIOx->IDR >> pin) & 0x00000001);
}

/**
 * @brief Reads and returns the entire internal data register
 *        for the passed @param pGPIOx bus.
 * 
 * @param pGPIOx     GPIO register pointer
 * @return uint16_t  Value of passed GPIO IDR
 */
uint16_t GPIO_ReadPort (GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t) (pGPIOx->IDR);
}

/**
 * @brief Write @param val to @param pin on the passed 
 *        @param pGPIOx register.
 * 
 * @param pGPIOx GPIO register pointer
 * @param pin    GPIO pin to write to
 * @param val    Value to write (Enable or Disable)
 */
void GPIO_WritePin     (GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t val)
{
    if (val == ENABLE) {
        pGPIOx->ODR |= (1 << pin);
    } else {
        pGPIOx->ODR &= ~(1 << pin);
    }
}

/**
 * @brief Allows one to write to all of the pins on the passed
 *        @param pGPIOx output data register at once based upon 
 *        @param val.
 * 
 * @param pGPIOx GPIO register pointer
 * @param val    Value to write to ODR
 */
void GPIO_WritePort    (GPIO_RegDef_t *pGPIOx, uint16_t val)
{
    pGPIOx->ODR = val;
}

/**
 * @brief Toggles the state of the passed @param pin on
 *        the passed @param pGPIOx bus.
 * 
 * @param pGPIOx GPIO register pointer
 * @param pin    GPIO pin
 */
void GPIO_TogglePin    (GPIO_RegDef_t *pGPIOx, uint8_t pin)
{
    pGPIOx->ODR ^= (1 << pin);
}

/**
 * @brief Interrupt Functions 
 */

/**
 * @brief Configures the appropriate NVIC set or clear registers
 *        based on the passed @param IRQNumber and @param state.
 * 
 * @param IRQNumber Interrupt request number
 * @param state     Enable or disable
 */
void GPIO_IRQConfig    (uint8_t IRQNumber, uint8_t state)
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
 * @brief Sets the @param IRQPriority for the passed
 *        @param IRQNumber.
 * 
 * @param IRQNumber    Interrupt request number
 * @param IRQPriority  Interrupt priority
 */
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
    /* Figure out register */
    uint8_t iprx         = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amnt   = (8 * iprx_section) + (8 - NVIC_IPR_BITS_IMP);

    /* Write to it */
    *(NVIC_IPR + iprx) |= (IRQPriority << (shift_amnt));

}

/**
 * @brief Function that handles what happens when an
 *        interrupt is thrown across a given @param pin.
 *        
 * 
 * @param pin Interrupt pin
 */
void GPIO_IRQHandling  (uint8_t pin)
{

    /* Clear EXTI PR Register corresponding to pin */
    if(EXTI->PR & (1 << pin)) {
        EXTI->PR |= (1 << pin);
    }

    // Then perhaps set a flag to indicate the interrupt occurred for the
    // next cycle across the main loop.

}

/**
 * @brief Tight CPU loop for timing adjustments.
 * 
 * @param val Passing 1 to this parameter for this board 
 *            results in about 300µs delay from the last clock
 *            edge of a SPI transaction to the NSS high edge.
 *             
 */
void delay(uint32_t val)
{
unsigned int j;
    while(val--)
        for(j=0; j < 50; j++);
}
