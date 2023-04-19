/*
 * STM32f446xx_IRQ.h
 *
 *  Created on: Jan 25, 2023
 *      Author: Asmod
 */

#ifndef STM32F446XX_IRQ_H_
#define STM32F446XX_IRQ_H_

/**
 * @brief This file contains macros for interrupt request
 *        numbers/positions.    
 * 
 */

/* GPIO EXTI IRQ Numbers*/
#define IRQ_NO_EXTI0             6
#define IRQ_NO_EXTI1             7
#define IRQ_NO_EXTI2             8
#define IRQ_NO_EXTI3             9
#define IRQ_NO_EXTI4             10
#define IRQ_NO_EXTI9_5           23
#define IRQ_NO_EXTI15_10         40

/* SPI IRQ Numbers */
#define IRQ_NO_SPI1              35
#define IRQ_NO_SPI2              36
#define IRQ_NO_SPI3              51
#define IRQ_NO_SPI4              84

#endif /* STM32F446XX_IRQ_H_ */
