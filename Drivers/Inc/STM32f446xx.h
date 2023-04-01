/*
 * STM32f446xx.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Asmod
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_

#include <stdint.h>

#define __vo volatile

/**
 * @brief This files contains macros for pointers to the addresses of 
 *        peripherals of the MCU unit, and NVIC register pointer macros.    
 * 
 */

/* Processor Specific: ARM Cortex-M4 registers */
/* Interrupt Set Registers */
#define NVIC_ISER0               ((__vo uint32_t *) 0xE000E100U)
#define NVIC_ISER1               ((__vo uint32_t *) 0xE000E104U)
#define NVIC_ISER2               ((__vo uint32_t *) 0xE000E108U)
#define NVIC_ISER3               ((__vo uint32_t *) 0xE000E10CU)
#define NVIC_ISER4               ((__vo uint32_t *) 0xE000E110U)
#define NVIC_ISER5               ((__vo uint32_t *) 0xE000E114U)
#define NVIC_ISER6               ((__vo uint32_t *) 0xE000E118U)
#define NVIC_ISER7               ((__vo uint32_t *) 0xE000E11CU)

/* Interrupt Clear Registers */
#define NVIC_ICER0               ((__vo uint32_t *) 0xE000E180U)
#define NVIC_ICER1               ((__vo uint32_t *) 0xE000E184U)
#define NVIC_ICER2               ((__vo uint32_t *) 0xE000E188U)
#define NVIC_ICER3               ((__vo uint32_t *) 0xE000E18CU)
#define NVIC_ICER4               ((__vo uint32_t *) 0xE000E190U)
#define NVIC_ICER5               ((__vo uint32_t *) 0xE000E194U)
#define NVIC_ICER6               ((__vo uint32_t *) 0xE000E198U)
#define NVIC_ICER7               ((__vo uint32_t *) 0xE000E19CU)

/* Interrupt Priority Base Register */
#define NVIC_IPR                ((__vo uint32_t *) 0xE000E400U)

/* Lower Order bits not implemented in this MCU */
#define NVIC_IPR_BITS_IMP        4

/* Base Addresses of Flash and SRAM memory */

#define FLASH_BASEADDR           0x08000000U
#define SRAM1_BASEADDR           0x20000000U
#define SRAM2_BASEADDR           0x2001C000U

/* AHBx and APBx Peripheral base addresses */

#define PERIPHERAL_BASEADDR      0x40000000u
#define APB1_BASEADDR            PERIPHERAL_BASEADDR
#define APB2_BASEADDR            0x40010000U
#define AHB1_BASEADDR            0x40020000U
#define AHB2_BASEADDR            0x50000000U
#define AHB3_BASEADDR            0x60000000U

/* Base addresses of peripherals hanging onto AHB1 bus */

/* GPIOx Registers */
#define GPIOA_BASEADDR           (AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR           (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR           (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR           (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR           (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR           (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR           (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR           (AHB1_BASEADDR + 0x1C00)

/* CRC Registers */
#define CRC_BASEADDR             (AHB1_BASEADDR + 0x3000)

/* RCC Registers */
#define RCC_BASEADDR             (AHB1_BASEADDR + 0x3800)

/* Flash Interface Register */
#define FLASHINT_BASEADDR        (AHB1_BASEADDR + 0x3C00)

/* BKPSRAM Register */
#define BKSPKRAM_BASEADDR        (AHB1_BASEADDR + 0x4000)

/* DMA Register*/
#define DMA1_BASEADDR            (AHB1_BASEADDR + 0x6000)
#define DMA2_BASEADDR            (AHB1_BASEADDR + 0x6400)

/* USBOTGHS Register */
#define USBOTGHS_BASEADDR        (AHB1_BASEADDR + 0x00040000)



/* Base addresses of peripherals hanging onto AHB2 bus */

/* USBOTGFS Register */
#define USBOTGFS_BASEADDR        (AHB2_BASEADDR + 0x0000)

/* DCMI Register */
#define DCMI_BASEADDR            (AHB2_BASEADDR + 0x00050000)


/* Base addresses of peripherals hanging onto AHB3 bus */

/* FMC Bank 1 Register */
#define FMCBANK1_BASEADDR        (AHB3_BASEADDR + 0x0000)

/* FMC Bank 3 Register */
#define FMCBANK3_BASEADDR        (AHB3_BASEADDR + 0x20000000)

/* QuadSPI Register */
#define QUADSPI_BASEADDR         (AHB3_BASEADDR + 0x30000000)

/* FMC Control Register */
#define FMCCR_BASEADDR           (AHB3_BASEADDR + 0x40000000)

/* QuadSPI Control Register */
#define QUADSPICR_BASEADDR       (AHB3_BASEADDR + 0x40001000)

/* Define FMC Bank 5 Register */
#define FMCBANK5_BASEADDR        (AHB3_BASEADDR + 0x60000000)

/* Define FMC Bank 6 Register */
#define FMCBANK6_BASEADDR        (AHB3_BASEADDR + 0x70000000)


/* Base addreses of peripherals hanging onto APB1 bus */

/* TIM Registers */
#define TIM2_BASEADDR            (APB1_BASEADDR + 0x0000)
#define TIM3_BASEADDR            (APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR            (APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR            (APB1_BASEADDR + 0x0C00)
#define TIM6_BASEADDR            (APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR            (APB1_BASEADDR + 0x1400)
#define TIM12_BASEADDR           (APB1_BASEADDR + 0x1800)
#define TIM13_BASEADDR           (APB1_BASEADDR + 0x1C00)

/* RTC & Bkp Registers */
#define RTCBKP_BASEADDR          (APB1_BASEADDR + 0x2800)

/* Window Watchdog Register */
#define WWDG_BASEADDR            (APB1_BASEADDR + 0x2C00)

/* Independent Watchdog Register */
#define IDWDG_BASEADDR           (APB1_BASEADDR + 0x3000)

/* SPI / I2S Registers */
#define SPI2_BASEADDR            (APB1_BASEADDR + 0x3800)
#define I2S2_BASEADDR            (APB1_BASEADDR + 0x3800)

#define SPI3_BASEADDR            (APB1_BASEADDR + 0x3C00)
#define I2S3_BASEADDR            (APB1_BASEADDR + 0x3C00)

/* SPDIFRX Register */
#define SPDIFRX_BASEADDR         (APB1_BASEADDR + 0x4000)

/* U(S)ART Registers */
#define USART2_BASEADDR          (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR          (APB1_BASEADDR + 0x4800)

#define UART4_BASEADDR           (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR           (APB1_BASEADDR + 0x5000)

/* I2C Registers */
#define I2C1_BASEADDR            (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR            (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR            (APB1_BASEADDR + 0x5C00)

/* FMPI2C1 Register */
#define FMPI2C1_BASEADDR         (APB1_BASEADDR + 0x6000)

/* CAN Registers */
#define CAN1_BASEADDR            (APB1_BASEADDR + 0x6400)
#define CAN2_BASEADDR            (APB1_BASEADDR + 0x6800)

/* HDMI-CEC Register */
#define HDMICEC_BASEADDR         (APB1_BASEADDR + 0x6C00)

/* PWR Register */
#define PWR_BASEADDR             (APB1_BASEADDR + 0x7000)

/* DAC Register */
#define DAC_BASEADDR             (APB1_BASEADDR + 0x7400)


/* Base addreses of peripherals hanging onto APB2 bus */

/* TIM Registers */
#define TIM1_BASEADDR            (APB2_BASEADDR + 0x0000)
#define TIM8_BASEADDR            (APB2_BASEADDR + 0x0400)
#define TIM9_BASEADDR            (APB2_BASEADDR + 0x4000)
#define TIM10_BASEADDR           (APB2_BASEADDR + 0x4400)
#define TIM11_BASEADDR           (APB2_BASEADDR + 0x4800)

/* USART Registers */
#define USART1_BASEADDR          (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR          (APB2_BASEADDR + 0x1400)

/* ADC Registers */
#define ADC_BASEADDR             (APB2_BASEADDR + 0x2000)

/* SDIO Registers */
#define SDIO_BASEADDR            (APB2_BASEADDR + 0x2C00)

/* SPI Registers */
#define SPI1_BASEADDR            (APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR            (APB2_BASEADDR + 0x3400)

/* SYSCFG Registers */
#define SYSCFG_BASEADDR          (APB2_BASEADDR + 0x3800)

/* EXTI Register */
#define EXTI_BASEADDR            (APB2_BASEADDR + 0x3C00)

/* SAI Registers */
#define SAI1_BASEADDR            (APB2_BASEADDR + 0x5800)
#define SAI2_BASEADDR            (APB2_BASEADDR + 0x5C00)


#endif /* STM32F446XX_H_ */
