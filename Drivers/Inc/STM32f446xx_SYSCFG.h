/*
 * STM32f446xx_SYSCFG.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Asmod
 */

#include "STM32f446xx.h"

#ifndef STM32F446XX_SYSCFG_H_
#define STM32F446XX_SYSCFG_H_

#include <stdint.h>

#define __vo volatile

/**
 * @brief This file contains a macro pointing to the 
 *        System Configuration register and a struct
 *        for its bus.    
 * 
 */

typedef struct {

    __vo uint32_t MEMRMPR;
    __vo uint32_t PMCR;
    __vo uint32_t EXTICR[4];
    __vo uint32_t CMPCR;
    __vo uint32_t CFGR;

} SYSCFG_RegDef_t;

/* SYSCFG Explicit definition */
#define SYSCFG                   ((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#endif /* STM32F446XX_SYSCFG_H_ */
