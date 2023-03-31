/*
 * STM32f446xx_SPI.h
 *
 *  Created on: Jan 31, 2023
 *      Author: hackr6
 */

#include "STM32f446xx.h"

#ifndef STM32F446XX_SPI_H_
#define STM32F446XX_SPI_H_

#include <stdint.h>

#define __vo volatile

/***************************************************************************/
/* SPI API Header */
/***************************************************************************/


/* SPI Register definition structure */
typedef struct {

    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t CRCPR;
    __vo uint32_t RXCRCR;
    __vo uint32_t TXCRCR;
    __vo uint32_t I2SCFGR;
    __vo uint32_t I2SPR;

} SPI_RegDef_t;

/* SPI Configuration structure */
typedef struct {

    uint8_t BIDIMODE;
    uint8_t BIDIOE;
    uint8_t CRCEN;
    uint8_t CRCNEXT;
    uint8_t DFF;
    uint8_t RXONLY;
    uint8_t SSM;
    uint8_t SSI;
    uint8_t LSBFIRST;
    uint8_t SPE;
    uint8_t BR;
    uint8_t MSTR;
    uint8_t CPOL;
    uint8_t CPHA;

} SPI_Config_t;

/* SPI Handle structure */
typedef struct {

    SPI_RegDef_t  *pSPIx;
    SPI_Config_t   SPI_Config;

} SPI_Handle_t;

/* Explicit SPI peripheral definitions */
#define SPI1                     ((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2                     ((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3                     ((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4                     ((SPI_RegDef_t *) SPI4_BASEADDR)

/* CR1 Shorthand definitions */
#define SPI_CPHA                 0
#define SPI_CPOL                 1
#define SPI_SSOE                 2
#define SPI_MSTR                 2
#define SPI_BR                   3
#define SPI_SPE                  6
#define SPI_LSBFIRST             7
#define SPI_SSI                  8
#define SPI_SSM                  9
#define SPI_RXONLY               10
#define SPI_DFF                  11
#define SPI_CRCNXT               12
#define SPI_CRCEN                13
#define SPI_BIDIOE               14
#define SPI_BIDIMODE             15

#define SPI_RXNE                 0
#define SPI_TXE                  1
#define SPI_BUSY                 7

#define SPI_TXE_FLAG             (1 << SPI_TXE)
#define SPI_RXNE_FLAG            (1 << SPI_RXNE)
#define SPI_BUSY_FLAG            (1 << SPI_BUSY)

/* SPI Mode Macros */
#define SPI_BIDIMODE_FD          0
#define SPI_BIDIMODE_HD          1

#define SPI_BIDIOE_RXONLY        0
#define SPI_BIDIOE_TXONLY        1

#define SPI_CRCEN_DIS            0
#define SPI_CRCEN_EN             1

#define SPI_CRCNEXT_DP           0
#define SPI_CRCNEXT_NT           1

#define SPI_DFF_8BIT             0
#define SPI_DFF_16BIT            1

#define SPI_RXONLY_FD            0
#define SPI_RXONLY_OD            1

#define SPI_SSM_SW               1
#define SPI_SSM_HW               0

#define SPI_LSBFIRST_DIS         0
#define SPI_LSBFIRST_EN          1

#define SPI_SPE_DIS              0
#define SPI_SPE_EN               1

#define SPI_BAUDSCALAR_2         0
#define SPI_BAUDSCALAR_4         1
#define SPI_BAUDSCALAR_8         2
#define SPI_BAUDSCALAR_16        3
#define SPI_BAUDSCALAR_32        4
#define SPI_BAUDSCALAR_64        5
#define SPI_BAUDSCALAR_128       6
#define SPI_BAUDSCALAR_256       7

#define SPI_MSTR_SLAVE           0
#define SPI_MSTR_MASTER          1

#define SPI_CPOL_LO              0
#define SPI_CPOL_HI              1

#define SPI_CPHA_RISE            0
#define SPI_CPHA_FALL            1


/* Generic macros */
#define ENABLE                   1
#define DISABLE                  0

#define SET                      ENABLE

/***************************************************************************/
/* SPI API Function Prototypes */
/***************************************************************************/

/* Peripheral clock setup */
void SPI_ClockCtl (SPI_RegDef_t *pSPIx, uint8_t state);

/* Init/De-init */
void SPI_Init   (SPI_Handle_t *pSPIHandle);
void SPI_DeInit (SPI_RegDef_t *pSPIx);

/* I/O  */
/* Blocking I/O */
void SPI_Write  (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_Read   (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

/* Interrput Handling */
void SPI_IRQConfig         (uint8_t IRQNumber, uint8_t state);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling       (SPI_Handle_t *pSPIHandle);

/* Utility functions */
uint8_t SPI_TXE_STATUS  (SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_SSI_Config     (SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_SSOE_Config    (SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_SPE_Config     (SPI_RegDef_t *pSPIx, uint32_t flag);

#endif /* STM32F446XX_SPI_H_ */
