/*
 * STM32f446xx_SPI.h
 *
 *  Created on: Jan 31, 2023
 *      Author: Asmod
 */

#include "STM32f446xx.h"

#ifndef STM32F446XX_SPI_H_
#define STM32F446XX_SPI_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/***************************************************************************/
/* SPI API Header */
/***************************************************************************/

/**
 * @brief This files contains the few basic structures for API implementation,
 *        macros for the base addresses of the SPI peripherals, macros to define
 *        the various configuration and mode settings, and lastly, the beloved 
 *        function prototypes.
 * 
 */


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

    __vo uint8_t    *phTxBuffer;
    __vo uint8_t    *phRxBuffer;
    __vo uint32_t    hTxLen;
    __vo uint32_t    hRxLen;
    __vo uint8_t     hTxState;
    __vo uint8_t     hRxState;

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
#define SPI_CRCERR               4
#define SPI_MODF                 5
#define SPI_OVR                  6
#define SPI_BUSY                 7
#define SPI_FRE                  8

#define SPI_ERRIE                5
#define SPI_RXNEIE               6
#define SPI_TXEIE                7

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

/* Events */
#define SPI_EVENT_TX_CMPLT       1
#define SPI_EVENT_RX_CMPLT       2
#define SPI_EVENT_OVR_CMPLT      3
#define SPI_EVENT_CRC_CMPLT      4
#define SPI_EVENT_MODF_CMPLT     5


/* Generic macros */
#define ENABLE                   1
#define DISABLE                  0

#define SET                      ENABLE

#define SPI_READY                0
#define SPI_BUSY_RX              1
#define SPI_BUSY_TX              2

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

/* Non-Blocking I/O */
uint8_t SPI_Int_Write    (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_Int_Read     (SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);

/* Interrput Handling */
void SPI_IRQConfig         (uint8_t IRQNumber, uint8_t state);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_ISR               (SPI_Handle_t *pSPIHandle);



/* Utility functions */
uint8_t SPI_TXE_STATUS  (SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_SSI_Config     (SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_SSOE_Config    (SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_SPE_Config     (SPI_RegDef_t *pSPIx, uint32_t flag);

void SPI_Tx_Abort       (SPI_Handle_t *pSPIHandle);
void SPI_Rx_Abort       (SPI_Handle_t *pSPIHandle);

/* Application callback interface */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);

#endif /* STM32F446XX_SPI_H_ */
