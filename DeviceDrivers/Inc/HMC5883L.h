/*
 * HMC5883L.h
 *
 *  Created on: May 16, 2023
 *      Author: Asmod
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <Inc/i2c.h>>

typedef struct {

    int16_t x_msmnt;
    int16_t y_msmnt;
    int16_t z_msmnt;

    int16_t bearing;

} CompassData_t;

typedef struct {

    uint8_t          devAddr;
    CompassData_t   *compDat;

} CompassHandle_t;

typedef struct {

    uint8_t averaging;
    uint8_t output_rate;
    uint8_t bias_control;

    uint8_t gain_control;

    uint8_t op_mode;
    uint8_t hsbit;

} CompassConfig_t;

/* Device address */
#define COMPASS_ADDR                           0x1E

/* Read/write command bytes */
#define COMPASS_REG_READ                       0x3D
#define COMPASS_REG_WRITE                      0x3C

/* Register base addresses */
#define COMPASS_CFGA                           0
#define COMPASS_CFGB                           1
#define COMPASS_MODE                           2
#define COMPASS_XMSB                           3
#define COMPASS_XLSB                           4
#define COMPASS_ZMSB                           5
#define COMPASS_ZLSB                           6
#define COMPASS_YMSB                           7
#define COMPASS_YLSB                           8
#define COMPASS_SR                             9
#define COMPASS_IDA                           10
#define COMPASS_IDB                           11
#define COMPASS_IDC                           12

/* Configuration Register A definitions */
#define COMPASS_CRA_MS0                        0
#define COMPASS_CRA_MS1                        1
#define COMPASS_CRA_DO0                        2
#define COMPASS_CRA_DO1                        3
#define COMPASS_CRA_DO2                        4
#define COMPASS_CRA_MA0                        5
#define COMPASS_CRA_MA1                        6
#define COMPASS_CRA_RESERVED                   7

#define COMPASS_MS_NORMAL                      0
#define COMPASS_MS_POS                         1
#define COMPASS_MS_NEG                         2

#define COMPASS_DSO_P75Hz                      0
#define COMPASS_DSO_1P5Hz                      1
#define COMPASS_DSO_3Hz                        2
#define COMPASS_DSO_P7P5Hz                     3
#define COMPASS_DSO_15Hz                       4
#define COMPASS_DSO_30Hz                       5
#define COMPASS_DSO_75Hz                       6

#define COMPASS_MA_AVG1                        0
#define COMPASS_MA_AVG2                        1
#define COMPASS_MA_AVG4                        2
#define COMPASS_MA_AVG8                        3

/* Configuration Register B definitions */
#define COMPASS_CRB_GN                         5

#define COMPASS_GAIN_P88G                      0
#define COMPASS_GAIN_P1P3G                     1
#define COMPASS_GAIN_P1P9G                     2
#define COMPASS_GAIN_P2P5G                     3
#define COMPASS_GAIN_P4P0G                     4
#define COMPASS_GAIN_P4P7G                     5
#define COMPASS_GAIN_P5P6G                     6
#define COMPASS_GAIN_P8P1G                     7

#define UNSIGNED_OFFSET                        0x07FF

/* Mode Register definitions */
#define COMPASS_MODE_MDO                       0
#define COMPASS_MODEHS                         7

#define COMPASS_MODEHS_EN                      1

#define COMPASS_MODE_CONT                      0
#define COMPASS_MODE_SNGL                      1
#define COMPASS_MODE_IDLE                      2, 3

/* Status Register definitions */
#define COMPASS_SR_RDY                         0
#define COMPASS_SR_LOCK                        1

void InitCompass(I2C_Handle_t   *pI2CHandle,
                CompassHandle_t *pCompass, 
                CompassConfig_t  compassCfg);

#endif /* HMC5883L_H_ */
