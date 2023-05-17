/*
 * HMC5883L.c
 *
 *  Created on: May 16, 2023
 *      Author: Asmod
 */

#include "HMC5883L.h"

 
/*
  HMC5833L Compass API Implementation
*/

static void SingleShotRead(I2C_Handle_t *pI2CHandle, CompassHandle_t *pCompass);

/**
 * @brief Initialize the compass by sequentially sending 
 *        a write byte, the register, and the value to write.
 *        
 * @param pI2CHandle Handle for the I2C peripheral 
 * @param pCompass   Compass device handle
 * @param compassCfg Compass configuration structure 
 */
void InitCompass(I2C_Handle_t    *pI2CHandle,
                 CompassHandle_t *pCompass, 
                 CompassConfig_t  compassCfg)
{

    uint8_t temp = 0;
    uint8_t cmd[3] = {COMPASS_REG_WRITE,0,0};

    pCompass->devAddr = COMPASS_ADDR;

    /* Configure CRA */
    temp = (compassCfg.averaging    << COMPASS_CRA_MA0)  |
           (compassCfg.output_rate  << COMPASS_CRA_DO0)  |
           (compassCfg.bias_control << COMPASS_CRA_MS0);

    *(cmd + 1) = COMPASS_CFGA;
    *(cmd + 2) = temp;

    I2C_MasterTx(pI2CHandle, cmd, 3, pCompass->devAddr);

    /* Configure CRB */
    temp = (compassCfg.gain_control << COMPASS_CRB_GN);

    *(cmd + 1) = COMPASS_CFGB;
    *(cmd + 2) = temp;

    I2C_MasterTx(pI2CHandle, cmd, 3, pCompass->devAddr);

    /* Configure Mode */
    temp = (compassCfg.op_mode << COMPASS_MODE_MDO) |
           (compassCfg.hsbit   << COMPASS_MODEHS);

    *(cmd + 1) = COMPASS_MODE;
    *(cmd + 2) = temp;

    I2C_MasterTx(pI2CHandle, cmd, 3, pCompass->devAddr);

}

/**
 * @brief Calibrate the compass.
 * 
 * @param pCompass Compass device handle
 */
void Calibrate (CompassHandle_t *pCompass);

/**
 * @brief Get compass bearing.
 * 
 * @param pI2CHandle Handle for the I2C peripheral
 * @param compass    Compass device handle
 * @return float     Bearing in degrees
 */
float GetBearing (I2C_Handle_t    *pI2CHandle,
                  CompassHandle_t *pCompass)
{

       SingleShotRead(pI2CHandle, pCompass);

       return pCompass->compDat->bearing;
}


/**
 * @brief Fill the compass device data buffer with
 *        a single shot reading.
 * 
 * @param pI2CHandle Handle for the I2C peripheral
 * @param pCompass   Compass device handle
 */
void SingleShotRead(I2C_Handle_t *pI2CHandle, CompassHandle_t *pCompass)
{

}