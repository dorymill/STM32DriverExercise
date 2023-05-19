/*
 * HMC5883L.c
 *
 *  Created on: May 16, 2023
 *      Author: Asmod
 */

#include "HMC5883L.h"
#include <math.h>

#define PI 3.1415926

 
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
    uint8_t (*Txcmd) [3];

    pCompass->devAddr = COMPASS_ADDR;

    /* Configure CRA */
    temp = (compassCfg.averaging    << COMPASS_CRA_MA0)  |
           (compassCfg.output_rate  << COMPASS_CRA_DO0)  |
           (compassCfg.bias_control << COMPASS_CRA_MS0);

    commandGenTx(COMPASS_CFGA, temp, Txcmd);

    I2C_MasterTx(pI2CHandle, &Txcmd, 3, pCompass->devAddr);

    /* Configure CRB */
    temp = (compassCfg.gain_control << COMPASS_CRB_GN);

    commandGenTx(COMPASS_CFGB, temp, Txcmd);

    I2C_MasterTx(pI2CHandle, &Txcmd, 3, pCompass->devAddr);

    /* Configure Mode */
    temp = (compassCfg.op_mode << COMPASS_MODE_MDO) |
           (compassCfg.hsbit   << COMPASS_MODEHS);

	commandGenTx(COMPASS_MODE, temp, Txcmd);

    I2C_MasterTx(pI2CHandle, &Txcmd, 3, pCompass->devAddr);

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

	float temp = 0;

	temp = (180 / PI) * atan2(pCompass->compDat->y_msmnt, 
							  pCompass->compDat->x_msmnt);

	while (temp > 360) {
		temp -= 360;
	}

	while(temp < 0) {
		temp += 360;
	}

	pCompass->compDat->bearing = temp;

	return temp;
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

	uint8_t (*data)[6];

	uint8_t (*Rxcmd)[2];

	volatile uint8_t dataflag = 0;

	/* Check status register for data ready flag. */
	while(dataflag) { 
		commandGenRx(COMPASS_SR_RDY, Rxcmd);

		I2C_MasterTx(pI2CHandle,  Rxcmd,    2, COMPASS_ADDR);
		I2C_MasterRx(pI2CHandle, &dataflag, 1, COMPASS_ADDR);
		}

	/* Read in axis data values */
	for (int iter = 0; iter < 6; iter++) {
		commandGenRx(COMPASS_DR+iter, Rxcmd);
		
		I2C_MasterTx(pI2CHandle,  Rxcmd,         2, COMPASS_ADDR);
		I2C_MasterRx(pI2CHandle, *(data + iter), 1, COMPASS_ADDR);

	}
	  
	/* Store X data */
	pCompass->compDat->x_msmnt = (int16_t) ((*data[0] & 0xFF00) << 8) |
										   ((*data[1] & 0x00FF) << 0);

	/* Store Y data */
	pCompass->compDat->x_msmnt = (int16_t) ((*data[4] & 0xFF00) << 8) |
										   ((*data[5] & 0x00FF) << 0);

	/* Store Z data */
	pCompass->compDat->x_msmnt = (int16_t) ((*data[2] & 0xFF00) << 8) |
										   ((*data[2] & 0x00FF) << 0);
}

/**
 * @brief Fill a 2 bute array consisting of a read byte,
 *        and the compass register address to read from.
 * 
 * @param address   Compass register address
 * @return uint8_t* Pointer to crafted command array
 */
void commandGenRx (uint8_t address, uint8_t *Rxcmd) {

	*(Rxcmd + 0) = COMPASS_REG_READ;
	*(Rxcmd + 1) = address;

}


/**
 * @brief Fill a 3 byte array consisting of a write byte, an
 *        address byte, and a data byte. This can then be sent
 *        using I2C_MasterTx.
 * 
 * @param address   Compass register address
 * @param txDat     Data byte to write
 * @return uint8_t* Pointer to crafted command array
 */
void commandGenTx (uint8_t address, uint8_t txDat, uint8_t *Txcmd) {

	*(Txcmd + 0) = COMPASS_REG_WRITE;
	*(Txcmd + 1) = address;
	*(Txcmd + 2) = txDat;

}
