/*
 * HMC5883L.c
 *
 *  Created on: May 16, 2023
 *      Author: Asmod
 */

#include <math.h>
#include <QMC5883L.h>

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
    uint8_t Txcmd [2];
	uint8_t *pTxcmd = &Txcmd[0];

    pCompass->devAddr = COMPASS_ADDR;

    /* Configure CRA */
    temp = (compassCfg.averaging    << COMPASS_CRA_MA0)  |
           (compassCfg.output_rate  << COMPASS_CRA_DO0)  |
           (compassCfg.bias_control << COMPASS_CRA_MS0);

    commandGenTx(COMPASS_CR1, temp, pTxcmd);

    I2C_MasterTx(pI2CHandle, pTxcmd, 2, pCompass->devAddr);

    /* Configure CRB */
    temp = (compassCfg.gain_control << COMPASS_CRB_GN);

    commandGenTx(COMPASS_CR2, temp, pTxcmd);

    I2C_MasterTx(pI2CHandle, pTxcmd, 2, pCompass->devAddr);

    /* Configure Mode */
    temp = (compassCfg.op_mode << COMPASS_MODE_MDO) |
           (compassCfg.hsbit   << COMPASS_MODEHS);

	commandGenTx(COMPASS_CR2, temp, pTxcmd);

    I2C_MasterTx(pI2CHandle, pTxcmd, 2, pCompass->devAddr);

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

	uint8_t data[6];
	uint8_t *pData = &data[0];

	uint8_t Rxcmd[2];
	uint8_t *pRxcmd = &Rxcmd[0];

	uint8_t temp = 0;

	volatile uint8_t dataflag = 0;

	/* Check status register for data ready flag. */
	while(!dataflag) { 
		commandGenRx(COMPASS_SR, pRxcmd);

		I2C_MasterTx(pI2CHandle, pRxcmd, 2, COMPASS_ADDR);
		I2C_MasterRx(pI2CHandle, &temp,  1, COMPASS_ADDR);

		dataflag = (temp >> COMPASS_SR_RDY) & 0x01;

		}

	/* Read in axis data values */
	for (int iter = 0; iter < 6; iter++) {
		commandGenRx(COMPASS_DR+iter, pRxcmd);
		
		I2C_MasterTx(pI2CHandle,  pRxcmd,         2, COMPASS_ADDR);
		I2C_MasterRx(pI2CHandle, (pData + iter), 1, COMPASS_ADDR);

	}
	  
	/* Store X data */
	pCompass->compDat->x_msmnt = (int16_t) ((*(pData + 0) & 0xFF00) << 8) |
										   ((*(pData + 1) & 0x00FF) << 0);

	/* Store Y data */
	pCompass->compDat->y_msmnt = (int16_t) ((*(pData + 4) & 0xFF00) << 8) |
										   ((*(pData + 5) & 0x00FF) << 0);

	/* Store Z data */
	pCompass->compDat->z_msmnt = (int16_t) ((*(pData + 2) & 0xFF00) << 8) |
										   ((*(pData + 3) & 0x00FF) << 0);
}

/**
 * @brief Fills a 2 byte array consisting of a read byte,
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

	*(Txcmd + 0) = address;
	*(Txcmd + 1) = txDat;

}
