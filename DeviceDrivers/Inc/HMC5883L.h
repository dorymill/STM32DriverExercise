/*
 * HMC5883L.h
 *
 *  Created on: May 16, 2023
 *      Author: Asmod
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

/* Device address */
#define COMPASS_ADDR             0x1E

/* Read/write command bytes */
#define COMPASS_REG_READ         0x3D
#define COMPASS_REG_WRITE        0x3C

/* Register number definitions */
#define COMPASS_CFGA             0
#define COMPASS_CFGB             1
#define COMPASS_MODE             2
#define COMPASS_XMSB             3
#define COMPASS_XLSB             4
#define COMPASS_ZMSB             5
#define COMPASS_ZLSB             6
#define COMPASS_YMSB             7
#define COMPASS_YLSB             8
#define COMPASS_SR               9
#define COMPASS_IDA              10
#define COMPASS_IDB              11
#define COMPASS_IDC              12

#endif /* HMC5883L_H_ */
