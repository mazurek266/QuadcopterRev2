/*
 * externalVariables.h
 *
 *  Created on: Sep 15, 2021
 *      Author: reks
 */

#ifndef INC_EXTERNALVARIABLES_H_
#define INC_EXTERNALVARIABLES_H_

#include "BMP280.h"
#include "RCFilter.h"

extern bmp280_calib_data bmp280CalibData;

extern RCFilter accLpf[3];
extern RCFilter gyrLpf[3];
extern RCFilter magLpf[3];
extern RCFilter barLpf;

#endif /* INC_EXTERNALVARIABLES_H_ */
