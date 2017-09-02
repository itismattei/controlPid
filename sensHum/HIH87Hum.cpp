/*
 * HIH87Hum.cpp
 *
 *  Created on: 02 set 2017
 *      Author: lenovoMax
 */

#include "HIH87Hum.h"
#include "stdlib.h"

HIH8_7Hum::HIH8_7Hum() {
	// TODO Auto-generated constructor stub
	i2cPtr = NULL;
}


///
/// collega il sensore alla classe che gestisce la comunicazione I2C
void HIH8_7Hum::attachI2C(I2C *p, uint8_t sa){
	i2cPtr = p;
	i2cPtr->I2CSetSlave_Add(sa);
}
