/*
 * HIH87Hum.h
 *
 *  Created on: 02 set 2017
 *      Author: lenovoMax
 */

/*
 * questa classe serve per verificare il funzionamento del sensore di temepratura e umidità della Honeywell
 * modello: HIH8000 e HIH7000
 *
 *
 * */

#include "../I2C/i2cTiva.h"

#ifndef SENSHUM_HIH87HUM_H_
#define SENSHUM_HIH87HUM_H_

class HIH8_7Hum {
public:
	HIH8_7Hum();

	void attachI2C(I2C *, uint8_t sa);

	/// proprieta'
	I2C *i2cPtr;
};

#endif /* SENSHUM_HIH87HUM_H_ */
