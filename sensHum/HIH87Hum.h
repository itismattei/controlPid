/*
 * HIH87Hum.h
 *
 *  Created on: 02 set 2017
 *      Author: lenovoMax
 */

/*
 * questa classe serve per verificare il funzionamento del sensore di temperatura e umidità della Honeywell
 * modello: HIH8000 e HIH7000
 *
 *
 * */

#include "../I2C/i2cTiva.h"
#include <stdint.h>

#ifndef SENSHUM_HIH87HUM_H_
#define SENSHUM_HIH87HUM_H_

class HIH8_7Hum {
public:
	HIH8_7Hum();

	void attachI2C(I2C *, uint8_t sa);

	void readRaw();
	void convertRaw();
    void newData();
    void toString();
    float hum;
    float temp;
    char humidity[16];
    char temperature[16];


private:
	HIH8_7Hum & convertToHum();
	HIH8_7Hum & convertToTemp();

public:

	/// proprieta'
	I2C *i2cPtr;
	uint8_t buff[8];
	uint8_t temper[16], umid[16];
	bool dataValid, dataOld;
};

#endif /* SENSHUM_HIH87HUM_H_ */
