/*
 * allineamento.h
 *
 *  Created on: 20/apr/2015
 *      Author: massimo
 */

#ifndef ALLINEAMENTO_H_
#define ALLINEAMENTO_H_

#include "../adc/adc.h"
#include "../pid.h"
#include <cmath>

class allineamento{

public:
	/// costruttori
	allineamento();
	/// distruttore
	virtual ~allineamento();
	///
	/// metodi
	///
	void adc_allinea(distanza *DIST, pwm *PWM, pid *C);
	void gyro_allinea();

private:
	int valore;
	distanza *dPtr;
	pwm *p;
	pid *cPtr;

};




#endif /* ALLINEAMENTO_H_ */
