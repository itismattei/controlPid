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
#include "../distMis.h"
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
	void adc_allinea(distMis &DIST, pwm *PWM, pid *C);
	void gyro_allinea();

private:
	int 	valore;
	float 	angoloCalc1, angoloCalc2;
	float 	distMDX, distMSX, distAssi;
	bool  	dx;
	pwm *p;
	pid *cPtr;

private:
	void allinea(void);


};




#endif /* ALLINEAMENTO_H_ */
