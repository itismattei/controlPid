/*
 * motpwm.h
 *
 *  Created on: 10/apr/2016
 *      Author: itismattei
 */

#ifndef MOTPWM_H_
#define MOTPWM_H_

#include "stdint.h"

class PWM_MOTORI{
public:
	PWM_MOTORI(){numPwm ++;}
	~PWM_MOTORI(){if (numPwm > 0) numPwm --;}

	void Init();
	void pwmPower();
	///properties
	static int numPwm;
	int NCont;
	int delta;
	uint32_t numPin;
};



#endif /* MOTPWM_H_ */
