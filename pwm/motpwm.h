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
	/// static properties
	static int numPwm;

public:
	PWM_MOTORI(){numPwm ++; direction = 1;}
	virtual ~PWM_MOTORI(){if (numPwm > 0) numPwm --;}

	virtual void Init();
	virtual void pwmPower();
	void MotorGo();
	virtual void MotorStop();
	inline void setDir(int dir){ direction = dir;}

	///properties
	int NCont;
	int delta;
	/// 1 avanti; -1 indietro
	int direction;
	uint32_t numPin;
};


///
///
class PWM_SERVI : public PWM_MOTORI{

public:
	/// static properties
	static int numServi;


public:
	PWM_SERVI(){numServi++; direction = 1;}
	~PWM_SERVI(){if (numServi > 0) numServi--;}
	void Init();
	void MotorGo(int16_t);

	uint32_t convertDeg2Pwm(int16_t gradi);

};


#endif /* MOTPWM_H_ */
