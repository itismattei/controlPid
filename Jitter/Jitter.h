/*
 * Jitter.h
 *
 *  Created on: 15/mar/2017
 *      Author: massimo
 */

#ifndef JITTER_H_
#define JITTER_H_

#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/timer.h"

class Jitter {
public:
	Jitter();
	virtual ~Jitter();

	inline uint32_t setActualGyro() {
		jitter_timerGyro = TimerValueGet(WTIMER2_BASE, TIMER_A);
		return jitter_timerGyro;
	}

	inline uint32_t setActualPID(){
		jitter_timerPID = TimerValueGet(WTIMER2_BASE, TIMER_A);
		return jitter_timerPID;

	}

	uint32_t jitter_timerGyro, prevValueGyro;
	uint32_t jitter_timerPID, prevValuePID;
};

#endif /* JITTER_H_ */
