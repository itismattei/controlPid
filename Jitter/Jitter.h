/*
 * Jitter.h
 *
 *  Created on: 15/mar/2017
 *      Author: itismattei
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
		/// salva il precedente valore del contatore di TIMER2
		prevValueGyro = jitter_timerGyro;
		/// aggiorna l'attuale valore del contatore di TIMER2
		jitter_timerGyro = TimerValueGet(WTIMER2_BASE, TIMER_A);
		/// ne restituisce il valore
		return jitter_timerGyro;
	}

	inline uint32_t setActualPID(){
		prevValuePID = jitter_timerPID;
		jitter_timerPID = TimerValueGet(WTIMER2_BASE, TIMER_A);
		return jitter_timerPID;

	}

	uint32_t jitter_timerGyro, prevValueGyro;
	uint32_t jitter_timerPID, prevValuePID;
};

#endif /* JITTER_H_ */
