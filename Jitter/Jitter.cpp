/*
 * Jitter.cpp
 *
 *  Created on: 15/mar/2017
 *      Author: itismattei
 */

#include "Jitter.h"
#include "driverlib/timer.h"


Jitter::Jitter() {
	// TODO Auto-generated constructor stub
	jitter_timerGyro = prevValueGyro = 0;
	jitter_timerPID = prevValuePID = 0;
}

Jitter::~Jitter() {
	// TODO Auto-generated destructor stub
}

